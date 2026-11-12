#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
conferir_camera.py — conferência da webcam simulada (2026-11-10). NÃO é detector.

Pergunta: o alvo aparece na imagem EXATAMENTE onde a geometria diz?  Para cada quadro de /webcam/image_raw:
  1. pega a posição VERDADEIRA do alvo (/alvo/verdade, interpolada no carimbo do quadro);
  2. leva para o frame da câmera pelo TF (world → webcam_optico, no mesmo carimbo);
  3. projeta com o modelo pinhole da /webcam/camera_info:  u = fx·X/Z + cx,  v = fy·Y/Z + cy;
  4. confere a imagem SÓ ali: o pixel previsto é laranja? E o centro dos pixels laranja numa janela de
     41 × 41 em volta da previsão está a quantos pixels dela?
Achar o alvo SEM a verdade (a imagem inteira, sem saber onde olhar) é a detecção, e essa é do Daniel.

No meio do teste, publica um alvo de mira em /torreta/alvo (entrada do comando_torreta do Daniel): a câmera
se move com o berço, e a projeção tem de continuar batendo (prova que a câmera está presa no lugar certo).

Saída: números no terminal + um vídeo da câmera com a previsão desenhada (círculo verde), escrito
quadro a quadro (nada de lista de quadros na memória: ver a queda do WSL de 2026-11-10).

Uso: python3 conferir_camera.py <segundos_simulados> <saida.mp4> [mira_pan_graus mira_tilt_graus] [--detectar] [--seguir]
  --detectar  compara o /alvo/pixel do Daniel com a verdade projetada, no mesmo carimbo de imagem
  --seguir    mede o rastreio: ângulo entre o alvo (verdade) e o centro da imagem, por trecho de tempo
  --salvar-quadro <arq.png>  grava o 60º quadro cru (sem a compressão do MP4)
  --disparar T  publica /torreta/disparar em t = T s;  --dardo <arq.jsonl>  mede a falta (gravar_dardo.py)
"""
import bisect
import math
import sys

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, Float64MultiArray
from tf2_ros import Buffer, TransformListener


def laranja_hsv(hsv):
    """Máscara do laranja do alvo numa imagem HSV do OpenCV (tom 0–180). MEDIDO no 1º teste (2026-11-10):
    visto de baixo, com o sol por cima, o laranja (1,0; 0,35; 0,0) chega MARROM: tom 11–27 (mediana 18),
    saturação 61–255 (mediana 173), brilho 40–146 (mediana 83). O fundo (céu e chão) tem saturação ~4.
    Por isso o critério é tom + saturação, não "R alto": um limiar em RGB errou 100% dos quadros."""
    t, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return (t >= 5) & (t <= 32) & (sat > 50) & (val > 25)


class Conferir(Node):
    def __init__(self, duracao, saida, mira):
        super().__init__("conferir_camera", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.duracao, self.mira = duracao, mira
        self.tf = Buffer(cache_time=Duration(seconds=30))
        TransformListener(self.tf, self)
        self.info = None
        self.verdade_t, self.verdade_p = [], []
        self.linhas = []                # (t, u, v, dentro, laranja_no_previsto, desvio_px, fase)
        self.t0 = None
        self.mirou = False
        self.pub_mira = self.create_publisher(Float64MultiArray, "/torreta/alvo", 10)
        self.pub_disp = self.create_publisher(Empty, "/torreta/disparar", 10)
        self.t_disparo = float(arg_de("--disparar", "-1"))
        self.disparou = False
        self.video = cv2.VideoWriter(saida, cv2.VideoWriter_fourcc(*"avc1"), 30, (640, 360))
        self.carimbos, self.frame_ids, self.encodings = [], set(), set()
        self.create_subscription(CameraInfo, "/webcam/camera_info", self.cb_info, 10)
        self.create_subscription(PointStamped, "/alvo/verdade", self.cb_verdade, 100)
        self.create_subscription(Image, "/webcam/image_raw", self.cb_imagem, 10)
        self.pixel = {}                 # carimbo (ns) → (u, v, área) do /alvo/pixel do Daniel
        self.previsto = {}              # carimbo (ns) → (u, v) pela verdade
        self.create_subscription(PointStamped, "/alvo/pixel", self.cb_pixel, 100)
        self.fim = False

    def cb_pixel(self, m):
        self.pixel[Time.from_msg(m.header.stamp).nanoseconds] = (m.point.x, m.point.y, m.point.z)

    def cb_info(self, m):
        self.info = m

    def cb_verdade(self, m):
        self.verdade_t.append(Time.from_msg(m.header.stamp).nanoseconds * 1e-9)
        self.verdade_p.append((m.point.x, m.point.y, m.point.z))

    def verdade_em(self, t):
        i = bisect.bisect_left(self.verdade_t, t)
        if i == 0 or i >= len(self.verdade_t):
            return None
        t0, t1 = self.verdade_t[i - 1], self.verdade_t[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return [p0 + a * (p1 - p0) for p0, p1 in zip(self.verdade_p[i - 1], self.verdade_p[i])]

    def cb_imagem(self, m):
        if self.fim or self.info is None:
            return
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        rel = t - self.t0
        if self.mira and not self.mirou and rel >= self.duracao / 2:
            self.pub_mira.publish(Float64MultiArray(data=list(self.mira)))
            self.mirou = True
            self.get_logger().info(f"mira {self.mira} publicada em t = {rel:.2f} s")
        if 0 <= self.t_disparo <= rel and not self.disparou:
            self.pub_disp.publish(Empty())
            self.disparou = True
            self.get_logger().info(f"disparo pedido em t = {rel:.2f} s")
        self.carimbos.append(t)
        self.frame_ids.add(m.header.frame_id)
        self.encodings.add(f"{m.encoding} {m.width}x{m.height}")
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)[:, :, ::-1].copy()   # rgb8 → BGR
        if arg_de("--salvar-quadro", "") and len(self.carimbos) == 60:          # quadro CRU, sem compressão (PNG)
            cv2.imwrite(arg_de("--salvar-quadro", ""), img)
        p = self.verdade_em(t)
        try:
            tr = self.tf.lookup_transform("webcam_optico", "world", Time(seconds=t, clock_type=self.get_clock().clock_type))
        except Exception:                                    # noqa: BLE001  (TF ainda sem esse instante)
            tr = None
        if p is not None and tr is not None:
            q = tr.transform.rotation
            w, x, y, z = q.w, q.x, q.y, q.z
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            tv = tr.transform.translation
            X, Y, Z = R @ np.array(p) + np.array([tv.x, tv.y, tv.z])
            K = self.info.k
            u, v = K[0] * X / Z + K[2], K[4] * Y / Z + K[5]
            dentro = Z > 0 and 0 <= u < m.width and 0 <= v < m.height
            lar, desvio = False, None
            if dentro:
                ui, vi = int(round(u)), int(round(v))
                u0, v0 = max(ui - 20, 0), max(vi - 20, 0)
                mask = laranja_hsv(cv2.cvtColor(img[v0:vi + 21, u0:ui + 21], cv2.COLOR_BGR2HSV))
                lar = bool(mask[max(vi - v0 - 1, 0):vi - v0 + 2, max(ui - u0 - 1, 0):ui - u0 + 2].any())
                if mask.any():
                    ys, xs = np.nonzero(mask)
                    desvio = math.hypot(xs.mean() + u0 - u, ys.mean() + v0 - v)
                cv2.circle(img, (ui, vi), 14, (0, 220, 0), 2)
            self.linhas.append((rel, u, v, dentro, lar, desvio, "depois" if self.mirou else "antes"))
            if dentro:
                self.previsto[Time.from_msg(m.header.stamp).nanoseconds] = (u, v, float(Z))
        cv2.putText(img, f"t = {rel:5.2f} s   verde = onde a conta diz que o alvo esta", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        self.video.write(img)
        if rel >= self.duracao:
            self.fim = True

    def relatorio(self):
        self.video.release()
        c = self.carimbos
        print("== webcam simulada")
        if len(c) < 2:
            print("  quase nenhum quadro chegou")
            return 1
        hz = (len(c) - 1) / (c[-1] - c[0])
        buracos = sum(1 for a, b in zip(c, c[1:]) if b - a > 1.5 / 30)
        print(f"  {len(c)} quadros em {c[-1] - c[0]:.2f} s simulados = {hz:.1f} Hz (pedido 30); buracos: {buracos}")
        print(f"  frame_id {sorted(self.frame_ids)}; imagem {sorted(self.encodings)}")
        K = self.info.k
        fx_conta = 320 / math.tan(1.2289 / 2)
        print(f"  camera_info: fx {K[0]:.1f}, fy {K[4]:.1f}, cx {K[2]:.1f}, cy {K[5]:.1f} px  (conta: fx = fy = 320/tan(35,2°) = {fx_conta:.1f})")
        print(f"  campo de visão vertical pela camera_info: {math.degrees(2 * math.atan(180 / K[4])):.1f}° (conta: 43,3°)")
        ok = 0
        for fase in ("antes", "depois"):
            L = [x for x in self.linhas if x[6] == fase]
            if not L:
                continue
            dentro = [x for x in L if x[3]]
            lar = [x for x in dentro if x[4]]
            des = [x[5] for x in dentro if x[5] is not None]
            print(f"  {fase} da mira: {len(L)} quadros com verdade e TF; alvo dentro da imagem em {len(dentro)}; "
                  f"laranja no pixel previsto em {len(lar)} ({100 * len(lar) / max(len(dentro), 1):.1f}%)")
            if des:
                print(f"    desvio do centro laranja à previsão: média {np.mean(des):.2f} px, máx. {np.max(des):.2f} px "
                      f"(1 px a 3 m ≈ {3000 / K[0]:.1f} mm)")
            ok += len(dentro) > 0 and len(lar) == len(dentro)
        if self.pixel or "--detectar" in sys.argv:
            print("== detecção do Daniel (/alvo/pixel) contra a verdade projetada")
            comuns = [k for k in self.previsto if k in self.pixel]
            print(f"  quadros com o alvo dentro da imagem: {len(self.previsto)}; com /alvo/pixel no mesmo carimbo: {len(comuns)} "
                  f"({100 * len(comuns) / max(len(self.previsto), 1):.1f}%); /alvo/pixel recebidos: {len(self.pixel)}")
            if comuns:
                e = [math.hypot(self.pixel[k][0] - self.previsto[k][0], self.pixel[k][1] - self.previsto[k][1]) for k in comuns]
                a = [self.pixel[k][2] for k in comuns]
                print(f"  erro (pixel detectado − previsto): média {np.mean(e):.2f} px, p95 {np.percentile(e, 95):.2f}, máx. {np.max(e):.2f} px")
                print(f"  point.z (largura do contorno desde 2026-11-11): {np.min(a):.0f} a {np.max(a):.0f}, mediana {np.median(a):.0f}")
                # distância estimada pelo tamanho (d = fx·0,060/largura) contra a profundidade real no frame da câmera
                fx = self.info.k[0]
                ed = [(fx * 0.060 / self.pixel[k][2], self.previsto[k][2]) for k in comuns if self.pixel[k][2] >= 2]
                if ed:
                    rel_ = [100 * (e - r) / r for e, r in ed]
                    print(f"  distância pela largura (fx·0,060/largura) × real: real {np.median([r for _, r in ed]):.2f} m, "
                          f"estimada mediana {np.median([e for e, _ in ed]):.2f} m, erro {np.median(rel_):+.1f}% (de {min(rel_):+.0f} a {max(rel_):+.0f}%)")
        if "--seguir" in sys.argv:
            print("== rastreio: distância do alvo (verdade) ao centro da imagem, em ângulo")
            L = [x for x in self.linhas if x[3]]
            fx = self.info.k[0]
            for a, b in ((0, 2), (2, 4), (4, 7), (7, self.duracao + 1)):
                d = [math.degrees(math.atan(math.hypot(x[1] - 320, x[2] - 180) / fx)) for x in L if a <= x[0] < b]
                if d:
                    print(f"  t {a:4.1f}–{min(b, self.duracao):4.1f} s: média {np.mean(d):5.2f}°, máx. {np.max(d):5.2f}°  ({len(d)} quadros)")
            print("  linha do tempo (a cada ~0,5 s): t | v previsto | /alvo/pixel v, largura")
            ks = sorted(self.pixel)
            for x in self.linhas[::15]:
                k = int(round((x[0] + self.t0) * 1e9))
                j = min(ks, key=lambda q: abs(q - k)) if ks else None
                px = self.pixel[j] if j is not None and abs(j - k) < 4e7 else None
                print(f"    {x[0]:5.2f} s | {x[2]:7.1f} {'(fora)' if not x[3] else '':6} | "
                      + (f"{px[1]:6.1f}, {px[2]:3.0f}" if px else "sem detecção"))
            fora = sum(1 for x in self.linhas if not x[3])
            print(f"  quadros com o alvo FORA da imagem: {fora}")
        if arg_de("--dardo", ""):
            self.relatorio_tiro(arg_de("--dardo", ""))
        us = [x[1] for x in self.linhas if x[3]]
        if us:
            print(f"  alvo andou na imagem: u de {min(us):.0f} a {max(us):.0f} px")
        return 0


def arg_de(nome, padrao):
    return sys.argv[sys.argv.index(nome) + 1] if nome in sys.argv else padrao


def relatorio_tiro(self, arq):
    """Acertou? Distância do centro do dardo ao centro do drone (verdade), nos mesmos instantes simulados."""
    import json
    print("== tiro: dardo × drone (verdade), no tempo simulado")
    try:
        L = [json.loads(x) for x in open(arq)]
    except OSError:
        print("  sem gravação do dardo")
        return
    L = [a for i, a in enumerate(L) if i == 0 or a["t"] > L[i - 1]["t"]]
    melhor = None
    for a in L:
        p = self.verdade_em(a["t"])
        if p is None:
            continue
        d = np.array(a["d"]["p"]) - np.array(p)
        if melhor is None or np.linalg.norm(d) < np.linalg.norm(melhor[1]):
            melhor = (a["t"], d, np.array(a["d"]["p"]))
    if melhor is None:
        print("  o dardo não cruzou o tempo da verdade")
        return
    t, d, pd = melhor
    # componentes da falta no plano perpendicular à linha torreta → drone: horizontal (lateral) e vertical
    alvo = pd - d
    eixo = alvo / np.linalg.norm(alvo)
    lat = np.cross([0, 0, 1], eixo)
    lat /= np.linalg.norm(lat)
    vert = np.cross(eixo, lat)
    R_D = 0.0064                       # raio do dardo (Ø12,8, colisão do dardo.sdf)
    corpo = abs(d[0]) <= 0.030 + R_D and abs(d[1]) <= 0.030 + R_D and abs(d[2]) <= 0.0125 + R_D
    drone = abs(d[0]) <= 0.0866 + R_D and abs(d[1]) <= 0.0866 + R_D and abs(d[2]) <= 0.0175 + R_D
    print(f"  maior aproximação em t = {t:.3f} s: {1000 * np.linalg.norm(d):.0f} mm do centro do drone, a {np.linalg.norm(alvo):.2f} m da torreta")
    print(f"  falta: vertical {1000 * float(d @ vert):+.0f} mm (+ = acima), lateral {1000 * float(d @ lat):+.0f} mm (+ = à esquerda da torreta)")
    print(f"  acertaria o corpo (60 × 60 × 25 mm)? {'SIM' if corpo else 'não'};  o drone (173 × 173 × 35 mm, hélices)? {'SIM' if drone else 'não'}")


Conferir.relatorio_tiro = relatorio_tiro


def main():
    duracao, saida = float(sys.argv[1]), sys.argv[2]
    nums = []
    for a in sys.argv[3:]:                  # ângulos de mira: só os posicionais ANTES da 1ª opção --
        if a.startswith("--"):
            break
        nums.append(a)
    mira = (float(nums[0]), float(nums[1])) if len(nums) > 1 else None
    rclpy.init()
    no = Conferir(duracao, saida, mira)
    while rclpy.ok() and not no.fim:
        rclpy.spin_once(no, timeout_sec=0.5)
    cod = no.relatorio()
    no.destroy_node()
    rclpy.try_shutdown()
    sys.exit(cod)


if __name__ == "__main__":
    main()
