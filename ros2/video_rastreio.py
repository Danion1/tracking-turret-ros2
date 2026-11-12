#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video_rastreio.py — os dois GIFs do README do rastreio (2026-11-11). Gazebo sem janela.

Quem detecta, segue e dispara são os nós do Daniel (detectar_alvo, seguir_alvo, comando_torreta).
Este roteiro só: cria as câmeras de filmagem, grava tudo quadro a quadro (nada de lista de quadros na
memória) e pede o disparo (/torreta/disparar) num instante escolhido.

POR QUE o disparo é no ponto de retorno do vaivém: ainda não há previsão (Kalman, passo 2). Do pedido ao
lançamento a sequência leva ~1,03 s (volantes 0,71 + empurrão 0,32); o roteiro pede o disparo 0,97 s
depois de o drone cruzar o centro, para o dardo sair quando o drone está parado na ponta (y = −1 m).
O vídeo e o README têm de dizer isso.

  gravar  <pasta> <segundos_de_parede>   webcam + câmeras de filmagem → <pasta>/<câmera>.mp4 + .json
  roteiro <pasta>                         câmeras, disparo, verdade do drone em <pasta>/verdade.jsonl
  montar  <pasta> <dardo.jsonl> <midia>   rastreio_webcam.{mp4,gif} e rastreio_tiro.{mp4,gif}
"""
import json
import math
import os
import sys
import threading
import time

FPS = 30
CAMERAS = {
    # a webcam simulada da torreta (nasce com o robô; aqui só é gravada)
    "webcam": dict(topico="/webcam/image_raw", hz=30, criar=False),
    # filmagem: de lado da linha de tiro (torreta na origem → drone na ponta, (3, −1, 0,8)); enquadramento pela
    # conta (f = 640/tan(0,65) = 842 px): torreta em x ≈ 211 px, drone na ponta em x ≈ 1112 px, ~6° acima do centro
    "cam_larga": dict(em=(1.20, -3.40, 0.55), alvo=(1.60, -0.40, 0.45), fov=1.30, res=(1280, 720), hz=30),
    "cam_perto": dict(em=(0.50, -0.46, 0.36), alvo=(0.01, 0.0, 0.14), fov=0.80, res=(640, 360), hz=30),
    "cam_tiro": dict(em=(1.20, -3.40, 0.55), alvo=(1.60, -0.40, 0.45), fov=1.30, res=(1280, 720), hz=240),
}
ATE_LANCAR = 1.03      # s simulados do pedido de disparo ao lançamento (testar_disparo.sh: 0,71 + 0,32)
QUARTO = 2.0           # s: um quarto do período do vaivém (8 s), do centro até a ponta


def gravar(pasta, segundos):
    import cv2
    import numpy as np
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node
    os.makedirs(pasta, exist_ok=True)
    trava = threading.Lock()
    escritores, carimbos = {}, {n: [] for n in CAMERAS}

    def fazer_cb(nome):
        def cb(m):
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)[:, :, ::-1]
            with trava:
                if nome not in escritores:
                    escritores[nome] = cv2.VideoWriter(os.path.join(pasta, nome + ".mp4"), cv2.VideoWriter_fourcc(*"avc1"), FPS, (m.width, m.height))
                escritores[nome].write(np.ascontiguousarray(img))
                carimbos[nome].append(m.header.stamp.sec + m.header.stamp.nsec * 1e-9)
        return cb

    n = Node()
    for nome, c in CAMERAS.items():
        n.subscribe(Image, c.get("topico", "/" + nome), fazer_cb(nome))
    fim = time.time() + segundos
    while time.time() < fim and not os.path.exists(os.path.join(pasta, "PARAR")):
        time.sleep(0.2)
    time.sleep(1.0)
    with trava:
        for nome, e in escritores.items():
            e.release()
            json.dump(carimbos[nome], open(os.path.join(pasta, nome + ".json"), "w"))
            c = carimbos[nome]
            hz = (len(c) - 1) / (c[-1] - c[0]) if len(c) > 1 else 0
            buracos = sum(1 for a, b in zip(c, c[1:]) if b - a > 1.5 / CAMERAS[nome]["hz"])
            print(f"  {nome}: {len(c)} quadros, {c[0]:.3f} a {c[-1]:.3f} s ({hz:.0f} Hz; pedido {CAMERAS[nome]['hz']}), buracos {buracos}")
    sys.stdout.flush()
    os._exit(0)          # o gz transport derruba o Python na saída (GIL), depois de tudo gravado


def roteiro(pasta):
    import rclpy
    from geometry_msgs.msg import PointStamped
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.entity_pb2 import Entity
    from gz.transport13 import Node as GzNode
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import Empty
    from video_torreta import camera_sdf
    rclpy.init()
    no = Node("roteiro_rastreio", parameter_overrides=[Parameter("use_sim_time", value=True)])
    pub_disp = no.create_publisher(Empty, "/torreta/disparar", 10)
    gz = GzNode()
    verdade = open(os.path.join(pasta, "verdade.jsonl"), "w")
    estado = {"y": None, "cruzou": None, "desde": None}   # desde: só vale cruzar depois de o rastreio convergir

    def cb_verdade(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        verdade.write(json.dumps({"t": t, "p": [m.point.x, m.point.y, m.point.z]}) + "\n")
        y = m.point.y
        if estado["y"] is not None and estado["y"] > 0 >= y and estado["cruzou"] is None \
                and estado["desde"] is not None and t >= estado["desde"]:
            estado["cruzou"] = t                       # centro, indo para y negativo
        estado["y"] = y
    no.create_subscription(PointStamped, "/alvo/verdade", cb_verdade, 100)

    def criar(nome):
        m = EntityFactory()
        m.sdf = '<?xml version="1.0"?><sdf version="1.9">' + camera_sdf(nome, CAMERAS[nome]) + "</sdf>"
        gz.request("/world/torreta/create", m, EntityFactory, Boolean, 3000)

    def tirar(nome):
        m = Entity()
        m.name, m.type = nome, Entity.MODEL
        gz.request("/world/torreta/remove", m, Entity, Boolean, 3000)

    def em_fio(f, nome):
        threading.Thread(target=f, args=(nome,), daemon=True).start()

    agora = lambda: no.get_clock().now().nanoseconds * 1e-9  # noqa: E731
    while rclpy.ok() and (pub_disp.get_subscription_count() == 0 or agora() == 0):
        rclpy.spin_once(no, timeout_sec=0.2)
    em_fio(criar, "cam_larga")
    em_fio(criar, "cam_perto")
    estado["desde"] = agora() + 4.0      # o rastreio leva ~3,5 s para convergir (testar_rastreio.sh, 2026-11-11)
    print(f"roteiro: câmeras criadas em t = {agora():.3f} s; disparo só depois de t = {estado['desde']:.3f} s")
    while rclpy.ok() and estado["cruzou"] is None:
        rclpy.spin_once(no, timeout_sec=0.01)
    t_disp = estado["cruzou"] + QUARTO - ATE_LANCAR
    print(f"  drone no centro em t = {estado['cruzou']:.3f} s; disparo pedido para t = {t_disp:.3f} s")
    passos = [(t_disp - 0.3, "criar cam_tiro"), (t_disp, "disparar"), (t_disp + 1.5, "tirar cam_tiro"), (t_disp + 3.0, "fim")]
    for t, acao in passos:
        while rclpy.ok() and agora() < t:
            rclpy.spin_once(no, timeout_sec=0.005)
        if acao == "criar cam_tiro":
            em_fio(criar, "cam_tiro")
        elif acao == "tirar cam_tiro":
            em_fio(tirar, "cam_tiro")
        elif acao == "disparar":
            pub_disp.publish(Empty())
        print(f"  t = {agora():.3f} s: {acao}")
    verdade.close()
    no.destroy_node()
    rclpy.try_shutdown()


def montar(pasta, dardo_jsonl, midia):
    import cv2
    import numpy as np
    from PIL import Image as PImage, ImageDraw, ImageFont
    fonte = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    F1, F2 = ImageFont.truetype(fonte, 26), ImageFont.truetype(fonte, 18)
    W, H = 1280, 720

    def quadros(nome, ti, tf):
        c = json.load(open(os.path.join(pasta, nome + ".json")))
        cap = cv2.VideoCapture(os.path.join(pasta, nome + ".mp4"))
        n = 0
        try:
            for t in c:
                if t > tf:
                    break
                ok, q = cap.read()
                if not ok:
                    break
                if t >= ti:
                    n += 1
                    yield t, q
        finally:
            cap.release()
            print(f"  {nome} [{ti:.3f}, {tf:.3f}] s: {n} quadros")

    # lançamento e falta (dardo × drone pela verdade, nos mesmos instantes)
    D = [json.loads(x) for x in open(dardo_jsonl)]
    D = [a for i, a in enumerate(D) if i == 0 or a["t"] > D[i - 1]["t"]]
    tl = next(a["t"] for a, b in zip(D, D[1:]) if math.dist(a["d"]["p"], b["d"]["p"]) / (b["t"] - a["t"]) > 2.0)
    V = [json.loads(x) for x in open(os.path.join(pasta, "verdade.jsonl"))]
    vt = np.array([v["t"] for v in V])
    vp = np.array([v["p"] for v in V])
    perto_ = min(((np.array(a["d"]["p"]) - [np.interp(a["t"], vt, vp[:, k]) for k in range(3)], a["t"]) for a in D
                  if vt[0] <= a["t"] <= vt[-1]), key=lambda x: np.linalg.norm(x[0]))
    dd, t_imp = perto_
    falta = float(np.linalg.norm(dd))
    R_D = 0.0064                                         # raio do dardo; corpo do drone 60 × 60 × 25 mm
    acertou = abs(dd[0]) <= 0.030 + R_D and abs(dd[1]) <= 0.030 + R_D and abs(dd[2]) <= 0.0125 + R_D
    print(f"  lançamento em t = {tl:.3f} s; maior aproximação em t = {t_imp:.3f} s: {1000 * falta:.0f} mm do centro "
          f"(x {1000 * dd[0]:+.0f}, y {1000 * dd[1]:+.0f}, z {1000 * dd[2]:+.0f} mm); acertou o corpo: {'SIM' if acertou else 'não'}")
    res = ("HIT: the dart strikes the drone's body and drops" if acertou
           else f"Miss: the dart passes {1000 * falta:.0f} mm from the drone's center")

    def gif_de(lista, arq, passo_ms):
        lista[0].save(arq, save_all=True, append_images=lista[1:], duration=passo_ms, loop=0, optimize=True)
        print(f"  {arq}: {len(lista)} quadros, {os.path.getsize(arq) / 1e6:.1f} MB")

    # ---- 1. a visão da webcam, LIMPA (sem marca nenhuma): do início do vaivém até o lançamento + 1 s
    cw = json.load(open(os.path.join(pasta, "webcam.json")))
    t_ini = max(tl - 6.0, cw[0])                    # ~6 s de rastreio já convergido antes do tiro
    v1 = cv2.VideoWriter(os.path.join(midia, "rastreio_webcam.mp4"), cv2.VideoWriter_fourcc(*"avc1"), FPS, (640, 360))
    g1, i = [], 0
    for t, q in quadros("webcam", t_ini, tl + 1.0):
        v1.write(q)
        if i % 2 == 0:
            g1.append(PImage.fromarray(cv2.cvtColor(cv2.resize(q, (480, 270)), cv2.COLOR_BGR2RGB)))
        i += 1
    v1.release()
    gif_de(g1, os.path.join(midia, "rastreio_webcam.gif"), 2000 // FPS)
    del g1

    # ---- 2. de fora: larga + close da torreta no canto, tiro em câmera lenta, falta medida
    def legenda(img, titulo, sub):
        p = PImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(p, "RGBA")
        d.rectangle([0, H - 76, W, H], fill=(0, 0, 0, 150))
        d.text((20, H - 70), titulo, font=F1, fill=(255, 255, 255))
        d.text((20, H - 34), sub, font=F2, fill=(225, 225, 225))
        return cv2.cvtColor(np.array(p), cv2.COLOR_RGB2BGR)

    zoom_c = [None]

    def zoom(q):
        """Zoom de 3× no drone (canto superior direito), para o acerto ser visível. Só VISUALIZAÇÃO: acha o laranja
        do corpo no quadro de filmagem (não é a detecção da torreta) e guarda o último centro achado."""
        hsv = cv2.cvtColor(q, cv2.COLOR_BGR2HSV)
        m = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 32) & (hsv[..., 1] > 50) & (hsv[..., 2] > 25)
        m[:280, :480] = False                            # ignora o close da torreta
        ys, xs = np.nonzero(m)
        if len(xs) > 3:
            zoom_c[0] = (int(np.median(xs)), int(np.median(ys)))
        if zoom_c[0] is None:
            return q
        cx_, cy_ = zoom_c[0]
        x0 = min(max(cx_ - 100, 0), W - 200)
        y0 = min(max(cy_ - 40, 0), H - 112)               # o drone fica no alto do recorte: o dardo cai abaixo dele
        rec = cv2.resize(q[y0:y0 + 112, x0:x0 + 200], (600, 336), interpolation=cv2.INTER_NEAREST)
        cv2.rectangle(rec, (0, 0), (599, 335), (255, 255, 255), 2)
        q[16:352, W - 616:W - 16] = rec
        cv2.putText(q, "3x zoom on the drone", (W - 606, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        return q

    v2 = cv2.VideoWriter(os.path.join(midia, "rastreio_tiro.mp4"), cv2.VideoWriter_fourcc(*"avc1"), FPS, (W, H))
    g2, n2 = [], [0]

    def escrever(q):
        v2.write(q)
        if n2[0] % 2 == 0:
            g2.append(PImage.fromarray(cv2.cvtColor(cv2.resize(q, (640, 360)), cv2.COLOR_BGR2RGB)))
        n2[0] += 1

    def com_close(t_fim, titulo, sub, t_ini_):
        perto = quadros("cam_perto", t_ini_, t_fim)
        pa, pb = next(perto), next(perto, None)
        for t, q in quadros("cam_larga", t_ini_, t_fim):
            while pb is not None and abs(pb[0] - t) <= abs(pa[0] - t):
                pa, pb = pb, next(perto, None)
            mini = cv2.resize(pa[1], (448, 252))
            cv2.rectangle(mini, (0, 0), (447, 251), (255, 255, 255), 2)
            q[16:268, 16:464] = mini
            escrever(legenda(q, titulo, sub))
        perto.close()

    com_close(tl - 0.10, "Tracking a drone with the webcam (Gazebo)",
              "HSV detection → pixel error → pan/tilt servos · inset: the turret up close", t_ini)
    for t, q in quadros("cam_tiro", tl - 0.06, tl + 0.30):
        escrever(legenda(zoom(q), "Shot, 8× slow motion",
                         res + " · fired at the end of the sweep (no prediction yet)"))
    com_close(tl + 1.0, "Tracking a drone with the webcam (Gazebo)",
              "HSV detection → pixel error → pan/tilt servos · inset: the turret up close", tl + 0.30)
    v2.release()
    print(f"  rastreio_tiro.mp4: {n2[0]} quadros = {n2[0] / FPS:.1f} s")
    gif_de(g2, os.path.join(midia, "rastreio_tiro.gif"), 2000 // FPS)
    del g2

    # ---- 3. de fora, em TEMPO REAL (sem câmera lenta): o mesmo plano, do início ao fim; o resultado entra
    #         na legenda a partir do impacto
    v2 = cv2.VideoWriter(os.path.join(midia, "rastreio_tiro_tempo_real.mp4"), cv2.VideoWriter_fourcc(*"avc1"), FPS, (W, H))
    g2, n2[0] = [], 0
    titulo = "Tracking a drone with the webcam (Gazebo)"
    sub = "HSV detection → pixel error → pan/tilt servos · inset: the turret up close"
    perto = quadros("cam_perto", t_ini, tl + 1.5)
    pa, pb = next(perto), next(perto, None)
    for t, q in quadros("cam_larga", t_ini, tl + 1.5):
        while pb is not None and abs(pb[0] - t) <= abs(pa[0] - t):
            pa, pb = pb, next(perto, None)
        mini = cv2.resize(pa[1], (448, 252))
        cv2.rectangle(mini, (0, 0), (447, 251), (255, 255, 255), 2)
        q[16:268, 16:464] = mini
        if t >= tl - 0.2:
            q = zoom(q)
        if t >= t_imp:
            escrever(legenda(q, res, "real time · fired at the end of the sweep (no prediction yet)"))
        elif t >= tl - 1.0:
            escrever(legenda(q, "Firing: flywheels spin up, the rack pushes the dart", sub))
        else:
            escrever(legenda(q, titulo, sub))
    perto.close()
    v2.release()
    print(f"  rastreio_tiro_tempo_real.mp4: {n2[0]} quadros = {n2[0] / FPS:.1f} s")
    gif_de(g2, os.path.join(midia, "rastreio_tiro_tempo_real.gif"), 2000 // FPS)


if __name__ == "__main__":
    a = sys.argv
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    if a[1] == "gravar":
        gravar(a[2], float(a[3]))
    elif a[1] == "roteiro":
        roteiro(a[2])
    elif a[1] == "montar":
        montar(a[2], a[3], a[4])
