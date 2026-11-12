#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video_torreta.py — o vídeo dos mecanismos da torreta no Gazebo sem janela (receita "vídeo liso"):
câmeras simuladas, cada quadro carimbado com o tempo da
SIMULAÇÃO, e o Gazebo espera cada quadro; por isso o vídeo sai liso mesmo com a simulação lenta.

Quem COMANDA a torreta é o nó do Daniel (comando_torreta): o roteiro aqui só publica nos tópicos
de entrada dele (/torreta/alvo e /torreta/disparar), como o `ros2 topic pub` dos testes, e cria e tira
as câmeras do mundo (/world/torreta/create e /remove). Cada câmera tem taxa FIXA: o /<câmera>/set_rate
existe mas não muda nada nesta versão (medido em 2026-11-10: 200 Hz antes e depois, pela linha de
comando e pelo Python); por isso a câmera lenta de 1 000 Hz só existe durante o disparo.

  mundo   <entrada.sdf> <saida.sdf>        o mundo com o sistema de sensores (as câmeras nascem pelo roteiro)
  gravar  <pasta> <segundos_de_parede>     grava cada câmera (gz transport) em <pasta>/<câmera>.mp4 + .json
  roteiro                                   a sequência no tempo simulado (mira, mira, centro, disparo)
  montar  <pasta> <dardo.jsonl> <saida.mp4> [saida.gif]   o filme final, com legendas
"""
import json
import math
import os
import sys
import threading
import time

FPS = 30
# câmeras: posição, alvo (m, mundo), campo de visão (rad), resolução, taxa (Hz de simulação)
LANC = dict(em=(0.035, -0.13, 0.32), alvo=(0.035, 0.0, 0.163), fov=0.78, res=(1280, 720))
CAMERAS = {
    "cam_geral": dict(em=(0.50, -0.46, 0.36), alvo=(0.01, 0.0, 0.12), fov=0.80, res=(1280, 720), hz=30),
    "cam_pan": dict(em=(0.07, 0.23, 0.11), alvo=(0.008, 0.035, 0.050), fov=0.55, res=(640, 360), hz=30),
    "cam_lancador": dict(LANC, hz=1000),
    "cam_lanc30": dict(LANC, hz=30),
    "cam_voo": dict(em=(1.90, -3.30, 0.55), alvo=(1.90, 0.0, 0.12), fov=1.25, res=(1280, 720), hz=250),
}
# roteiro: (t simulado desde o início, ação)
ROTEIRO = [
    (0.0, ("criar", ["cam_geral", "cam_pan"])),
    (1.0, ("alvo", [45.0, 20.0])),
    (3.0, ("alvo", [-30.0, -10.0])),
    (5.0, ("alvo", [0.0, 0.0])),
    (6.4, ("tirar", ["cam_geral", "cam_pan"])),
    (6.5, ("criar", ["cam_lancador"])),
    (7.0, ("disparar", None)),
    (7.3, ("criar", ["cam_voo"])),
    (8.15, ("criar", ["cam_lanc30"])),
    (8.2, ("tirar", ["cam_lancador"])),
    (8.6, ("tirar", ["cam_voo"])),
    (9.6, ("fim", None)),
]


def camera_sdf(nome, c):
    dx, dy, dz = (a - b for a, b in zip(c["alvo"], c["em"]))
    yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
    x, y, z = c["em"]
    return f"""
    <model name="{nome}">
      <static>true</static>
      <pose>{x} {y} {z} 0 {pitch:.5f} {yaw:.5f}</pose>
      <link name="link">
        <sensor name="{nome}" type="camera">
          <always_on>1</always_on>
          <update_rate>{c['hz']}</update_rate>
          <topic>/{nome}</topic>
          <camera>
            <horizontal_fov>{c['fov']}</horizontal_fov>
            <image><width>{c['res'][0]}</width><height>{c['res'][1]}</height></image>
            <clip><near>0.01</near><far>30</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


def mundo(entrada, saida):
    texto = open(entrada, encoding="utf-8").read()
    extra = ('\n    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
             '<render_engine>ogre2</render_engine></plugin>\n')
    open(saida, "w", encoding="utf-8").write(texto.replace("</world>", extra + "  </world>"))


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
    for nome in CAMERAS:
        n.subscribe(Image, "/" + nome, fazer_cb(nome))
    fim = time.time() + segundos
    while time.time() < fim and not os.path.exists(os.path.join(pasta, "PARAR")):
        time.sleep(0.2)
    time.sleep(1.0)
    with trava:
        for nome, e in escritores.items():
            e.release()
            json.dump(carimbos[nome], open(os.path.join(pasta, nome + ".json"), "w"))
            c = carimbos[nome]
            fora = sum(1 for a, b in zip(c, c[1:]) if b <= a)
            hz = (len(c) - 1) / (c[-1] - c[0]) if len(c) > 1 else 0
            buracos = sum(1 for a, b in zip(c, c[1:]) if b - a > 1.5 / CAMERAS[nome]["hz"])
            print(f"  {nome}: {len(c)} quadros, {c[0]:.3f} a {c[-1]:.3f} s simulados ({hz:.0f} Hz; pedido {CAMERAS[nome]['hz']}), "
                  f"buracos: {buracos}, fora de ordem: {fora}")
    sys.stdout.flush()
    os._exit(0)          # sem isto o gz transport derruba o Python na saída (GIL), depois de tudo gravado


def roteiro():
    import rclpy
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.entity_pb2 import Entity
    from gz.transport13 import Node as GzNode
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import Empty, Float64MultiArray
    rclpy.init()
    no = Node("roteiro_video", parameter_overrides=[Parameter("use_sim_time", value=True)])
    pub_alvo = no.create_publisher(Float64MultiArray, "/torreta/alvo", 10)
    pub_disp = no.create_publisher(Empty, "/torreta/disparar", 10)
    gz = GzNode()

    def criar(nome):
        m = EntityFactory()
        m.sdf = '<?xml version="1.0"?><sdf version="1.9">' + camera_sdf(nome, CAMERAS[nome]) + "</sdf>"
        gz.request("/world/torreta/create", m, EntityFactory, Boolean, 3000)    # a resposta pode não vir no prazo; a prova é o quadro chegar

    def tirar(nome):
        m = Entity()
        m.name, m.type = nome, Entity.MODEL
        gz.request("/world/torreta/remove", m, Entity, Boolean, 3000)
    while rclpy.ok() and (pub_alvo.get_subscription_count() == 0 or no.get_clock().now().nanoseconds == 0):
        rclpy.spin_once(no, timeout_sec=0.2)
    agora = lambda: no.get_clock().now().nanoseconds * 1e-9  # noqa: E731
    t0 = agora()
    print(f"roteiro: início em t = {t0:.3f} s simulados")
    for t, (acao, arg) in ROTEIRO:
        while rclpy.ok() and agora() < t0 + t:
            rclpy.spin_once(no, timeout_sec=0.01)
        if acao in ("criar", "tirar"):
            for nome in arg:
                threading.Thread(target=criar if acao == "criar" else tirar, args=(nome,), daemon=True).start()
        elif acao == "alvo":
            pub_alvo.publish(Float64MultiArray(data=arg))
        elif acao == "disparar":
            pub_disp.publish(Empty())
        print(f"  t = {agora() - t0:6.3f} s ({agora():.3f}): {acao} {arg if arg is not None else ''}")
    no.destroy_node()
    rclpy.try_shutdown()


# ------------------------------------------------------------------ montagem do filme
def montar(pasta, dardo_jsonl, saida, gif=None):
    """Escreve cada quadro no vídeo assim que ele fica pronto: na RAM ficam só uns poucos quadros.
    (A versão de 2026-11-10 guardava todos em listas, ~3,9 GB, e derrubou o WSL às 21h.)"""
    import cv2
    import numpy as np
    from PIL import Image as PImage, ImageDraw, ImageFont
    fonte = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    F1, F2 = ImageFont.truetype(fonte, 30), ImageFont.truetype(fonte, 20)
    W, H = 1280, 720

    def quadros(nome, ti, tf):
        """Gerador: lê o .mp4 da câmera e devolve (t, quadro) só dentro de [ti, tf], um de cada vez."""
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
            print(f"  {nome} [{ti - t0:.3f}, {tf - t0:.3f}] s: {n} quadros usados")

    D = [json.loads(x) for x in open(dardo_jsonl)]
    tl = None
    for a, b in zip(D, D[1:]):
        if b["t"] > a["t"] and math.dist(a["d"]["p"], b["d"]["p"]) / (b["t"] - a["t"]) > 2.0:
            tl = a["t"]
            break
    if tl is None:
        raise SystemExit("o dardo não foi lançado na gravação")
    print(f"  lançamento em t = {tl:.3f} s simulados")
    t0 = json.load(open(os.path.join(pasta, "cam_geral.json")))[0]
    td = t0 + 7.0

    def legenda(img, titulo, sub, canto=""):
        p = PImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(p, "RGBA")
        d.rectangle([0, H - 92, W, H], fill=(0, 0, 0, 160))
        d.text((24, H - 84), titulo, font=F1, fill=(255, 255, 255))
        d.text((24, H - 42), sub, font=F2, fill=(220, 220, 220))
        if canto:
            d.rectangle([W - 330, 0, W, 40], fill=(0, 0, 0, 140))
            d.text((W - 318, 8), canto, font=F2, fill=(255, 230, 120))
        return cv2.cvtColor(np.array(p), cv2.COLOR_RGB2BGR)

    def ajustar(q):
        return q if q.shape[1] == W else cv2.resize(q, (W, H))

    v = cv2.VideoWriter(saida, cv2.VideoWriter_fourcc(*"avc1"), FPS, (W, H))
    imgs, n_saida = [], 0          # GIF: só 1 de cada 3 quadros, já em 640x360 (~0,7 MB cada)

    def escrever(q):
        nonlocal n_saida
        v.write(q)
        if gif and n_saida % 3 == 0:
            imgs.append(PImage.fromarray(cv2.cvtColor(cv2.resize(q, (640, 360)), cv2.COLOR_BGR2RGB)))
        n_saida += 1

    # 0. título
    base = ajustar(next(quadros("cam_geral", t0, t0))[1])
    titulo = legenda(base, "Torreta de rastreio no Gazebo", "comandada pelo nó comando_torreta (Daniel) · CAD → URDF → ros2_control")
    for _ in range(int(2.5 * FPS)):
        escrever(titulo)
    # 1. mira, tempo real, com o close do pinhão e da coroa no canto (quadro do pan mais perto no tempo)
    pan = quadros("cam_pan", t0, t0 + 6.3)
    pa, pb = next(pan), next(pan, None)
    for t, q in quadros("cam_geral", t0 + 0.8, t0 + 6.3):
        while pb is not None and abs(pb[0] - t) <= abs(pa[0] - t):
            pa, pb = pb, next(pan, None)
        mini = cv2.resize(pa[1], (480, 270))
        cv2.rectangle(mini, (0, 0), (479, 269), (255, 255, 255), 2)
        q[20:290, W - 500:W - 20] = mini
        escrever(legenda(q, "Mira: pan e tilt (tempo real)", "alvos 45°/20°, −30°/−10°, 0°/0° · no canto: pinhão 20T no servo → coroa 40T (−2:1)",
                         f"t = {t - t0:5.2f} s"))
    pan.close()
    # 2. lançador em câmera lenta: volantes começando a girar, depois empurrão e lançamento
    for t, q in quadros("cam_lancador", td, td + 0.16):
        escrever(legenda(ajustar(q), "Disparo: os volantes aceleram (33× mais lento)",
                         "marca branca em cada volante, que giram em sentidos opostos · gaiola e funil escondidos só no vídeo",
                         f"t = {t - t0:6.3f} s"))
    for t, q in quadros("cam_lancador", tl - 0.17, tl + 0.03):
        escrever(legenda(ajustar(q), "SG90 → pinhão → cremalheira empurra o dardo (33× mais lento)",
                         "a 95% da rotação o nó manda empurrar; a ponta entra entre os volantes e o dardo sai a 21,6 m/s",
                         f"t = {t - t0:6.3f} s"))
    # 3. voo
    for t, q in quadros("cam_voo", tl - 0.02, tl + 0.30):
        escrever(legenda(ajustar(q), "Voo do dardo (8× mais lento)", "sem arrasto do ar: alcance é limite de cima; o dardo some ao tocar o chão",
                         f"t = {t - t0:6.3f} s"))
    # 4. recolhe e desliga, tempo real
    for t, q in quadros("cam_lanc30", tl + 0.2, t0 + 9.5):
        escrever(legenda(ajustar(q), "Recolhe a cremalheira e desliga os volantes (tempo real)", "sequência PARADO → ACELERANDO → EMPURRANDO → RECOLHENDO → PARADO",
                         f"t = {t - t0:6.3f} s"))
    v.release()
    print(f"  {saida}: {n_saida} quadros = {n_saida / FPS:.1f} s")
    if gif:
        imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=1000 * 3 // FPS, loop=0, optimize=True)
        print(f"  {gif}: {len(imgs)} quadros, {os.path.getsize(gif) / 1e6:.1f} MB")


if __name__ == "__main__":
    a = sys.argv
    if a[1] == "mundo":
        mundo(a[2], a[3])
    elif a[1] == "gravar":
        gravar(a[2], float(a[3]))
    elif a[1] == "roteiro":
        roteiro()
    elif a[1] == "montar":
        montar(a[2], a[3], a[4], a[5] if len(a) > 5 else None)
