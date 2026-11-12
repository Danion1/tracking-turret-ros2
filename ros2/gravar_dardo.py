#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gravar_dardo.py — CONFERÊNCIA (não comanda nada): grava a pose do dardo e a do berço (Gazebo, 1 kHz)
e mede o tiro.

  gravar:   python3 gravar_dardo.py gravar <arquivo.jsonl> <segundos>
  analisar: python3 gravar_dardo.py analisar <arquivo.jsonl> <config/dardo.json>

Mede: onde a ponta parou no empurrão (frame do berço) contra o ponto de toque nos volantes; se o
dardo saiu da calha (|y| e z no berço) durante o empurrão; a velocidade de saída e a direção contra o
cano; o voo (tempo, distância e onde caiu).
"""
import json
import sys
import time

import numpy as np


def matriz(p):
    w, x, y, z = p["q"]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def gravar(arq, dur):
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
    f = open(arq, "w")
    berco = {}

    def pq(p):
        return {"p": [p.position.x, p.position.y, p.position.z], "q": [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]}

    def ao_berco(m):
        for p in m.pose:
            if p.name == "berco" or p.name.endswith("::berco"):
                berco["b"] = pq(p)

    def ao_dardo(m):
        if "b" in berco:
            f.write(json.dumps({"t": m.header.stamp.sec + m.header.stamp.nsec * 1e-9, "d": pq(m), "b": berco["b"]}) + "\n")

    n = Node()
    n.subscribe(Pose_V, "/model/torreta/pose", ao_berco)
    n.subscribe(Pose, "/model/dardo/pose", ao_dardo)
    time.sleep(dur)
    f.close()


def analisar(arq, cfg_arq):
    C = json.load(open(cfg_arq, encoding="utf-8"))
    L = [json.loads(x) for x in open(arq)]
    L = [a for i, a in enumerate(L) if i == 0 or a["t"] > L[i - 1]["t"]]
    t = np.array([a["t"] for a in L])
    pw = np.array([a["d"]["p"] for a in L])
    loc = np.array([matriz(a["b"]).T @ (np.array(a["d"]["p"]) - np.array(a["b"]["p"])) for a in L])   # centro do dardo no berço
    ponta = loc[:, 0] + C["comprimento_m"] / 2
    v = np.gradient(pw, t, axis=0)
    rap = np.linalg.norm(v, axis=1)
    print(f"  {len(L)} amostras em {t[-1] - t[0]:.2f} s de simulação ({len(L) / (t[-1] - t[0]):.0f} Hz)")
    i_l = int(np.argmax(rap > 2.0)) if (rap > 2.0).any() else None
    fim_emp = i_l if i_l is not None else len(L)
    print(f"  ponta no início: x = {ponta[0] * 1000:.2f} mm (berço; CAD {C['x_ponta_inicial_m'] * 1000:.2f})")
    print(f"  ponta máx. no empurrão: x = {ponta[:fim_emp].max() * 1000:.2f} mm; toque nos volantes em {C['x_toque_m'] * 1000:.2f} "
          f"({(ponta[:fim_emp].max() - C['x_toque_m']) * 1000:+.2f} mm)")
    print(f"  dentro da calha no empurrão: |y| máx {np.abs(loc[:fim_emp, 1]).max() * 1000:.3f} mm, "
          f"z de {loc[:fim_emp, 2].min() * 1000:.3f} a {loc[:fim_emp, 2].max() * 1000:.3f} mm (centro; piso a −6,5, teto a +6,5, raio 6,4)")
    if i_l is None:
        print("  NÃO LANÇOU (o dardo não passou de 2 m/s)")
        return False
    j = i_l + int(np.argmax(rap[i_l:i_l + 50]))
    eixo = matriz(L[j]["b"])[:, 0]
    vs = v[j]
    ang = np.degrees(np.arccos(np.clip(vs @ eixo / np.linalg.norm(vs), -1, 1)))
    print(f"  LANÇOU em t = {t[i_l]:.3f} s; ponta em x = {ponta[i_l] * 1000:.2f} mm")
    print(f"  velocidade de saída {np.linalg.norm(vs):.2f} m/s, {ang:.2f}° do eixo do cano")
    chao = np.argmax((pw[j:, 2] < 0.008) & (t[j:] > t[j] + 0.01))
    if chao:
        k = j + chao
        d = np.linalg.norm(pw[k, :2] - pw[j, :2])
        print(f"  voo: {t[k] - t[j]:.3f} s, caiu a {d:.2f} m da saída (chão em ({pw[k, 0]:.2f}, {pw[k, 1]:.2f}) m)")
        h = pw[j, 2]
        print(f"  conta à mão (sem arrasto): saída a {h:.3f} m de altura, v {vs[0]:.2f}/{vs[2]:+.2f} m/s → "
              f"t = {(vs[2] + np.sqrt(vs[2] ** 2 + 2 * 9.80665 * h)) / 9.80665:.3f} s")
    else:
        print("  o dardo ainda não tinha caído no fim da gravação")
    return True


if __name__ == "__main__":
    if sys.argv[1] == "gravar":
        gravar(sys.argv[2], float(sys.argv[3]))
    else:
        sys.exit(0 if analisar(sys.argv[2], sys.argv[3]) else 1)
