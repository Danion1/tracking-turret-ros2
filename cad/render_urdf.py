#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
render_urdf.py — a torreta montada PELO ARQUIVO torreta.urdf (não pelo CAD), em PNG e GIF.

Lê o URDF gravado do zero (origens, eixos, tipos, mimic, malhas e cores), sem usar a tabela
do gerar_urdf.py: é uma 2ª leitura independente, como o RViz2 faria. Por isso confere também:
  - que as mimic do ARQUIVO dão a razão e o sinal esperados (coroa −0,5; volante −1; 0,010 m/rad);
  - que as malhas abrem e ficam onde o CAD diz (ponto primitivo do pan a 30 mm do eixo da coroa).

Nos volantes, uma marca vermelha (só no render, não no URDF nem no CAD) deixa o giro visível.

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/render_urdf.py [--gif]
Saída: previews/urdf_*.png e previews/urdf_mecanismos.gif
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import vtk

AQUI = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(AQUI)
PKG = os.path.join(PROJ, "ros2", "torreta_description")
PREV = os.path.join(PROJ, "previews")
ok = True


def conferir(nome, cond, detalhe=""):
    global ok
    print(f"  [{'OK' if cond else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok &= bool(cond)


# ------------------------------------------------------------------ leitura do URDF
def ler():
    raiz = ET.parse(os.path.join(PKG, "urdf", "torreta.urdf")).getroot()
    cores = {m.get("name"): [float(v) for v in m.find("color").get("rgba").split()[:3]] for m in raiz.findall("material")}
    vis = {}
    for l in raiz.findall("link"):
        vis[l.get("name")] = []
        for v in l.findall("visual"):
            me = v.find("geometry/mesh")
            arq = me.get("filename").replace("package://torreta_description", PKG)
            vis[l.get("name")].append((arq, float(me.get("scale").split()[0]), cores[v.find("material").get("name")]))
    juntas = []
    for j in raiz.findall("joint"):
        mm = j.find("mimic")
        ax = j.find("axis")
        juntas.append({"nome": j.get("name"), "tipo": j.get("type"), "pai": j.find("parent").get("link"),
                       "filho": j.find("child").get("link"),
                       "xyz": [float(v) for v in j.find("origin").get("xyz").split()],
                       "eixo": [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1],
                       "mimic": (mm.get("joint"), float(mm.get("multiplier")), float(mm.get("offset", 0))) if mm is not None else None})
    return vis, juntas


def rot(e, a):
    ux, uy, uz = e
    c, s, v = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + ux * ux * v, ux * uy * v - uz * s, ux * uz * v + uy * s, 0],
                     [uy * ux * v + uz * s, c + uy * uy * v, uy * uz * v - ux * s, 0],
                     [uz * ux * v - uy * s, uz * uy * v + ux * s, c + uz * uz * v, 0], [0, 0, 0, 1.0]])


def fk(juntas, q):
    v = {}
    for j in juntas:
        if j["mimic"]:
            lider, mult, off = j["mimic"]
            v[j["nome"]] = mult * q.get(lider, 0.0) + off
        else:
            v[j["nome"]] = q.get(j["nome"], 0.0)
    T = {"world": np.eye(4)}
    pend = list(juntas)
    while pend:
        for j in pend:
            if j["pai"] in T:
                o = np.eye(4); o[:3, 3] = j["xyz"]
                if j["tipo"] == "prismatic":
                    m = np.eye(4); m[:3, 3] = np.array(j["eixo"]) * v[j["nome"]]
                elif j["tipo"] in ("revolute", "continuous"):
                    m = rot(j["eixo"], v[j["nome"]])
                else:
                    m = np.eye(4)
                T[j["filho"]] = T[j["pai"]] @ o @ m
                pend.remove(j)
                break
    return T, v


# ------------------------------------------------------------------ cena
class Cena:
    def __init__(self, vis, larg=1100, alt=820):
        self.vis = vis
        self.jan = vtk.vtkRenderWindow(); self.jan.SetOffScreenRendering(1); self.jan.SetSize(larg, alt)
        self.r = vtk.vtkRenderer(); self.r.SetBackground(1, 1, 1); self.jan.AddRenderer(self.r)
        self.atores = {}
        for link, lista in vis.items():
            for arq, esc, cor in lista:
                rd = vtk.vtkSTLReader(); rd.SetFileName(arq); rd.Update()
                mp = vtk.vtkPolyDataMapper(); mp.SetInputConnection(rd.GetOutputPort())
                a = vtk.vtkActor(); a.SetMapper(mp); a.GetProperty().SetColor(*cor)
                a.GetProperty().SetAmbient(0.25); a.GetProperty().SetSpecular(0.15)
                self.r.AddActor(a)
                self.atores.setdefault(link, []).append((a, esc, tuple(round(v, 2) for v in cor)))
        for link, sy in (("rotor_mais_y", 1), ("rotor_menos_y", -1)):        # marca de referência do giro
            cb = vtk.vtkCubeSource(); cb.SetBounds(0, 17.5, -1.2, 1.2, 5.0, 5.8)
            mp = vtk.vtkPolyDataMapper(); mp.SetInputConnection(cb.GetOutputPort())
            a = vtk.vtkActor(); a.SetMapper(mp); a.GetProperty().SetColor(0.85, 0.05, 0.05)
            self.r.AddActor(a)
            self.atores[link].append((a, 0.001, "marca"))
        self.texto = vtk.vtkTextActor(); self.texto.GetTextProperty().SetFontSize(20); self.texto.GetTextProperty().SetColor(0.1, 0.1, 0.1)
        self.texto.SetPosition(16, 16); self.r.AddViewProp(self.texto)

    def pose(self, T, so=None, transl=None):
        for link, lista in self.atores.items():
            for a, esc, cor in lista:
                m = vtk.vtkMatrix4x4()
                for i in range(4):
                    for k in range(4):
                        m.SetElement(i, k, T[link][i][k])
                tf = vtk.vtkTransform(); tf.SetMatrix(m); tf.Scale(esc, esc, esc)
                a.SetUserTransform(tf)
                a.SetVisibility(so is None or link in so or (link, cor) in so)    # so: links ou (link, cor)
                a.GetProperty().SetOpacity(0.18 if transl and link in transl else 1.0)

    def camera(self, foco, pos, zoom=1.0):
        c = self.r.GetActiveCamera(); c.SetViewUp(0, 0, 1); c.SetFocalPoint(*foco); c.SetPosition(*pos)
        c.SetViewAngle(30); self.r.ResetCameraClippingRange()
        c.Zoom(zoom); self.r.ResetCameraClippingRange()

    def png(self, caminho, legenda=""):
        self.texto.SetInput(legenda)
        self.jan.Render()
        f = vtk.vtkWindowToImageFilter(); f.SetInput(self.jan); f.ReadFrontBufferOff(); f.Update()
        w = vtk.vtkPNGWriter(); w.SetFileName(caminho); w.SetInputConnection(f.GetOutputPort()); w.Write()


def main():
    vis, juntas = ler()
    print("1. URDF lido do arquivo")
    print(f"  {len(vis)} links, {sum(len(v) for v in vis.values())} malhas, {len(juntas)} juntas")
    _, v = fk(juntas, {"pan_motor": 1.0, "volante_motor": 1.0, "sg90": 1.0})
    conferir("mimic do arquivo: coroa = −0,5 × motor do pan", abs(v["pan"] + 0.5) < 1e-12, f"{v['pan']:+.3f}")
    conferir("mimic do arquivo: volante −y = −1 × volante +y", abs(v["volante_espelho"] + 1.0) < 1e-12, f"{v['volante_espelho']:+.3f}")
    conferir("mimic do arquivo: cremalheira = 0,010 m por rad do SG90", abs(v["cremalheira"] - 0.010) < 1e-12, f"{v['cremalheira']:.4f} m")
    for L, lista in vis.items():
        for arq, _, _ in lista:
            conferir(f"malha existe: {os.path.basename(arq)}", os.path.isfile(arq)) if not os.path.isfile(arq) else None

    print("\n2. Renders")
    c = Cena(vis)
    def T_de(q):
        return fk(juntas, q)[0]
    BERCO = {"berco"}
    PAN = {"pinhao_pan", ("pan_link", (0.2, 0.45, 0.8)), ("base_link", (0.2, 0.2, 0.22))}    # pinhão, coroa, servo do pan
    lanc = ["rotor_mais_y", "rotor_menos_y", "pinhao_sg90", "empurrador", "berco"]
    cenas = [
        ("urdf_torreta.png", {"pan_motor": -2 * math.radians(30), "tilt": math.radians(15)}, None, None,
         (0.02, 0, 0.11), (0.42, -0.42, 0.33), 1.0, "torreta inteira: pan 30°, tilt 15°"),
        ("urdf_pan_transmissao.png", {"pan_motor": -2 * math.radians(40)}, PAN, None,
         (0.006, 0.022, 0.012), (0.07, -0.07, 0.13), 1.0, "pan: pinhão 20T (laranja) e coroa 40T, pan 40° → motor −80°"),
        ("urdf_lancador.png", {"sg90": 1.32, "volante_motor": 0.6}, lanc, BERCO,
         (0.01, 0, 0.118), (0.05, -0.05, 0.40), 1.0, "lançador visto de cima: empurrador no fim do curso (13,2 mm), volantes"),
        ("urdf_tilt_40.png", {"tilt": math.radians(40), "sg90": 1.32}, None, None,
         (0.02, 0, 0.11), (0.05, -0.55, 0.13), 1.0, "tilt 40° (nariz para cima), disparo"),
    ]
    for arq, q, so, transl, foco, pos, zoom, leg in cenas:
        c.pose(T_de(q), set(so) if so else None, transl)
        c.camera(foco, pos, zoom)
        c.png(os.path.join(PREV, arq), leg)
        print(f"  {arq}")

    if "--gif" in sys.argv:
        from PIL import Image
        print("\n3. GIF dos mecanismos")
        quadros = []
        tmp = os.path.join(PREV, "_quadro.png")
        n = 36
        seq = []
        for i in range(n):                                     # pan: motor −180°..+180° (torre +90°..−90°)
            a = math.sin(2 * math.pi * i / n) * math.pi
            seq.append(("pan", {"pan_motor": a}, f"pan_motor {math.degrees(a):+5.0f}°  →  coroa (pan) {-math.degrees(a) / 2:+5.0f}°"))
        for i in range(24):                                    # tilt
            t = math.radians(10 + 30 * math.sin(2 * math.pi * i / 24))
            seq.append(("geral", {"tilt": t}, f"tilt {math.degrees(t):+4.0f}° (direto no servo)"))
        for i in range(36):                                    # volantes girando + disparo
            sg = 1.32 * max(0.0, math.sin(math.pi * (i - 10) / 16)) if 10 <= i < 26 else 0.0
            seq.append(("lanc", {"volante_motor": i * 0.35, "sg90": sg},
                        f"volante +y {math.degrees(i * 0.35) % 360:5.0f}°, volante −y espelhado  |  SG90 {math.degrees(sg):4.0f}° → cremalheira {sg * 10:4.1f} mm"))
        for k, (vista, q, leg) in enumerate(seq):
            if vista == "pan":
                c.pose(T_de(q), PAN)
                c.camera((0.006, 0.022, 0.012), (0.07, -0.07, 0.13))
            elif vista == "lanc":
                c.pose(T_de(q), set(lanc), BERCO)
                c.camera((0.01, 0, 0.118), (0.05, -0.05, 0.40))
            else:
                c.pose(T_de(q))
                c.camera((0.02, 0, 0.11), (0.05, -0.55, 0.13))
            c.png(tmp, leg)
            quadros.append(Image.open(tmp).convert("RGB").resize((660, 492), Image.LANCZOS).quantize(colors=128))
        os.remove(tmp)
        saida = os.path.join(PREV, "urdf_mecanismos.gif")
        quadros[0].save(saida, save_all=True, append_images=quadros[1:], duration=90, loop=0, optimize=True)
        print(f"  {saida} ({len(quadros)} quadros, {os.path.getsize(saida) / 1e6:.1f} MB)")
    if not ok:
        raise SystemExit("conferência falhou")


if __name__ == "__main__":
    main()
