#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
berco.py — peça 4 da parte B: o berço (piso + 2 faces), que inclina no tilt.

Frame do berço: origem no eixo do tilt, x para o cano, z para cima; no mundo,
com tilt t, gira −t em torno de y e sobe para (0, 0, z_tilt). t > 0 levanta o nariz.

- Piso (199 × 92 × 3): imprime com a face de baixo na cama. Leva a boca
  (guias de saída, como no plano), 4 ressaltos que prendem as faces (insertos
  M3 em y, só nas pontas em x), o entalhe que dá lugar às cabeças dos M3 do
  horn, os furos dos motores (ressalto Ø6,15), a janela e os rasgos do SG90
  (os furos das orelhas não são publicados), os furos da gaiola e o da USB.
- Face +y: o horn do servo do tilt aparafusa nela (4 × M3 de dentro, rosca do
  horn ASSUMIDA M3), com a bolsa para a cabeça do parafuso central.
- Face −y: ressalto Ø20 com a porca travante M8 presa num sextavado e a ponta
  Ø10,4 que encosta SÓ no anel interno do 608 (SKF: da 10–10,5). Do outro lado,
  arruela impressa Ø10,4 e a cabeça do M8×20 por fora do braço. Furo dos cabos.
- Fios: o feixe que a peça 3 trouxe até o vão do braço −y entra pela face −y,
  perto do eixo (o cabo quase não estica no tilt), corre sob o piso até os
  motores e o SG90; a USB sobe pelo furo do piso (a peça 5 leva até a webcam).

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/berco.py [--rapido]
"""

import json
import math
import os
import sys

import cadquery as cq

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import usar_biblioteca  # noqa: E402,F401
import caixa as cx  # noqa: E402
import mesa_bracos as mb  # noqa: E402
import parafusos_iso as pi  # noqa: E402
import tampo as tp  # noqa: E402
from check_bbox import check_bbox, check_volume, exigir_tudo_ok  # noqa: E402
from dados_materiais import PLA  # noqa: E402
from exportar_stl_mm import exportar_stl_mm  # noqa: E402
from formas_basicas import box_minmax, cylinder_y_at, cylinder_z_at  # noqa: E402
from massa_inercia import propriedades_solido  # noqa: E402
from render_cad import render  # noqa: E402

PROJ = os.path.dirname(AQUI)
P = json.load(open(os.path.join(PROJ, "plano", "parametros.json"), encoding="utf-8"))
BE, BC, TC = P["berco"], P["berco_cad"], P["tilt_cad"]
K, INS, FI = P["pecas"], P["pecas"]["inserto_m3"], P["caixa"]["fios"]
ZT = TC["z"]
RAPIDO = "--rapido" in sys.argv
ok = True


def conferir(nome, cond, detalhe=""):
    global ok
    print(f"  [{'OK' if cond else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok &= bool(cond)
    return cond


def caixa(x0, x1, y0, y1, z0, z1):
    return box_minmax(x0, x1, y0, y1, z0, z1).val()


def cil_y(x, z, d, y0, y1):
    return cylinder_y_at(x, (y0 + y1) / 2, z, d, y1 - y0).val()


def mundo(s, tilt=0.0):
    """Berço → mundo: gira −tilt em torno de y (t > 0 levanta o nariz) e sobe ao eixo."""
    return s.rotate(cq.Vector(), cq.Vector(0, 1, 0), -tilt).translate(cq.Vector(0, 0, ZT))


def sextavado_y(s_af, y0, y1, x=0.0, z=0.0):
    """Prisma sextavado (medida entre faces s_af) ao longo de y."""
    return (cq.Workplane("XZ").workplane(offset=-y1).center(x, z).polygon(6, s_af / math.cos(math.pi / 6))
            .extrude(y1 - y0).val())


# ---------------------------------------------------------------- peças
def fazer_piso():
    pi_ = BE["piso"]
    s = (cq.Workplane("XY").workplane(offset=pi_["z0"]).center((pi_["x0"] + pi_["x1"]) / 2, 0)
         .rect(pi_["x1"] - pi_["x0"], 2 * pi_["y"]).extrude(pi_["z1"] - pi_["z0"]).edges("|Z").fillet(3.0).val())
    ca, bo, d = BE["calha"], BE["boca"], K["dardo"]["d"]
    adic = []
    for sy in (1, -1):          # boca: guias de saída depois dos volantes (parte do piso, como no plano)
        y0, y1 = (ca["y0"], ca["y1"]) if sy > 0 else (-ca["y1"], -ca["y0"])
        adic.append(caixa(bo["x0"], bo["x1"], y0, y1, pi_["z1"] - 0.3, pi_["z1"] + d + 2))
    R = BC["ressaltos"]
    for xa, xb in R["x"]:       # ressaltos das faces (só nas pontas em x: o meio é das cabeças do horn e da porca M8)
        for sy in (1, -1):
            y0, y1 = (pi_["y"] - R["esp_y"], pi_["y"]) if sy > 0 else (-pi_["y"], -pi_["y"] + R["esp_y"])
            adic.append(caixa(xa, xb, y0, y1, pi_["z1"] - 0.3, R["z1"]))
    for a in adic:
        s = s.fuse(a)
    ferr = []
    prof = INS["L"] + INS["fundo_extra"]
    for xa, xb in R["x"]:
        for sy in (1, -1):
            yf = sy * pi_["y"]
            ferr.append(cil_y((xa + xb) / 2, 0.0, INS["furo_d"], min(yf, yf - sy * prof), max(yf, yf - sy * prof)))
    e = BC["entalhe_horn"]
    ferr.append(caixa(e["x"][0], e["x"][1], pi_["y"] - e["y_prof"], pi_["y"] + 1, pi_["z0"] - 1, pi_["z1"] + 1))
    fu = BC["piso_furos"]
    for nome in ("motores", "gaiola"):
        for x, y in fu[nome]["pontos"]:
            ferr.append(cylinder_z_at(x, y, pi_["z0"] - 1, fu[nome]["d"], pi_["z1"] - pi_["z0"] + 2).val())
    u = fu["usb_webcam"]
    ferr.append(caixa(u["x"] - u["w"] / 2, u["x"] + u["w"] / 2, u["y"] - u["d"] / 2, u["y"] + u["d"] / 2, pi_["z0"] - 1, pi_["z1"] + 1))
    sg, cs = BC["sg90"], K["sg90"]["caixa"]
    f = sg["janela_folga"]
    ferr.append(caixa(sg["x"] - cs[0] / 2 - f, sg["x"] + cs[0] / 2 + f, sg["y"] - cs[1] / 2 - f, sg["y"] + cs[1] / 2 + f,
                      pi_["z0"] - 1, pi_["z1"] + 1))
    for sx in (-1, 1):          # rasgos das orelhas do SG90: entre-furos de 26 a 29 (medir o comprado)
        a0, a1 = sg["x"] + sx * sg["rasgo_entre"][0] / 2, sg["x"] + sx * sg["rasgo_entre"][1] / 2
        rr = sg["rasgo_d"] / 2
        ferr.append(cq.Workplane("XY").workplane(offset=pi_["z0"] - 1).center((a0 + a1) / 2, sg["y"])
                    .slot2D(abs(a1 - a0) + 2 * rr, 2 * rr).extrude(pi_["z1"] - pi_["z0"] + 2).val())
    for t in ferr:
        s = s.cut(t)
    return s


def contorno_face(y0, y1):
    F = BE["face"]
    return (cq.Workplane("XZ").workplane(offset=-y1).center((F["x0"] + F["x1"]) / 2, (F["z0"] + F["z1"]) / 2)
            .rect(F["x1"] - F["x0"], F["z1"] - F["z0"]).extrude(y1 - y0).edges("|Y").fillet(6.0).val())


def furos_face(s, y0, y1):
    for xa, xb in BC["ressaltos"]["x"]:
        s = s.cut(cil_y((xa + xb) / 2, 0.0, pi.furo_passagem("M3"), y0 - 1, y1 + 1))
    return s


def fazer_face_mais():
    F = BE["face"]
    y0, y1 = F["y0"], F["y1"]
    s = contorno_face(y0, y1)
    fm = BC["face_mais"]
    r = K["sts3215"]["horn_furos_circulo"] / 2
    for k in range(4):          # 4 furos do horn, a 45° (±4,95; ±4,95)
        a = math.radians(45 + 90 * k)
        s = s.cut(cil_y(r * math.cos(a), r * math.sin(a), pi.furo_passagem(fm["horn_rosca"]), y0 - 1, y1 + 1))
    s = s.cut(cil_y(0, 0, fm["bolsa_d"], y1 - fm["bolsa_prof"], y1 + 1))
    return furos_face(s, y0, y1)


def fazer_face_menos():
    F, fm = BE["face"], BC["face_menos"]
    y0, y1 = -F["y1"], -F["y0"]
    s = contorno_face(y0, y1)
    y_ponta = TC["menos"]["y"][1]                                 # face de dentro do braço −y = face do anel interno
    y_ress = y_ponta + fm["ponta_l"]
    s = s.fuse(cil_y(0, 0, fm["ressalto_d"], y_ress, y0 + 0.3))
    s = s.fuse(cil_y(0, 0, fm["ponta_d"], y_ponta, y_ress + 0.3))
    s = s.cut(cil_y(0, 0, fm["furo_m8"], y_ponta - 1, y1 + 1))
    s = s.cut(sextavado_y(fm["porca_s"] + fm["porca_folga"], y1 - fm["porca_m"], y1 + 1))
    fc = fm["furo_cabos"]
    s = s.cut(caixa(fc["x"] - fc["w"] / 2, fc["x"] + fc["w"] / 2, y0 - 1, y1 + 1, fc["z"] - fc["h"] / 2, fc["z"] + fc["h"] / 2))
    return furos_face(s, y0, y1), {"y_ponta": y_ponta, "y_ress": y_ress}


# ---------------------------------------------------------------- referências (compradas)
def motor_130(x, y):
    """Lata em D (Ø20,1, faces a 15,1) pendurada sob o piso, eixo para cima; bucha traseira."""
    m = K["motor130"]
    z1, z0 = BE["motor"]["z1"], BE["motor"]["z0"]
    lata = cylinder_z_at(x, y, z0, m["larg"], z1 - z0).val().intersect(caixa(x - 15, x + 15, y - m["plano"] / 2, y + m["plano"] / 2, z0 - 1, z1 + 1))
    bucha = cylinder_z_at(x, y, z0 - 3.4, 6.0, 3.4 + 0.1).val()
    eixo = cylinder_z_at(x, y, z1, m["eixo_d"], 9.4).val()
    return lata.fuse(bucha).fuse(eixo)


def sg90():
    sg, cs = BC["sg90"], K["sg90"]["caixa"]
    z1 = BE["piso"]["z1"]                                       # topo da caixa rente ao topo do piso (orelhas 3 abaixo)
    corpo = caixa(sg["x"] - cs[0] / 2, sg["x"] + cs[0] / 2, sg["y"] - cs[1] / 2, sg["y"] + cs[1] / 2, z1 - cs[2], z1)
    orelhas = caixa(sg["x"] - 16, sg["x"] + 16, sg["y"] - 6, sg["y"] + 6, BE["piso"]["z0"] - 2.5, BE["piso"]["z0"])
    return corpo.fuse(orelhas)


def pilha_m8(info):
    """Cabeça, arruela, corpo e porca do M8 (giram com o berço), no frame do berço."""
    fm = BC["face_menos"]
    yb = TC["menos"]["y"][0]                                    # face de fora do braço −y
    # arruela impressa: encosta no anel interno (que fica 1,0 dentro do braço, no fundo do encosto) e sai 0,5
    # para fora; a cabeça fica 0,5 longe da face do braço
    ya1 = yb + 1.0
    ya0 = ya1 - fm["arruela_e"]
    arr = cil_y(0, 0, fm["arruela_d"], ya0, ya1)
    cab = cil_y(0, 0, 13.0, ya0 - 8.0, ya0)
    yf = -BE["face"]["y0"]
    corpo = cil_y(0, 0, 8.0, ya0, yf)
    porca = sextavado_y(fm["porca_s"], yf - fm["porca_m"], yf)
    return [("arruela", arr), ("cabeça M8", cab), ("corpo M8", corpo), ("porca", porca)], yf - ya0


# ---------------------------------------------------------------- fios (frame do berço, tilt 0)
def fios():
    zi = -18.0                                                  # a peça 3 entregou em z 100 do mundo = −18 do berço
    yv = mb.M["furos_cabos"]["menos"]["y"]                      # y −53,5 (vão do braço −y)
    m = K["motor130"]
    mx, my = BE["motor"]["x"], BE["motor"]["y"]
    zt_ = BE["motor"]["z0"] - 4.5                               # abaixo dos terminais (traseira da lata)
    yfora = -my - m["larg"] / 2 - 2.0
    u = BC["piso_furos"]["usb_webcam"]
    sg = BC["sg90"]
    xsg = sg["x"] - K["sg90"]["caixa"][0] / 2 - 1.5             # ASSUMIDO: o cabo do SG90 sai da ponta −x, perto do fundo
    L = []
    feixe = [("usb", "USB da webcam", "cinza", FI["usb_d"], -14.0, -13.0),
             ("m+", "motores + (3 V)", "amarelo", FI["potencia_d"], -10.0, -13.0),
             ("m-", "motores − (dreno)", "preto", FI["potencia_d"], -7.4, -13.0),
             ("sg1", "SG90 V+", "laranja", FI["sinal_d"], -4.5, -17.5),
             ("sg2", "SG90 GND", "preto", FI["sinal_d"], -2.9, -17.5),
             ("sg3", "SG90 sinal", "branco", FI["sinal_d"], -1.3, -17.5)]
    for k, (cod, nome, cor, d, x0, zn) in enumerate(feixe):
        base = [(x0, yv, zi), (x0, yv, zn), (x0, -40.0 + 1.6 * k, zn)]
        if cod == "usb":
            q = base + [(u["x"], -40.0, zn), (u["x"], u["y"], zn), (u["x"], u["y"], BE["piso"]["z1"] + 6.0)]
        elif cod in ("m+", "m-"):
            xt = mx + (4.25 if cod == "m+" else -4.25)
            q = base + [(xt, -40.0 + 1.6 * k, zn), (xt, yfora, zn), (xt, yfora, zt_), (xt, -my, zt_)]
        else:
            yy = sg["y"] + (k - 4) * 1.6
            q = base + [(xsg - (k - 3) * 1.6, -40.0 + 1.6 * k, zn), (xsg - (k - 3) * 1.6, yy, zn),
                        (xsg - (k - 3) * 1.6, yy, -24.0), (xsg + 1.5 - 0.8, yy, -24.0)]
        L.append((f"{nome}: entra pela face −y e vai ao destino", cor, d, q))
    for xt, cor, n in ((mx + 4.25, "amarelo", "+"), (mx - 4.25, "preto", "−")):   # ponte entre os 2 motores (em paralelo)
        L.append((f"ponte dos motores ({n})", cor, FI["potencia_d"], [(xt, -my, zt_), (xt, my, zt_)]))
    return L


def main():
    print("1. Peças")
    piso = fazer_piso()
    fmais = fazer_face_mais()
    fmenos, inf = fazer_face_menos()
    for nome, s in (("piso", piso), ("face +y", fmais), ("face −y", fmenos)):
        n = len(s.Solids())
        conferir(f"{nome}: 1 sólido válido, volume > 0", n == 1 and s.isValid() and s.Volume() > 0, f"{n} sólido(s), {s.Volume():.0f} mm³")

    print("\n2. Caixa e volume contra contas à mão")
    ok_ = check_bbox("piso", piso, (-95, 104, -46, 46, -9.5, 8.5), tol=0.06)
    ok_ &= check_bbox("face +y", fmais, (-18, 18, 46, 50, -27.5, 16), tol=0.06)
    ok_ &= check_bbox("face −y", fmenos, (-18, 18, -57, -46, -27.5, 16), tol=0.06)
    # piso: 199 × 92 × 3 − cantos (4−π)·3²·3 = 54 924 − 23 = 54 901; boca 2 × 14 × 2 × 15 = 840;
    #  ressaltos 4 × 8 × 8 × 12,5 = 3 200; − insertos 4 × π·2²·6,7 = 337; − entalhe 18 × 4 × 3 = 216;
    #  − motores 2 × π·3,35²·3 = 212; − gaiola 4 × 27 = 109; − USB 18×10×3 = 540;
    #  − SG90 23,4 × 12,6 × 3 = 885 − rasgos 2 × (3 × 2,4 + π·1,2²) × 3 = 70  ->  ≈ 56 572
    ok_ &= check_volume("piso", piso, 56572, tol=0.03 * 56572)
    # face: 36 × 43,5 × 4 − cantos (4−π)·6²·4 = 6 264 − 124 = 6 140
    #  +y: − 6 furos Ø3,4 × 4 = 218 − bolsa π·3,5²·2 = 77  ->  5 845
    #  −y: + ressalto π·10²·6,5 = 2 042 + ponta π·5,2²·0,5 = 42 − furo M8 π·4,2²·11 = 610 (já fora o que cai no sextavado)
    #      − sextavado 0,866·13,3²·8 = 1 226 + furo M8 nele (π·4,2²·8 = 443) − cabos 18 × 10 × 4 = 720 − 2 M3 = 73  ->  ≈ 6 038
    ok_ &= check_volume("face +y", fmais, 5845, tol=0.03 * 5845)
    ok_ &= check_volume("face −y", fmenos, 6038, tol=0.05 * 6038)
    conferir("caixa e volume", ok_)

    print("\n3. Montagem no tilt 0: faces, horn, eixo M8, 608")
    servo = mb.servo_para_mundo(cq.importers.importStep(tp.STEP).val())
    bm, _ = mb.fazer_braco_mais()
    bn, _ = mb.fazer_braco_menos()
    mesa = mb.fazer_mesa()
    W = lambda s: mundo(s, 0.0)                                  # noqa: E731
    for nome, s in (("face +y", fmais), ("face −y", fmenos)):
        conferir(f"{nome} × piso: zero e encostada nos ressaltos", W(s).intersect(W(piso)).Volume() < 1e-3 and s.distance(piso) < 0.01)
    v = W(fmais).intersect(servo).Volume()
    conferir("face +y × servo (horn e cabeça do parafuso central na bolsa): zero", v < 1e-3, f"{v:.4f} mm³")
    conferir("horn encosta na face +y", W(fmais).distance(servo) < 0.01, f"{W(fmais).distance(servo):.3f} mm")
    r = K["sts3215"]["horn_furos_circulo"] / 2
    vazio = 0.0
    for k in range(4):                                          # os furos da face caem nos furos do horn
        a = math.radians(45 + 90 * k)
        sonda = W(cil_y(r * math.cos(a), r * math.sin(a), 1.0, 50.3, 52.3))
        vazio = max(vazio, sonda.intersect(servo).Volume())
    conferir("furos da face +y alinhados com os 4 furos do horn (sonda Ø1 dentro do horn: vazia)", vazio < 1e-3, f"{vazio:.4f} mm³")
    Lh, eh = pi.parafuso_em_rosca_cega("M3", BE["face"]["y1"] - BE["face"]["y0"], prof_max_rosca=K["sts3215"]["horn_e"],
                                       folga_fundo=0.0, engajamento_min=2.0)
    print(f"  M3 do horn: 4 × M3×{Lh} ISO 4762 de dentro do berço ({eh:.1f} mm na rosca do horn)")
    cabeca_z = -(r / math.sqrt(2)) - pi.PARAFUSO_ISO4762["M3"]["dk_max"] / 2
    conferir("cabeças do horn passam no entalhe do piso (desceriam a z −7,7)", cabeca_z < BE["piso"]["z1"],
             f"cabeça até z {cabeca_z:.2f}; entalhe x ±9, 4 de fundo")
    y1 = TC["menos"]["y"][1]
    R = TC["r608"]
    rol = cil_y(0, ZT, R["D"], y1 - R["B"], y1).cut(cil_y(0, ZT, 8.0, y1 - R["B"] - 1, y1 + 1))
    anel_int = cil_y(0, ZT, 12.0, y1 - R["B"], y1).cut(cil_y(0, ZT, 8.0, y1 - R["B"] - 1, y1 + 1))
    pilha, aperto = pilha_m8(inf)
    conferir("ponta Ø10,4 encosta no anel interno do 608 (e não no braço)", W(fmenos).distance(anel_int) < 0.01
             and W(fmenos).intersect(bn).Volume() < 1e-3 and W(fmenos).distance(bn) >= 0.49,
             f"folga ressalto ↔ braço {W(fmenos).distance(bn):.2f} mm")
    pior = 0.0
    for nome, s in pilha:
        pior = max(pior, W(s).intersect(bn).Volume(), W(s).intersect(rol).Volume())
    conferir("arruela, cabeça e corpo do M8 × braço −y e 608: zero", pior < 1e-3, f"{pior:.4f} mm³")
    conferir("M8×20: a ponta chega à face de dentro da porca (porca inteira em rosca)", 0.0 <= 20.0 - aperto < 1.0,
             f"pilha {aperto:.1f} mm da cabeça à face de dentro da face −y; a ponta sai {20.0 - aperto:.1f}")

    print("\n4. Giro do tilt (−20° a +40°): o berço não bate em nada")
    berco = [("piso", piso), ("face +y", fmais), ("face −y", fmenos), ("motor −y", motor_130(BE["motor"]["x"], -BE["motor"]["y"])),
             ("motor +y", motor_130(BE["motor"]["x"], BE["motor"]["y"])), ("SG90", sg90())] + pilha
    fixos = [("braço +y", bm), ("braço −y", bn), ("mesa", mesa), ("servo do tilt", servo)]
    tmin, tmax = P["limites"]["tilt"]
    pior, quem, folga_mesa = 0.0, "", 1e9
    passos = [tmin + i * (tmax - tmin) / (6 if RAPIDO else 12) for i in range((6 if RAPIDO else 12) + 1)]
    for t in passos:
        for nb, sb in berco:
            wb = mundo(sb, t)
            for nf, sf in fixos:
                if nb in ("face +y",) and nf == "servo do tilt":
                    v = wb.intersect(servo).Volume()        # horn é de revolução: o giro não muda o encosto
                else:
                    v = wb.intersect(sf).Volume()
                if v > pior:
                    pior, quem = v, f"{nb} × {nf} a {t:.0f}°"
            folga_mesa = min(folga_mesa, wb.BoundingBox().zmin - P["pan"]["mesa"]["z1"])
    conferir("berço, motores, SG90 e M8 × braços, mesa e servo: zero em toda a faixa", pior < 1e-3, f"máx {pior:.4f} mm³ {quem}")
    conferir("ponto mais baixo do berço passa a mesa", folga_mesa > 2.0, f"folga mínima {folga_mesa:.1f} mm (plano: 4, com o SG90 e a lata)")

    print("\n5. Fios (tilt 0)")
    lista = fios()
    obst = [("piso", W(piso)), ("face +y", W(fmais)), ("face −y", W(fmenos)), ("braço −y", bn), ("mesa", mesa),
            ("motor −y", W(berco[3][1])), ("motor +y", W(berco[4][1])), ("SG90", W(berco[5][1]))]
    batidas, pior = [], 0.0
    for nome, cor, d, pts in lista:
        for parte in cx.tubo([(x, y, z + ZT) for x, y, z in pts], d):
            for no, ob in obst:
                v = parte.intersect(ob).Volume()
                if v > 1e-3:
                    batidas.append(f"{nome} × {no}: {v:.2f} mm³")
                pior = max(pior, v)
    for b_ in batidas:
        print("      bate:", b_)
    conferir(f"{len(lista)} fios: nenhum atravessa piso, faces, braço, mesa, motores ou SG90", pior < 1e-3, f"máx {pior:.3f} mm³")
    fc = BC["face_menos"]["furo_cabos"]
    print(f"  folga para o tilt: o feixe cruza o eixo a ~{math.hypot(fc['x'], fc['z']):.0f} mm dele; "
          f"60° de faixa pedem ≥ {math.radians(60) * math.hypot(fc['x'], fc['z']):.0f} mm de laço no vão (deixar 30)")

    print("\n6. Fabricação")
    for nome, s in (("piso", piso), ("face +y", fmais), ("face −y", fmenos)):
        bb = s.BoundingBox()
        dims = sorted([bb.xlen, bb.ylen, bb.zlen], reverse=True)
        conferir(f"{nome} cabe na Inventor (230 × 150 × 160)", dims[0] <= 230 and dims[1] <= 150 and dims[2] <= 160,
                 f"{bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f}")
    fm = BC["face_menos"]
    paredes = {"inserto dos ressaltos (x)": (8.0 - INS["furo_d"]) / 2,
               "sextavado ↔ ressalto Ø20 (nos cantos)": (fm["ressalto_d"] - (fm["porca_s"] + fm["porca_folga"]) / math.cos(math.pi / 6)) / 2,
               "ponta Ø10,4 ↔ furo M8": (fm["ponta_d"] - fm["furo_m8"]) / 2}
    for k, v in paredes.items():
        conferir(f"parede {k} ≥ 1,0", v >= 1.0 - 1e-9, f"{v:.2f} mm")
    exigir_tudo_ok(ok)

    print("\n7. Massa, exportação e render")
    for nome, s in (("piso", piso), ("face +y", fmais), ("face −y", fmenos)):
        m, _, _ = propriedades_solido(s, PLA["densidade_g_cm3"])
        print(f"  {nome}: {m * 1000:.0f} g (PLA maciço)")
    saida = os.path.join(PROJ, "cad_export")
    for nome, s in (("berco_piso", piso), ("berco_face_mais_y", fmais), ("berco_face_menos_y", fmenos)):
        exportar_stl_mm(cq.Compound.makeCompound([s]), saida, f"{nome}.stl")
        cq.exporters.export(cq.Workplane().add(s), os.path.join(saida, f"{nome}.step"))
    cores = {"vermelho": (0.85, 0.1, 0.1), "preto": (0.12, 0.12, 0.12), "laranja": (1.0, 0.55, 0.0),
             "amarelo": (0.95, 0.85, 0.1), "azul": (0.15, 0.35, 0.95), "verde": (0.1, 0.7, 0.2),
             "branco": (0.92, 0.92, 0.92), "cinza": (0.5, 0.5, 0.55)}
    prev = os.path.join(PROJ, "previews")
    for t, arq, pos in ((0.0, "berco.png", (230, -260, 200)), (40.0, "berco_tilt_40.png", (40, -330, 150)),
                        (-20.0, "berco_tilt_menos_20.png", (40, -330, 150))):
        itens = [(mundo(piso, t), (0.95, 0.80, 0.55)), (mundo(fmais, t), (0.90, 0.65, 0.40)), (mundo(fmenos, t), (0.90, 0.65, 0.40)),
                 (mundo(berco[3][1], t), (0.70, 0.72, 0.75)), (mundo(berco[4][1], t), (0.70, 0.72, 0.75)),
                 (mundo(berco[5][1], t), (0.15, 0.30, 0.75)), (bm, (0.55, 0.70, 0.55)), (bn, (0.55, 0.70, 0.55)),
                 (mesa, (0.60, 0.75, 0.90)), (servo, (0.2, 0.2, 0.22))] + [(mundo(s, t), (0.45, 0.45, 0.5)) for _, s in pilha]
        if t == 0.0:
            for nome, cor, d, pts in lista:
                itens += [(q, cores[cor]) for q in cx.tubo([(x, y, z + ZT) for x, y, z in pts], d)]
        render(itens, os.path.join(prev, arq), foco=(5, 0, 105), posicao=(pos[0], pos[1], pos[2]))
    print(f"  renders em {prev}/")


if __name__ == "__main__":
    main()
