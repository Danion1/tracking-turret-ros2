#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
caixa.py — peça 2b da parte B: a caixa da base (piso, paredes, colunas dos
insertos do tampo, berços e colunas das placas), com a eletrônica e os FIOS
desenhados como referência de montagem (pedido do Daniel, 2026-11-06).

Eletrônica (decisões do Daniel, 2026-11-06): fonte externa de 12 V 5 A no
jack PJ-002AH; 3 bucks XL4015 (7,0 V servos via Waveshare; 3,0 V motores;
5 V SG90); placa perfurada central com o ESP32-S3 em barras fêmeas, o
MOSFET AO3400A e o diodo 1N5819 dos motores. GND comum: o XL4015 não é
isolado (IN− = OUT−), então todos os negativos já são o mesmo nó.

Cada placa entra como ENVELOPE (a caixa que contém a placa, os componentes e
o espaço dos conectores). Os fios são tubos com bitola ASSUMIDA, num
corredor a z −20 (acima de todas as placas, abaixo do tampo), descendo até
cada borne. Os que sobem para a torre passam pelo furo de Ø20 do tampo.

Conferências: a caixa é 1 sólido válido; nenhum envelope cruza outro, a
caixa, o tampo ou o servo do pan; nenhum fio atravessa placa, parede,
coluna, tampo ou servo; o feixe que sobe cabe no furo; cabe na Inventor.

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/caixa.py
"""

import json
import math
import os
import sys

import cadquery as cq

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import usar_biblioteca  # noqa: E402,F401
import parafusos_iso as pi  # noqa: E402
import tampo as tp  # noqa: E402
from check_bbox import check_bbox, check_volume, exigir_tudo_ok  # noqa: E402
from dados_materiais import PLA  # noqa: E402
from exportar_stl_mm import exportar_stl_mm  # noqa: E402
from formas_basicas import box_minmax, cylinder_z_at  # noqa: E402
from massa_inercia import propriedades_solido  # noqa: E402
from render_cad import render  # noqa: E402

PROJ = os.path.dirname(AQUI)
P = json.load(open(os.path.join(PROJ, "plano", "parametros.json"), encoding="utf-8"))
CX, B, INS = P["caixa"], P["base"], P["pecas"]["inserto_m3"]
FI = CX["fios"]
ok = True


def conferir(nome, cond, detalhe=""):
    global ok
    print(f"  [{'OK' if cond else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok &= bool(cond)
    return cond


def caixa(x0, x1, y0, y1, z0, z1):
    return box_minmax(x0, x1, y0, y1, z0, z1).val()


def cil(d, z0, z1, x, y):
    return cylinder_z_at(x, y, z0, d, z1 - z0).val()


def comp(nome_inicio):
    return next(c for c in CX["comps"] if c["nome"].startswith(nome_inicio))


# ---------------------------------------------------------------- a caixa
def fazer_caixa():
    w, r, t = B["w"], B["r"], CX["parede"]
    z0, z1, zf = CX["z0"], CX["z1"], CX["z_piso"]
    fora = cq.Workplane("XY").workplane(offset=z0).rect(w, w).extrude(z1 - z0).edges("|Z").fillet(r).val()
    dentro = (cq.Workplane("XY").workplane(offset=zf).rect(w - 2 * t, w - 2 * t).extrude(z1 - zf + 1)
              .edges("|Z").fillet(r - t).val())
    s = fora.cut(dentro)
    adic, ferr = [], []
    # colunas dos 4 insertos do tampo (sobrepõem 0,3 no piso e nas paredes)
    c = CX["colunas"]["c"]
    prof = INS["L"] + INS["fundo_extra"]
    for sx in (-1, 1):
        for sy in (-1, 1):
            adic.append(cil(CX["colunas"]["d"], zf - 0.3, z1, sx * c, sy * c))
            ferr.append(cil(INS["furo_d"], z1 - prof, z1 + 1, sx * c, sy * c))
    # bucks: 2 trilhos sob a placa + cerca baixa com 0,3 de folga (fixação por fita dupla-face)
    for b in CX["comps"]:
        if b["tipo"] != "buck":
            continue
        (x0, x1), (y0, y1), (zb, _) = b["x"], b["y"], b["z"]
        for yy in (y0 + 2.0, y1 - 5.0):
            adic.append(caixa(x0 + 4, x1 - 4, yy, yy + 3.0, zf - 0.3, zb))
        g, e, h = 0.3, 1.2, zb + 3.0
        adic.append(caixa(x0 - g - e, x1 + g + e, y0 - g - e, y0 - g, zf - 0.3, h))
        adic.append(caixa(x0 - g - e, x1 + g + e, y1 + g, y1 + g + e, zf - 0.3, h))
    # colunas Ø6 das placas aparafusadas (M2, furo piloto Ø1,6 ASSUMIDO)
    ws = comp("Waveshare")
    cxw, cyw = sum(ws["x"]) / 2, sum(ws["y"]) / 2
    pb = comp("placa perfurada")
    pontos = [(cxw + sx * ws["furos"][0] / 2, cyw + sy * ws["furos"][1] / 2, ws["z"][0]) for sx in (-1, 1) for sy in (-1, 1)]
    pontos += [(x, y, pb["z"][0]) for x in (pb["x"][0] + 2.5, pb["x"][1] - 2.5) for y in (pb["y"][0] + 2.5, pb["y"][1] - 2.5)]
    for x, y, ztop in pontos:
        adic.append(cil(6.0, zf - 0.3, ztop, x, y))
        ferr.append(cil(1.6, ztop - 4.0, ztop + 1, x, y))
    # berço do jack: guias laterais, batente atrás (o plugue empurra) e a parede segura a frente
    jk = comp("jack")
    (jx0, jx1), (jy0, jy1), (jz0, jz1) = jk["x"], jk["y"], jk["z"]
    for xa, xb in ((jx0 - 1.8, jx0 - 0.2), (jx1 + 0.2, jx1 + 1.8)):
        adic.append(caixa(xa, xb, -w / 2 + t - 0.3, jy1, zf - 0.3, (jz0 + jz1) / 2))
    adic.append(caixa(jx0 - 1.8, jx1 + 1.8, jy1 + 0.2, jy1 + 2.2, (jz0 + jz1) / 2 - 2, jz1))   # batente (acima dos pinos)
    adic.append(caixa(jx0 - 1.8, jx1 + 1.8, jy1 + 0.2, jy1 + 2.2, zf - 0.3, (jz0 + jz1) / 2 - 1.9))
    for a in adic:
        s = s.fuse(a)
    # aberturas nas paredes
    for nome, (lado, u, zc, lw, lh) in CX["aberturas"].items():
        if nome.startswith("ventilacao"):
            for k in range(7):          # 7 fendas de 3 × 16, passo 10
                uu = u - 30 + 10 * k
                ferr.append(caixa(-w / 2 - 1, -w / 2 + t + 1, uu - 1.5, uu + 1.5, zc - lh / 2, zc + lh / 2))
            continue
        if lado == "+x":
            ferr.append(caixa(w / 2 - t - 1, w / 2 + 1, u - lw / 2, u + lw / 2, zc - lh / 2, zc + lh / 2))
        elif lado == "-y":
            ferr.append(caixa(u - lw / 2, u + lw / 2, -w / 2 - 1, -w / 2 + t + 1, zc - lh / 2, zc + lh / 2))
    for f in ferr:
        s = s.cut(f)
    return s


# ---------------------------------------------------------------- fios
# posição (x, y) de cada cabo que sobe pelo furo de Ø20 (dentro de r < 10); a peça 3 continua daqui
SOBE = {"bus": (-5.0, 3.0), "usb": (-3.0, -4.0), "m+": (3.0, 5.5), "m-": (5.5, 3.0),
        "sg1": (3.0, -3.0), "sg2": (5.5, -5.0), "sg3": (6.5, -1.5)}
def tubo(pts, d):
    """Fio como segmentos de cilindro + esferas nas dobras (lista de sólidos)."""
    r, partes = d / 2, []
    for a, b in zip(pts, pts[1:]):
        va, vb = cq.Vector(*a), cq.Vector(*b)
        L = (vb - va).Length
        if L < 1e-6:
            continue
        partes.append(cq.Solid.makeCylinder(r, L, va, (vb - va).normalized()))
    for q in pts[1:-1]:
        partes.append(cq.Solid.makeSphere(r, cq.Vector(*q), angleDegrees1=-90, angleDegrees2=90))
    return partes


def desloca(pts, dx, dy, dz=0.0):
    return [(x + dx, y + dy, z + dz) for x, y, z in pts]


def fios(servo_traseira):
    """Lista (nome, cor, d, pontos). Pares: + vermelho/laranja, − preto."""
    zc = FI["z_corredor"]
    dp, ds, dsv, du = FI["potencia_d"], FI["sinal_d"], FI["servo_d"], FI["usb_d"]
    b5, b3, b7 = comp("buck 5"), comp("buck 3"), comp("buck 7")
    ws, pb, jk = comp("Waveshare"), comp("placa perfurada"), comp("jack")
    ym = lambda c: sum(c["y"]) / 2                                      # noqa: E731
    zt = lambda c, d: c["z"][1] + d / 2 + 0.3                           # noqa: E731  (borne por cima)
    xw = CX["z_piso"]  # (só para legibilidade)
    del xw
    xin = b5["x"][0] - 1.0 - dp / 2                                     # bornes de entrada (ponta −x)
    xout = b5["x"][1] + 1.0 + dp / 2                                    # bornes de saída (ponta +x)
    zb = b5["z"][0] + 5.0                                               # altura do borne de parafuso
    xpar = -CX["colunas"]["c"] + 1.0                                    # descida junto da parede −x
    L = []
    # 12 V: jack -> buck 5 V -> buck 3 V -> buck 7 V (bornes de entrada aceitam 2 fios)
    j0 = (sum(jk["x"]) / 2, jk["y"][1] + 3.2 + dp / 2, jk["z"][0] - 1.5)    # pinos por baixo/atrás
    p12 = [j0, (j0[0], j0[1], zc), (xpar, j0[1], zc), (xpar, ym(b5), zc), (xpar, ym(b5), zb), (xin, ym(b5), zb)]
    L.append(("12 V + : jack → buck 5 V", "vermelho", dp, p12))
    L.append(("12 V − : jack → buck 5 V", "preto", dp, desloca(p12, 0, 0, dp + 0.3)))
    for a, b, n in ((b5, b3, "buck 5 V → buck 3 V"), (b3, b7, "buck 3 V → buck 7 V")):
        q = [(xin, ym(a) + 3, zb), (xpar - 2.5, ym(a) + 3, zb), (xpar - 2.5, ym(b) + 3, zb), (xin, ym(b) + 3, zb)]
        L.append((f"12 V + : {n}", "vermelho", dp, q))
        L.append((f"12 V − : {n}", "preto", dp, desloca(q, 0, dp + 0.3, 0)))
    # 7 V: buck 7 V -> borne da Waveshare (por cima da placa)
    wpt = (ws["x"][1] - 6.0, ws["y"][0] + 6.0)
    q = [(xout, ym(b7), zb), (xout + 3, ym(b7), zb), (xout + 3, ym(b7), zc), (wpt[0], ym(b7), zc), (wpt[0], wpt[1], zc),
         (wpt[0], wpt[1], zt(ws, dp))]
    L.append(("7 V + : buck 7 V → Waveshare", "laranja", dp, q))
    L.append(("7 V − : buck 7 V → Waveshare", "preto", dp, desloca(q, 0, dp + 0.3, 0)))
    # 3 V e 5 V: bucks -> bornes da placa perfurada, nas faixas livres dela (fora do ESP32), por cima
    for bb, n, cor, yy, sinal in ((b3, "3 V", "amarelo", pb["y"][1] - 5.0, -1), (b5, "5 V", "laranja", pb["y"][0] + 4.0, 1)):
        xb2 = pb["x"][0] + 4.0
        q = [(xout, ym(bb), zb), (xout + 3, ym(bb), zb), (xout + 3, ym(bb), zc), (xb2, ym(bb), zc),
             (xb2, yy, zc), (xb2, yy, zt(pb, dp))]
        L.append((f"{n} + : buck → placa perfurada", cor, dp, q))
        L.append((f"{n} − : buck → placa perfurada", "preto", dp, desloca(q, 0, sinal * (dp + 0.3), 0)))
    # barramento: Waveshare -> servo do pan (o cabo do servo sai da traseira dele)
    st = servo_traseira
    q = [st, (st[0], 18.0, st[2]), (st[0], 18.0, zc), (wpt[0] - 12, 18.0, zc), (wpt[0] - 12, wpt[1] + 14, zc),
         (wpt[0] - 12, wpt[1] + 14, zt(ws, dsv))]
    L.append(("barramento: Waveshare → servo do pan", "azul", dsv, q))
    # feixe que sobe pelo furo de Ø20 (até o topo do cubo, z 8,5); posições dentro de r < 10
    topo = P["pan"]["cubo"]["assento_z1"]
    sobe = SOBE
    xb_ = wpt[0] - 20
    q = [(xb_, wpt[1] + 14, zt(ws, dsv)), (xb_, wpt[1] + 14, zc), (xb_, sobe["bus"][1], zc),
         (sobe["bus"][0], sobe["bus"][1], zc), (sobe["bus"][0], sobe["bus"][1], topo)]
    L.append(("barramento: Waveshare → servo do tilt (sobe)", "azul", dsv, q))
    # UART: placa perfurada (ESP32) -> Waveshare, 3 fios (TX, RX, GND)
    for k, cor in enumerate(("verde", "branco", "preto")):
        y0 = pb["y"][1] - 2.0 - 1.6 * k
        q = [(pb["x"][0] + 8.0, y0, zt(pb, ds)), (pb["x"][0] + 8.0, y0, zc + 2.0 + 1.6 * k),
             (ws["x"][0] + 8.0, y0, zc + 2.0 + 1.6 * k), (ws["x"][0] + 8.0, ws["y"][0] + 8.0 + 1.6 * k, zc + 2.0 + 1.6 * k),
             (ws["x"][0] + 8.0, ws["y"][0] + 8.0 + 1.6 * k, zt(ws, ds))]
        L.append((f"UART {('TX', 'RX', 'GND')[k]}: ESP32 → Waveshare", cor, ds, q))
    # motores e SG90: placa perfurada -> sobem
    for nome, cor, d, x0 in (("m+", "amarelo", dp, 50.0), ("m-", "preto", dp, 53.0),
                             ("sg1", "laranja", ds, 40.0), ("sg2", "preto", ds, 42.0), ("sg3", "branco", ds, 44.0)):
        sx, sy = sobe[nome]
        y0 = pb["y"][1] - 1.5
        q = [(x0, y0, zt(pb, d)), (x0, y0, zc - 2.0), (sx + 0.0, y0, zc - 2.0), (sx, sy, zc - 2.0), (sx, sy, topo)]
        rot = {"m+": "motores + (3 V)", "m-": "motores − (dreno do AO3400A)",
               "sg1": "SG90 V+ (5 V)", "sg2": "SG90 GND", "sg3": "SG90 sinal (ESP32)"}[nome]
        L.append((rot + " (sobe)", cor, d, q))
    # USB da webcam: desce pelo furo e sai pela abertura da parede +x
    ux, uz = CX["aberturas"]["usb_webcam"][1], CX["aberturas"]["usb_webcam"][2]
    q = [(sobe["usb"][0], sobe["usb"][1], topo), (sobe["usb"][0], sobe["usb"][1], uz), (sobe["usb"][0], ux, uz),
         (B["w"] / 2 + 6.0, ux, uz)]
    L.append(("USB da webcam: desce pelo furo e sai pela parede +x (vai ao PC)", "cinza", du, q))
    return L


def main():
    print("1. Peça")
    cx_ = fazer_caixa()
    n = len(cx_.Solids())
    conferir("caixa: 1 sólido válido, volume > 0", n == 1 and cx_.isValid() and cx_.Volume() > 0,
             f"{n} sólido(s), {cx_.Volume():.0f} mm³")
    print("\n2. Caixa envolvente e volume contra contas à mão")
    ok_ = check_bbox("caixa", cx_, (-70, 70, -70, 70, -45.0, -6.0), tol=0.06)
    # à mão: casca = 140²−(4−π)·7²  × 39  −  (135,2²−(4−π)·4,6²) × 36,6  = 762 763 − 668 342 = 94 421;
    # colunas 4 × π·5²·36,6 = 11 496 (quase inteiras dentro: o canto de dentro tem raio 4,6 e o MESMO centro
    # da coluna, então só sai uma lasca de 0,4); − insertos 4 × π·2²·6,7 = 337;
    # trilhos 6 × 43 × 3 × 2,8 = 2 167; cercas 6 × 53,7 × 1,2 × 5,8 = 2 243; colunas Ø6: 8 × π·3²·4,3 − pilotos ≈ 900;
    # berço do jack ≈ 600; − aberturas (22×9 + 18×10 + 9,4×11,4 + 7 × 3×16) × 2,4 ≈ 1 970  ->  ≈ 107 520
    ok_ &= check_volume("caixa", cx_, 107520, tol=0.05 * 107520)
    conferir("caixa e volume", ok_)

    print("\n3. Placas: nada se cruza")
    tampo_, _ = tp.fazer_tampo()
    servo = tp.para_mundo(cq.importers.importStep(tp.STEP).val())
    env = {c["nome"]: caixa(c["x"][0], c["x"][1], c["y"][0], c["y"][1], c["z"][0], c["z"][1]) for c in CX["comps"]}
    jack = comp("jack")["nome"]
    pior, quem = 0.0, ""
    nomes = list(env)
    for i, a in enumerate(nomes):
        for b in nomes[i + 1:]:
            v = env[a].intersect(env[b]).Volume()
            if v > pior:
                pior, quem = v, f"{a} × {b}"
        for nb, ob in (("caixa", cx_), ("tampo", tampo_), ("servo do pan", servo)):
            v = env[a].intersect(ob).Volume()
            if a == jack and nb == "caixa":
                continue    # o jack encaixa na abertura e no berço (conferido à parte abaixo)
            if v > pior:
                pior, quem = v, f"{a} × {nb}"
    conferir("envelopes × envelopes, caixa, tampo e servo: zero", pior < 1e-3, f"máx {pior:.3f} mm³ {quem}")
    jk = comp("jack")
    corpo_jack = caixa(jk["x"][0], jk["x"][1], jk["y"][0], jk["y"][1], jk["z"][0], jk["z"][1])
    conferir("jack × caixa: zero (a abertura e o berço têm folga)", corpo_jack.intersect(cx_).Volume() < 1e-3,
             f"{corpo_jack.intersect(cx_).Volume():.3f} mm³")
    conferir("boca do jack rente à face de fora da parede", abs(jk["y"][0] + B["w"] / 2) < 1e-6)
    sb = servo.BoundingBox()
    conferir("servo do pan acima do piso", sb.zmin > CX["z_piso"] + 1.0, f"fundo do servo z {sb.zmin:.1f}, piso {CX['z_piso']}")

    print("\n4. Fios")
    # traseira do servo (saída do cabo): ponto além da caixa dele, na direção u, a meia altura
    u = P["servo_pan"]["u"]
    e = P["servo_pan"]["eixo"]
    dtras = 22.7 + P["servo_pan"]["x_eixo_step"] + 1.0 + FI["servo_d"] / 2
    st = (e[0] + dtras * u[0], e[1] + dtras * u[1], -26.0)
    lista = fios(st)
    obst = [(k, v) for k, v in env.items()] + [("caixa", cx_), ("tampo", tampo_), ("servo do pan", servo)]
    pior, quem, batidas = 0.0, "", []
    comprimento = {}
    for nome, cor, d, pts in lista:
        comprimento[nome] = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        for parte in tubo(pts, d):
            for no, ob in obst:
                v = parte.intersect(ob).Volume()
                if v > 1e-3:
                    batidas.append(f"{nome} × {no}: {v:.2f} mm³")
                if v > pior:
                    pior, quem = v, f"{nome} × {no}"
    conferir(f"{len(lista)} fios: nenhum atravessa placa, parede, coluna, tampo ou servo", pior < 1e-3,
             f"máx {pior:.3f} mm³ {quem}")
    for b_ in sorted(set(batidas)):
        print("      bate:", b_)
    sobe = [d for nome, _, d, _ in lista if "sobe" in nome or "webcam" in nome]
    area = sum(math.pi * (d / 2) ** 2 for d in sobe)
    conferir("feixe que passa no furo de Ø20: ocupação < 40 %", area / (math.pi * 10 ** 2) < 0.40,
             f"{len(sobe)} cabos, {100 * area / (math.pi * 100):.0f} % da área")

    print("\n5. Fabricação")
    bb = cx_.BoundingBox()
    conferir("cabe na Inventor (230 × 150 × 160)", bb.xlen <= 230 and bb.ylen <= 150 and bb.zlen <= 160,
             f"{bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f}")
    conferir("parede do inserto na coluna ≥ 1,6", (CX["colunas"]["d"] - INS["furo_d"]) / 2 >= INS["parede_min"],
             f"{(CX['colunas']['d'] - INS['furo_d']) / 2:.1f} mm")
    exigir_tudo_ok(ok)

    print("\n6. Massa, lista de fios, exportação e render")
    m, _, _ = propriedades_solido(cx_, PLA["densidade_g_cm3"])
    print(f"  caixa: {m * 1000:.0f} g (PLA maciço)")
    print("  fios (comprimento no CAD; cortar com +30 mm de sobra):")
    for nome, cor, d, _ in lista:
        print(f"    {nome:<62} {cor:<9} Ø{d:<4} {comprimento[nome]:6.0f} mm")
    saida = os.path.join(PROJ, "cad_export")
    exportar_stl_mm(cq.Compound.makeCompound([cx_]), saida, "caixa.stl")
    cq.exporters.export(cq.Workplane().add(cx_), os.path.join(saida, "caixa.step"))
    cores = {"vermelho": (0.85, 0.1, 0.1), "preto": (0.12, 0.12, 0.12), "laranja": (1.0, 0.55, 0.0),
             "amarelo": (0.95, 0.85, 0.1), "azul": (0.15, 0.35, 0.95), "verde": (0.1, 0.7, 0.2),
             "branco": (0.92, 0.92, 0.92), "cinza": (0.5, 0.5, 0.55)}
    cor_placa = {"buck": (0.80, 0.25, 0.20), "waveshare": (0.10, 0.45, 0.25), "perfurada": (0.75, 0.62, 0.35),
                 "esp32": (0.15, 0.15, 0.20), "jack": (0.05, 0.05, 0.05)}
    itens = [(cx_, (0.85, 0.85, 0.80))]
    itens += [(env[c["nome"]], cor_placa[c["tipo"]]) for c in CX["comps"]]
    itens += [(servo, (0.25, 0.25, 0.28))]
    for nome, cor, d, pts in lista:
        itens += [(pp, cores[cor]) for pp in tubo(pts, d)]
    prev = os.path.join(PROJ, "previews")
    render(itens, os.path.join(prev, "caixa_por_cima.png"), foco=(0, 0, -28), posicao=(0, -1, 230), zoom=1.15,
           opacidade={len(CX["comps"]) + 1: 0.5})
    render(itens, os.path.join(prev, "caixa_perspectiva.png"), foco=(0, 0, -28), posicao=(150, -170, 150),
           opacidade={len(CX["comps"]) + 1: 0.5})
    render(itens + [(tampo_, (0.85, 0.85, 0.80))], os.path.join(prev, "caixa_fechada.png"), foco=(0, 0, -20),
           posicao=(170, -190, 110))
    print(f"  renders em {prev}/")


if __name__ == "__main__":
    main()
