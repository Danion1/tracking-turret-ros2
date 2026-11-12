#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lancador.py — peça 5 da parte B: o lançador, montado no berço (frame do berço).

Peças impressas:
  volante (2×)     Ø36 × 10, prensado no eixo Ø2 do FA-130RA (furo Ø1,9, ASSUMIDO).
  copo do motor (2×) o FA-130RA não tem furos na face (Mabuchi): copo em D
                   (posiciona e não deixa girar), fenda + canal para abraçadeira, e
                   um braço até o furo da gaiola (86, ±40), onde o mesmo M3 prende tudo.
  gaiola           tampa sobre os volantes + 4 colunas Ø7,2 (impressa de cabeça
                   para baixo); 4 × M3×25 + porca atravessam tampa, coluna e piso.
  funil            2 paredes (a calha do dardo, 0,1 de folga por lado), fundo e
                   frente numa peça; 3 dardos + 1 na câmara; colado no piso.
  cremalheira m1   empurra o dardo; dentes no lado +y, onde está o pinhão.
  pinhão do SG90   20T m1, no horn do SG90 (horn não cotado: furar os M2 na montagem).
  guia             parede + aba do lado −y da cremalheira (a força do dente empurra
                   a cremalheira para −y), colada no piso.
Fios: a USB da webcam sobe pelo furo do piso até acima da gaiola (a peça 6 liga).

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/lancador.py [--rapido]
"""

import json
import math
import os
import sys

import cadquery as cq

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import usar_biblioteca  # noqa: E402,F401
import berco as bc  # noqa: E402
import caixa as cx  # noqa: E402
import engrenagem as eg  # noqa: E402
import mesa_bracos as mb  # noqa: E402
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
BE, BC, LC, K = P["berco"], P["berco_cad"], P["lancador_cad"], P["pecas"]
PIS, MOT, VO = BE["piso"], BE["motor"], BE["volante"]
RAPIDO = "--rapido" in sys.argv
ok = True


def conferir(nome, cond, detalhe=""):
    global ok
    print(f"  [{'OK' if cond else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok &= bool(cond)
    return cond


def caixa(x0, x1, y0, y1, z0, z1):
    return box_minmax(x0, x1, y0, y1, z0, z1).val()


def cil(d, z0, z1, x=0.0, y=0.0):
    return cylinder_z_at(x, y, z0, d, z1 - z0).val()


def em_d(x, y, r, meia, z0, z1):
    """Prisma em D: círculo de raio r cortado por |y − yc| ≤ meia (as faces planas da lata do 130)."""
    return cil(2 * r, z0, z1, x, y).intersect(caixa(x - r - 1, x + r + 1, y - meia, y + meia, z0 - 1, z1 + 1))


# ---------------------------------------------------------------- peças
def fazer_volante(y):
    v = LC["volante"]
    s = cil(2 * VO["r"], -VO["l"] / 2, VO["l"] / 2, VO["x"], y)
    s = s.fuse(cil(2 * v["cubo_r"], v["z0"], -VO["l"] / 2 + 0.3, VO["x"], y))
    return s.cut(cil(v["furo"], v["z0"] - 1, VO["l"] / 2 + 1, VO["x"], y))


def fazer_copo(sy):
    c, m = LC["copo"], K["motor130"]
    x, y = MOT["x"], sy * MOT["y"]
    ri, mi = m["larg"] / 2 + c["folga"], m["plano"] / 2 + c["folga"]
    ro, mo = ri + c["parede"], mi + c["parede"]
    z0, z1 = c["fundo_z"], PIS["z0"]
    s = em_d(x, y, ro, mo, z0, z1)
    hx, hy = BC["piso_furos"]["gaiola"]["pontos"][2 if sy > 0 else 3]          # (86, ±40)
    ang = math.degrees(math.atan2(hy - y, hx - x))
    L = math.hypot(hx - x, hy - y)
    braco = (cq.Workplane("XY").workplane(offset=z1 - c["braco_e"]).center(x, y).transformed(rotate=(0, 0, ang))
             .center(L / 2, 0).rect(L, c["braco_l"]).extrude(c["braco_e"]).val())
    s = s.fuse(braco).fuse(cil(c["braco_l"], z1 - c["braco_e"], z1, hx, hy))
    ferr = [em_d(x, y, ri, mi, z0 - 1, z1 + 1),                                  # o furo em D atravessa o braço também
            cil(pi.furo_passagem("M3"), z1 - c["braco_e"] - 1, z1 + 1, hx, hy),
            caixa(x - ro - 1, x - ri + 0.01, y - c["fenda"] / 2, y + c["fenda"] / 2, z0 - 1, z1 - c["braco_e"])]  # fenda (lado −x)
    ca = c["canal_abracadeira"]
    anel = em_d(x, y, ro + 1, mo + 1, ca["z"] - ca["l"] / 2, ca["z"] + ca["l"] / 2).cut(
        em_d(x, y, ro - ca["prof"], mo - ca["prof"], ca["z"] - ca["l"] / 2 - 1, ca["z"] + ca["l"] / 2 + 1))
    ferr.append(anel)
    for f in ferr:
        s = s.cut(f)
    return s


def fazer_gaiola():
    G, g = BE["gaiola"], LC["gaiola"]
    z0 = G["z0"]
    tampa = (cq.Workplane("XY").workplane(offset=z0).center((G["x0"] + G["x1"]) / 2, 0)
             .rect(G["x1"] - G["x0"], 2 * g["tampa_y"]).extrude(g["tampa_e"]).edges("|Z").fillet(3.0).val())
    s = tampa
    pts = BC["piso_furos"]["gaiola"]["pontos"]
    for x, y in pts:
        s = s.fuse(cil(g["coluna_d"], PIS["z1"], z0 + 0.3, x, y))
    for x, y in pts:
        s = s.cut(cil(pi.furo_passagem("M3"), PIS["z1"] - 1, z0 + g["tampa_e"] + 1, x, y))
    return s


def fazer_funil():
    ca, f = BE["calha"], LC["funil"]
    s = None
    for sy in (1, -1):
        y0, y1 = (ca["y0"], ca["y1"]) if sy > 0 else (-ca["y1"], -ca["y0"])
        w = caixa(ca["x0"], ca["x1"], y0, y1, PIS["z1"], f["topo"])
        s = w if s is None else s.fuse(w)
    s = s.fuse(caixa(ca["x0"] - f["esp"], ca["x0"] + 0.3, -ca["y1"], ca["y1"], f["fundo_z0"], f["topo"]))
    s = s.fuse(caixa(ca["x1"] - 0.3, ca["x1"] + f["esp"], -ca["y1"], ca["y1"], f["frente_z0"], f["topo"]))
    return s


EP = BE["empurrador"]


def fazer_cremalheira():
    c = LC["cremalheira"]
    m, p = EP["m"], math.pi * EP["m"]
    yp = EP["y_primitiva"]
    raiz = yp - 1.25 * m
    corpo = caixa(c["x0"], c["x1"], c["y0"], raiz, PIS["z1"], c["z1"])     # até a raiz: o perfil traz 1 mm de corpo abaixo dela (sobrepõe)
    n = int((c["x1"] - c["x0"]) / p) + 3
    pts = eg.perfil_cremalheira(m, n, altura_corpo=1.0, folga_flanco=c["folga_flanco"])
    # fase: um VÃO da cremalheira no eixo do pinhão (x_pinhao) com o empurrador recolhido
    k0 = round((EP["x_pinhao"] - c["x0"]) / p)
    sx = EP["x_pinhao"] - (k0 + 0.5) * p
    dentes = (cq.Workplane("XY").workplane(offset=c["dente_z"][0]).polyline([(x + sx + p * 0, y + yp) for x, y in pts])
              .close().extrude(c["dente_z"][1] - c["dente_z"][0]).val())
    dentes = dentes.intersect(caixa(c["x0"], c["x1"], raiz - 0.5, yp + m + 1, c["dente_z"][0] - 1, c["dente_z"][1] + 1))
    return corpo.fuse(dentes)


def fazer_pinhao_sg90():
    pz = LC["pinhao_sg90"]
    xc, yc = EP["x_pinhao"], EP["y_primitiva"] + EP["rp"]
    pts = eg.perfil_pinhao(EP["m"], EP["z"], folga_flanco=pz["folga"] / 2)
    s = cq.Workplane("XY").workplane(offset=pz["z"][0]).polyline(pts).close().extrude(pz["z"][1] - pz["z"][0]).val()
    s = s.rotate(cq.Vector(), cq.Vector(0, 0, 1), -90.0).translate(cq.Vector(xc, yc, 0))   # dente apontando para −y (no vão)
    return s.cut(cil(pz["furo_d"], pz["z"][0] - 1, pz["z"][1] + 1, xc, yc))


def fazer_guia():
    g, c = LC["guia"], LC["cremalheira"]
    ya = c["y0"] - g["folga"]
    parede = caixa(c["x0"], g["x1"], ya - g["parede"], ya, PIS["z1"] - 0.0, c["z1"] + g["folga"] + g["aba"])
    aba = caixa(c["x0"], g["x1"], ya - g["parede"], c["y0"] + g["aba"], c["z1"] + g["folga"], c["z1"] + g["folga"] + g["aba"])
    return parede.fuse(aba)


def dardo(x0, z=0.0):
    d = K["dardo"]
    return cq.Workplane("YZ").workplane(offset=x0).center(0, z).circle(d["d"] / 2).extrude(d["L"]).val()


def posar_empurrador(crem, pin, s):
    """Curso s (mm): a cremalheira anda +x; o pinhão (acima, em +y) gira +s/rp (anti-horário: o ponto de
    baixo dele anda +x junto com a cremalheira)."""
    xc, yc = EP["x_pinhao"], EP["y_primitiva"] + EP["rp"]
    return (crem.translate(cq.Vector(s, 0, 0)),
            pin.rotate(cq.Vector(xc, yc, 0), cq.Vector(xc, yc, 1), math.degrees(s / EP["rp"])))


def main():
    print("1. Peças")
    va, vb = fazer_volante(VO["y"]), fazer_volante(-VO["y"])
    ca_, cb_ = fazer_copo(1), fazer_copo(-1)
    gai, fun, crem, pin, gui = fazer_gaiola(), fazer_funil(), fazer_cremalheira(), fazer_pinhao_sg90(), fazer_guia()
    pecas = [("volante +y", va), ("volante −y", vb), ("copo +y", ca_), ("copo −y", cb_), ("gaiola", gai),
             ("funil", fun), ("cremalheira", crem), ("pinhão do SG90", pin), ("guia", gui)]
    for nome, s in pecas:
        n = len(s.Solids())
        conferir(f"{nome}: 1 sólido válido, volume > 0", n == 1 and s.isValid() and s.Volume() > 0, f"{n} sólido(s), {s.Volume():.0f} mm³")

    print("\n2. Caixa e volume contra contas à mão")
    ok_ = check_bbox("volante +y", va, (47, 83, 4.825, 40.825, -6, 5), tol=0.06)
    ok_ &= check_bbox("gaiola", gai, (48.4, 90, -46, 46, -6.5, 9.5), tol=0.06)
    ok_ &= check_bbox("funil", fun, (-31.5, 47.9, -8.6, 8.6, -6.5, 48.1), tol=0.06)
    ok_ &= check_bbox("cremalheira", crem, (-91, -29.5, -2.5, 5.0, -6.5, 5.5), tol=0.1)
    # volante π·18²·10 + cubo π·5²·1 − furo π·0,95²·11 = 10 179 + 79 − 31 = 10 227
    ok_ &= check_volume("volante +y", va, 10227, tol=0.02 * 10227)
    # gaiola: tampa 41,6 × 92 × 3 = 11 482 − 4 furos 109 + 4 colunas π(3,6²−1,7²)·13 = 1 645 -> 13 018
    ok_ &= check_volume("gaiola", gai, 13018, tol=0.03 * 13018)
    # funil: paredes 2 × 74,4 × 2 × 54,6 = 16 248; fundo 2,5 × 17,2 × 40,6 = 1 746; frente 2,5 × 17,2 × 39,6 = 1 703 -> 19 697
    ok_ &= check_volume("funil", fun, 19697, tol=0.02 * 19697)
    # cremalheira: corpo 61,5 × 5,25 × 12 = 3 875 + dentes ≈ 19,5 × (1,42 × 2,25) × 6 = 374 -> 4 249
    ok_ &= check_volume("cremalheira", crem, 4249, tol=0.05 * 4249)
    # pinhão: π·10²·6 (área ≈ π·rp²) − folga 20 dentes × 0,15 × 2,25 × 6 = 41 − furo π·2,5²·6 = 118 -> 1 726
    ok_ &= check_volume("pinhão do SG90", pin, 1726, tol=0.04 * 1726)
    # copo: parede em D (387,7 − 281,0) × 15 = 1 600 − fenda 48 − canal ≈ 150 + braço ≈ 489 -> ≈ 1 890
    ok_ &= check_volume("copo −y", cb_, 1890, tol=0.10 * 1890)
    conferir("caixa e volume", ok_)

    print("\n3. Montagem (tilt 0, frame do berço)")
    piso = bc.fazer_piso()
    mot = {+1: bc.motor_130(MOT["x"], MOT["y"]), -1: bc.motor_130(MOT["x"], -MOT["y"])}
    sg = bc.sg90()
    vao = (VO["y"] - VO["r"]) * 2
    conferir("vão entre os volantes = 9,65 (dardo de 13: compressão de 3,35)", abs(vao - VO["folga"]) < 1e-6, f"{vao:.2f} mm")
    aperto = va.intersect(mot[1]).Volume() + vb.intersect(mot[-1]).Volume()
    conferir("volantes × eixos dos motores: só o aperto do furo Ø1,9 no eixo Ø2", 0.5 < aperto < 6.0, f"{aperto:.2f} mm³")
    fixos = [("piso", piso), ("motor +y", mot[1]), ("motor −y", mot[-1]), ("SG90", sg)]
    moveis = [("volante +y", va), ("volante −y", vb), ("copo +y", ca_), ("copo −y", cb_), ("gaiola", gai), ("funil", fun),
              ("cremalheira", crem), ("pinhão do SG90", pin), ("guia", gui)]
    pior, quem = 0.0, ""
    todos = fixos + moveis
    for i, (na, sa) in enumerate(todos):
        for nb, sb in todos[i + 1:]:
            if {na, nb} in ({"volante +y", "motor +y"}, {"volante −y", "motor −y"}, {"copo +y", "motor +y"}, {"copo −y", "motor −y"}):
                continue
            v = sa.intersect(sb).Volume()
            if v > pior:
                pior, quem = v, f"{na} × {nb}"
    conferir("todas as peças e compradas entre si: zero (fora o aperto do volante)", pior < 1e-3, f"máx {pior:.4f} mm³ {quem}")
    for nome, s in (("copo +y", ca_), ("copo −y", cb_)):
        m_ = mot[1 if "+" in nome else -1]
        conferir(f"{nome}: motor dentro com folga (sem tocar), encostado no piso", s.intersect(m_).Volume() < 1e-3
                 and s.distance(piso) < 0.01 and 0.1 <= s.distance(m_) <= 0.2, f"folga {s.distance(m_):.2f} mm")
    d0 = dardo(BE["calha"]["x0"] + 0.5)
    conferir("dardo na câmara × funil e piso: zero (0,1 de folga na calha)", d0.intersect(fun).Volume() < 1e-3 and d0.intersect(piso).Volume() < 1e-3)
    conferir("dardo parado não toca os volantes", d0.intersect(va).Volume() + d0.intersect(vb).Volume() < 1e-3)
    pilha = [dardo(BE["calha"]["x0"] + 0.5, k * BE["funil"]["passo"]) for k in range(1, BE["funil"]["n"] + 1)]
    conferir("3 dardos no funil cabem (abaixo do topo, sem tocar as paredes)",
             all(p.intersect(fun).Volume() < 1e-3 for p in pilha) and pilha[-1].BoundingBox().zmax < LC["funil"]["topo"])

    print("\n4. Empurrador: pinhão × cremalheira no curso inteiro")
    curso = EP["curso"]
    passos = [curso * i / (8 if RAPIDO else 40) for i in range((8 if RAPIDO else 40) + 1)]
    pior = 0.0
    for s in passos:
        c_, p_ = posar_empurrador(crem, pin, s)
        pior = max(pior, c_.intersect(p_).Volume(), c_.intersect(gui).Volume(), c_.intersect(fun).Volume(), c_.intersect(piso).Volume())
    conferir(f"pinhão × cremalheira, guia, funil e piso: zero em {len(passos)} posições do curso", pior < 1e-3, f"máx {pior:.4f} mm³")
    c_, p_ = posar_empurrador(crem, pin, 0.0)
    dist = c_.distance(p_)
    conferir("dentes perto (engrena)", dist < 0.3, f"{dist:.3f} mm")
    p_neg = pin.translate(cq.Vector(0, -0.6, 0))
    conferir("NEGATIVO: aproximar o pinhão 0,6 cruza", crem.intersect(p_neg).Volume() > 1e-3)
    c_fim, _ = posar_empurrador(crem, pin, curso)
    anda = curso - 1.0                                          # 1,0 de vão antes de encostar no dardo
    d_fim = dardo(BE["calha"]["x0"] + 0.5 + anda)
    # Desde 2026-11-10 a ponta ENTRA ~2 mm entre os volantes (pegada): com o curso antigo ela parava 0,36 mm
    # antes do toque e o dardo não saía no Gazebo. A sobreposição dardo × volante aqui é a espuma comprimida.
    toque = VO["x"] - math.sqrt(VO["r"] ** 2 - (VO["y"] - K["dardo"]["d"] / 2) ** 2)
    entra = BE["calha"]["x0"] + 0.5 + anda + K["dardo"]["L"] - toque
    comprime = d_fim.intersect(va).Volume() + d_fim.intersect(vb).Volume()
    conferir("no fim do curso a cremalheira encosta no dardo e a ponta entra 2 mm entre os volantes (pegada)",
             c_fim.distance(d_fim) < 0.01 and 1.5 <= entra <= 2.5 and comprime > 0,
             f"dardo andou {anda:.1f}; ponta {entra:+.2f} mm além do toque; espuma comprimida {comprime:.1f} mm³")

    print("\n5. Giro do tilt (−20° a +40°) e fios")
    servo = mb.servo_para_mundo(cq.importers.importStep(tp.STEP).val())
    bm, _ = mb.fazer_braco_mais()
    bn, _ = mb.fazer_braco_menos()
    mesa = mb.fazer_mesa()
    tmin, tmax = P["limites"]["tilt"]
    pior, quem, folga_mesa = 0.0, "", 1e9
    for t in [tmin + i * (tmax - tmin) / (4 if RAPIDO else 12) for i in range((4 if RAPIDO else 12) + 1)]:
        for nb, sb in moveis:
            wb = bc.mundo(sb, t)
            for nf, sf in (("braço +y", bm), ("braço −y", bn), ("mesa", mesa), ("servo do tilt", servo)):
                v = wb.intersect(sf).Volume()
                if v > pior:
                    pior, quem = v, f"{nb} × {nf} a {t:.0f}°"
            folga_mesa = min(folga_mesa, wb.BoundingBox().zmin - P["pan"]["mesa"]["z1"])
    conferir("lançador × braços, mesa e servo: zero em toda a faixa", pior < 1e-3, f"máx {pior:.4f} mm³ {quem}")
    conferir("ponto mais baixo do lançador passa a mesa", folga_mesa > 2.0, f"folga mínima {folga_mesa:.1f} mm")
    u = BC["piso_furos"]["usb_webcam"]
    up = LC["usb_ponta"]
    usb = [(u["x"], u["y"], PIS["z1"] + 6.0), (u["x"], u["y"], up[2]), (up[0], u["y"], up[2])]
    v = sum(t.intersect(o).Volume() for t in cx.tubo(usb, P["caixa"]["fios"]["usb_d"]) for _, o in fixos + moveis)
    conferir("USB da webcam sobe pelo furo do piso até acima da gaiola sem tocar nada", v < 1e-3, f"{v:.4f} mm³")
    fios4 = bc.fios()
    v = 0.0
    for nome, cor, d, pts in fios4:
        for t in cx.tubo(pts, d):
            for o in (ca_, cb_, gai, crem, gui):
                v = max(v, t.intersect(o).Volume())
    conferir("fios da peça 4 (sob o piso) × copos, gaiola, cremalheira e guia: zero", v < 1e-3, f"{v:.4f} mm³")

    print("\n6. Fabricação e fixação")
    for nome, s in pecas:
        bb = s.BoundingBox()
        dims = sorted([bb.xlen, bb.ylen, bb.zlen], reverse=True)
        conferir(f"{nome} cabe na Inventor", dims[0] <= 230 and dims[1] <= 150 and dims[2] <= 160, f"{bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f}")
    g = LC["gaiola"]
    pts = BC["piso_furos"]["gaiola"]["pontos"]
    gap_col = min(math.hypot(x - VO["x"], y - s_ * VO["y"]) - VO["r"] - g["coluna_d"] / 2 for x, y in pts for s_ in (1, -1))
    conferir("colunas da gaiola longe dos volantes (giram a 12 300 rpm)", gap_col >= 1.0, f"{gap_col:.2f} mm")
    conferir("parede das colunas ≥ 1,6", (g["coluna_d"] - pi.furo_passagem("M3")) / 2 >= 1.6, f"{(g['coluna_d'] - pi.furo_passagem('M3')) / 2:.1f} mm")
    presa_86 = LC["gaiola"]["tampa_e"] + (BE["gaiola"]["z0"] - PIS["z1"]) + (PIS["z1"] - PIS["z0"]) + LC["copo"]["braco_e"]
    presa_52 = presa_86 - LC["copo"]["braco_e"]
    porca = pi.PORCA_ISO4032["M3"]["m_max"]
    conferir("M3×25 + porca: a ponta passa a porca nas 4 colunas", presa_86 + porca <= 25.0 and presa_52 + porca <= 25.0,
             f"presa {presa_52:.1f} / {presa_86:.1f} (+ porca {porca}) ≤ 25")
    exigir_tudo_ok(ok)

    print("\n7. Massa, exportação e render")
    for nome, s in pecas:
        m, _, _ = propriedades_solido(s, PLA["densidade_g_cm3"])
        print(f"  {nome}: {m * 1000:.1f} g (PLA maciço)")
    saida = os.path.join(PROJ, "cad_export")
    for nome, s in (("volante", va), ("copo_motor_mais_y", ca_), ("copo_motor_menos_y", cb_), ("gaiola", gai), ("funil", fun),
                    ("cremalheira", crem), ("pinhao_sg90", pin), ("guia_cremalheira", gui), ("berco_piso", piso)):
        exportar_stl_mm(cq.Compound.makeCompound([s]), saida, f"{nome}.stl")
        cq.exporters.export(cq.Workplane().add(s), os.path.join(saida, f"{nome}.step"))
    cor = {"volante": (0.95, 0.45, 0.15), "copo": (0.55, 0.55, 0.60), "gaiola": (0.45, 0.65, 0.85), "funil": (0.80, 0.80, 0.55),
           "crem": (0.85, 0.25, 0.25), "pin": (0.95, 0.75, 0.10), "guia": (0.50, 0.50, 0.50), "piso": (0.95, 0.80, 0.55),
           "dardo": (0.15, 0.55, 0.95), "metal": (0.70, 0.72, 0.75), "sg": (0.15, 0.30, 0.75)}
    itens = [(piso, cor["piso"]), (va, cor["volante"]), (vb, cor["volante"]), (ca_, cor["copo"]), (cb_, cor["copo"]),
             (gai, cor["gaiola"]), (fun, cor["funil"]), (crem, cor["crem"]), (pin, cor["pin"]), (gui, cor["guia"]),
             (mot[1], cor["metal"]), (mot[-1], cor["metal"]), (sg, cor["sg"]), (d0, cor["dardo"])] + [(p, cor["dardo"]) for p in pilha]
    itens_usb = [(t, (0.5, 0.5, 0.55)) for t in cx.tubo(usb, P["caixa"]["fios"]["usb_d"])]
    prev = os.path.join(PROJ, "previews")
    render(itens + itens_usb, os.path.join(prev, "lancador.png"), foco=(5, 0, 5), posicao=(170, -230, 170), opacidade={5: 0.35, 6: 0.55})
    render(itens + itens_usb, os.path.join(prev, "lancador_por_baixo.png"), foco=(10, 0, -12), posicao=(150, -170, -150))
    render([(x, c) for x, c in itens if x is not gai], os.path.join(prev, "lancador_sem_gaiola_por_cima.png"),
           foco=(5, 0, 0), posicao=(0, -1, 260), zoom=1.2, opacidade={5: 0.5})
    print(f"  renders em {prev}/")


if __name__ == "__main__":
    main()
