#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
webcam.py — peça 6 da parte B: o suporte da webcam, opção A (Daniel, 2026-11-05).

O clipe da webcam dobra 90°: a perna deita para trás com a rosca 1/4"-20 para
baixo, e a aba de trás fica de pé. O suporte (frame do berço):
- placa do tamanho da tampa da gaiola, presa pelos MESMOS 4 M3 das colunas.
  A tampa tem 3 mm e não leva inserto, e porca sob ela bateria nos volantes;
- ressalto de 6,5 com uma borda de 1,5 que contorna a perna (folga 0,3):
  a perna não gira;
- rebaixo por baixo para a cabeça abaulada do 1/4"-20, que se aperta antes
  de montar o suporte na gaiola;
- entalhe por onde passa a USB que sobe do piso.
A webcam (corpo, perna, aba, dobradiça) entra como referência, com medidas
ASSUMIDAS pela foto: medir a webcam do Daniel.

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/webcam.py [--rapido]
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
import lancador as la  # noqa: E402
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
BE, BC, LC, WC, K = P["berco"], P["berco_cad"], P["lancador_cad"], P["webcam_cad"], P["pecas"]
G, W = BE["gaiola"], K["webcam"]
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


# geometria comum (mesmas contas da figura aprovada)
PE, AB, DB = WC["perna"], WC["aba"], WC["dobradica"]
WX = BE["webcam"]["x"]
Z_TAMPA = G["z0"] + LC["gaiola"]["tampa_e"]                    # topo da tampa da gaiola
Z_PLACA = Z_TAMPA + WC["placa_e"]
Z_FUNDO = Z_TAMPA + WC["base"]                                 # fundo do rebaixo (a perna apoia aqui)
PX0, PX1 = WX - PE["h"] + PE["t"] / 2, WX + PE["t"] / 2        # perna deitada
F, AR = WC["folga_rebaixo"], WC["borda"]
RX = (PX0 + PX1) / 2                                           # rosca no meio da perna (ASSUMIDO)


def fazer_suporte():
    gy = LC["gaiola"]["tampa_y"]
    placa = (cq.Workplane("XY").workplane(offset=Z_TAMPA).center((G["x0"] + G["x1"]) / 2, 0)
             .rect(G["x1"] - G["x0"], 2 * gy).extrude(WC["placa_e"]).edges("|Z").fillet(3.0).val())
    bx0, bx1, by = PX0 - F - AR, PX1 + F + AR, PE["w"] / 2 + F + AR
    ressalto = caixa(bx0, bx1, -by, by, Z_PLACA - 0.3, Z_FUNDO + WC["borda_h"])
    s = placa.fuse(ressalto)
    ferr = [caixa(PX0 - F, PX1 + F, -PE["w"] / 2 - F, PE["w"] / 2 + F, Z_FUNDO, Z_FUNDO + WC["borda_h"] + 1)]   # contorno da perna
    p14 = WC["parafuso_14"]
    ferr.append(cil(p14["furo"], Z_TAMPA - 1, Z_FUNDO + 1, RX, 0))
    ferr.append(cil(p14["cabeca_d"] + 2 * p14["folga_cabeca"], Z_TAMPA - 1, Z_TAMPA + p14["cabeca_h"] + 0.25, RX, 0))
    e = WC["entalhe_usb"]
    ferr.append(caixa(e["x"][0], e["x"][1], e["y"][0], e["y"][1], Z_TAMPA - 1, Z_PLACA + 1))
    for x, y in BC["piso_furos"]["gaiola"]["pontos"]:
        ferr.append(cil(pi.furo_passagem("M3"), Z_TAMPA - 1, Z_PLACA + 1, x, y))
    for f in ferr:
        s = s.cut(f)
    return s, {"sob_cabeca": Z_FUNDO - (Z_TAMPA + p14["cabeca_h"] + 0.25)}


def webcam_ref():
    """Perna, aba, dobradiça, corpo e lente (ASSUMIDOS pela foto), e o parafuso 1/4"-20."""
    perna = caixa(PX0, PX1, -PE["w"] / 2, PE["w"] / 2, Z_FUNDO, Z_FUNDO + PE["t"])
    aba = caixa(PX0 - AB["t"], PX0, -AB["w"] / 2, AB["w"] / 2, Z_FUNDO + WC["borda_h"], Z_FUNDO + WC["borda_h"] + AB["l"])
    dob = caixa(WX - DB["t"] / 2, WX + DB["t"] / 2, -DB["w"] / 2, DB["w"] / 2, Z_FUNDO + PE["t"], Z_FUNDO + PE["t"] + DB["h"])
    zc0 = Z_FUNDO + PE["t"] + DB["h"]
    corpo = caixa(WX - W["prof"] / 2, WX + W["prof"] / 2, -W["larg"] / 2, W["larg"] / 2, zc0, zc0 + W["alt"])
    lente = cq.Workplane("YZ").workplane(offset=WX + W["prof"] / 2).center(0, zc0 + W["alt"] / 2).circle(W["lente_r"]).extrude(1.6).val()
    p14 = WC["parafuso_14"]
    cab = cil(p14["cabeca_d"], Z_TAMPA + 0.25, Z_TAMPA + 0.25 + p14["cabeca_h"], RX, 0)
    haste = cil(6.35, Z_TAMPA + 0.25 + p14["cabeca_h"], Z_TAMPA + 0.25 + p14["cabeca_h"] + p14["L"], RX, 0)
    return {"perna": perna, "aba": aba, "dobradiça": dob, "corpo": corpo, "lente": lente,
            "cabeça 1/4\"": cab, "haste 1/4\"": haste}, zc0


def main():
    print("1. Peça")
    sup, inf = fazer_suporte()
    n = len(sup.Solids())
    conferir("suporte: 1 sólido válido, volume > 0", n == 1 and sup.isValid() and sup.Volume() > 0, f"{n} sólido(s), {sup.Volume():.0f} mm³")

    print("\n2. Caixa e volume contra contas à mão")
    ok_ = check_bbox("suporte", sup, (G["x0"], G["x1"], -46, 46, 9.5, 17.5), tol=0.06)
    # placa 41,6 × 92 × 3 = 11 482 − 4 furos 109 − entalhe 13,6 × 8 × 3 = 326 − cantos 23
    # ressalto 35,6 × 35,6 × 5 = 6 337 − contorno 30,6 × 30,6 × 1,5 = 1 405 − furo 1/4" π·3,3²·5 = 171
    # − rebaixo da cabeça (na placa) π·6,05²·3 = 345  ->  ≈ 15 440
    ok_ &= check_volume("suporte", sup, 15440, tol=0.04 * 15440)
    conferir("caixa e volume", ok_)

    print("\n3. Webcam no suporte (frame do berço, tilt 0)")
    ref, zc0 = webcam_ref()
    for nome, s in ref.items():
        if nome.startswith("haste"):
            continue
        conferir(f"{nome} × suporte: zero", s.intersect(sup).Volume() < 1e-3, f"{s.intersect(sup).Volume():.4f} mm³")
    conferir("perna apoiada no fundo do rebaixo, com 0,3 de folga no contorno",
             ref["perna"].distance(sup) < 0.01, f"distância {ref['perna'].distance(sup):.3f}")
    eng = WC["parafuso_14"]["L"] - (Z_FUNDO - (Z_TAMPA + 0.25 + WC["parafuso_14"]["cabeca_h"]))
    conferir("1/4\"-20 × 5/16\": 4 a 6 mm na rosca da perna (profundidade ASSUMIDA ~6: medir)", 4.0 <= eng <= 6.0, f"{eng:.1f} mm de rosca")
    gai = la.fazer_gaiola()
    fun = la.fazer_funil()
    va, vb = la.fazer_volante(BE["volante"]["y"]), la.fazer_volante(-BE["volante"]["y"])
    conferir("suporte assenta na tampa da gaiola", sup.intersect(gai).Volume() < 1e-3 and sup.distance(gai) < 0.01)
    pior, quem = 0.0, ""
    for nome, s in list(ref.items()) + [("suporte", sup)]:
        for no, o in (("funil", fun), ("volante +y", va), ("volante −y", vb), ("gaiola", gai)):
            if nome == "suporte" and no == "gaiola":
                continue
            v = s.intersect(o).Volume()
            if v > pior:
                pior, quem = v, f"{nome} × {no}"
    conferir("webcam e suporte × funil, volantes e gaiola: zero", pior < 1e-3, f"máx {pior:.4f} mm³ {quem}")
    fr_f = BE["calha"]["x1"] + LC["funil"]["esp"]
    conferir("aba do clipe passa à frente do funil", PX0 - AB["t"] - fr_f >= 0.4, f"{PX0 - AB['t'] - fr_f:.2f} mm")
    lente_z = zc0 + W["alt"] / 2
    boca_topo = BE["piso"]["z1"] + K["dardo"]["d"] + 2
    ang = math.degrees(math.atan2(lente_z - boca_topo, BE["boca"]["x1"] - (WX + W["prof"] / 2)))
    conferir("a boca não entra no campo da lente (meia-abertura vertical ~22° para 78° na diagonal 16:9)", ang > 25,
             f"a boca fica {ang:.0f}° abaixo do eixo da lente")

    print("\n4. Giro do tilt e fios")
    servo = mb.servo_para_mundo(cq.importers.importStep(tp.STEP).val())
    bm, _ = mb.fazer_braco_mais()
    bn, _ = mb.fazer_braco_menos()
    mesa = mb.fazer_mesa()
    tmin, tmax = P["limites"]["tilt"]
    pior, quem, zmax = 0.0, "", -1e9
    for t in [tmin + i * (tmax - tmin) / (4 if RAPIDO else 12) for i in range((4 if RAPIDO else 12) + 1)]:
        for nb, sb in list(ref.items()) + [("suporte", sup)]:
            wb = bc.mundo(sb, t)
            for nf, sf in (("braço +y", bm), ("braço −y", bn), ("mesa", mesa), ("servo do tilt", servo)):
                v = wb.intersect(sf).Volume()
                if v > pior:
                    pior, quem = v, f"{nb} × {nf} a {t:.0f}°"
            zmax = max(zmax, wb.BoundingBox().zmax)
    conferir("webcam e suporte × braços, mesa e servo: zero em toda a faixa", pior < 1e-3, f"máx {pior:.4f} mm³ {quem}")
    print(f"  ponto mais alto da torreta (webcam no tilt máximo): z {zmax:.0f} mm acima do tampo")
    up = LC["usb_ponta"]
    du = P["caixa"]["fios"]["usb_d"]
    xa = WX - W["prof"] / 2 - 1.0 - du / 2
    zu = zc0 + W["alt"] / 2
    usb = [tuple(up), (up[0], up[1], zu), (up[0], 0.0, zu), (xa, 0.0, zu)]
    obst = list(ref.items()) + [("suporte", sup), ("gaiola", gai), ("funil", fun)]
    v, quem = 0.0, ""
    for t_ in cx.tubo(usb, du):
        for no, o in obst:
            vv = t_.intersect(o).Volume()
            if vv > v:
                v, quem = vv, no
    conferir("USB: do topo da gaiola até a traseira da webcam sem tocar nada (passa no entalhe)", v < 1e-3, f"{v:.4f} mm³ {quem}")
    pl = cx.tubo([(LC["usb_ponta"][0] - 20, up[1], up[2]), tuple(up)], du)       # trecho da peça 5 sobre a placa
    conferir("o trecho da USB da peça 5 passa no entalhe da placa", all(t_.intersect(sup).Volume() < 1e-3 for t_ in pl))

    print("\n5. Fabricação e fixação")
    bb = sup.BoundingBox()
    dims = sorted([bb.xlen, bb.ylen, bb.zlen], reverse=True)
    conferir("suporte cabe na Inventor", dims[0] <= 230 and dims[1] <= 150 and dims[2] <= 160, f"{bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f}")
    conferir("plástico sob a cabeça do 1/4\" ≥ 2", inf["sob_cabeca"] >= 2.0, f"{inf['sob_cabeca']:.2f} mm")
    porca = pi.PORCA_ISO4032["M3"]["m_max"]
    presa_86 = WC["placa_e"] + LC["gaiola"]["tampa_e"] + (G["z0"] - BE["piso"]["z1"]) + (BE["piso"]["z1"] - BE["piso"]["z0"]) + LC["copo"]["braco_e"]
    presa_52 = presa_86 - LC["copo"]["braco_e"]
    conferir("M3 das colunas: ×25 nas de trás, ×30 nas da frente (com o copo do motor)", presa_52 + porca <= 25 and presa_86 + porca <= 30 and presa_86 + porca > 25,
             f"presa {presa_52:.1f} + porca {porca} ≤ 25; {presa_86:.1f} + {porca} ≤ 30")
    exigir_tudo_ok(ok)

    print("\n6. Massa, exportação e render")
    m, _, _ = propriedades_solido(sup, PLA["densidade_g_cm3"])
    print(f"  suporte: {m * 1000:.1f} g (PLA maciço)")
    saida = os.path.join(PROJ, "cad_export")
    exportar_stl_mm(cq.Compound.makeCompound([sup]), saida, "suporte_webcam.stl")
    cq.exporters.export(cq.Workplane().add(sup), os.path.join(saida, "suporte_webcam.step"))
    cores = {"perna": (0.15, 0.15, 0.17), "aba": (0.15, 0.15, 0.17), "dobradiça": (0.3, 0.3, 0.32), "corpo": (0.12, 0.12, 0.13),
             "lente": (0.6, 0.65, 0.75), "cabeça 1/4\"": (0.7, 0.72, 0.75), "haste 1/4\"": (0.7, 0.72, 0.75)}
    piso = bc.fazer_piso()
    base_itens = [(piso, (0.95, 0.80, 0.55)), (gai, (0.45, 0.65, 0.85)), (fun, (0.80, 0.80, 0.55)), (va, (0.95, 0.45, 0.15)),
                  (vb, (0.95, 0.45, 0.15)), (sup, (0.55, 0.80, 0.55))]
    itens = base_itens + [(s, cores[n]) for n, s in ref.items()] + [(t_, (0.5, 0.5, 0.55)) for t_ in cx.tubo(usb, du)]
    prev = os.path.join(PROJ, "previews")
    render(itens, os.path.join(prev, "webcam.png"), foco=(60, 0, 25), posicao=(230, -200, 150))
    render([(sup, (0.55, 0.80, 0.55))], os.path.join(prev, "webcam_suporte.png"), foco=(69, 0, 13), posicao=(140, -110, 90))
    render([(sup, (0.55, 0.80, 0.55))], os.path.join(prev, "webcam_suporte_por_baixo.png"), foco=(69, 0, 11), posicao=(140, -110, -70))
    print(f"  renders em {prev}/")


if __name__ == "__main__":
    main()
