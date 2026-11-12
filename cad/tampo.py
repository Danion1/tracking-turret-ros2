#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tampo.py — peça 2a da parte B: o tampo da base, com o cubo fixo do 6808 e o
encaixe do servo do pan (STS3215).

Por que o servo fica no TAMPO: o entre-eixos de 45 mm (pinhão × coroa) é a
cota mais sensível da torreta. A folga do par zera se os eixos chegarem
0,41 mm mais perto. Se o cubo (eixo do pan) e o servo (eixo do pinhão)
ficarem na mesma peça impressa, essa distância depende de um desenho só, e
não da soma de duas peças aparafusadas.

Como o servo fica preso: ele sobe por baixo num rebaixo com o contorno da
caixa dele (2,5 mm de encaixe, folga de 0,15 por lado), que posiciona em x e
y. A face de cima, na parte elevada de trás, encosta no tampo e dá a altura.
Dois parafusos B-PA2.0 descem pelo tampo nos furos de trás da tampa de cima
do servo. Os furos da frente não servem: a cabeça bateria no cubo do pinhão.

Lê SÓ plano/parametros.json. As formas da tampa do servo vêm do STEP do
SO-ARM100 (medidas no componentes.json, com a fonte); o mesmo STEP entra
aqui só para CONFERIR a interferência.

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/tampo.py [--rapido]
"""

import json
import math
import os
import sys

import cadquery as cq

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import usar_biblioteca  # noqa: E402,F401
import pan  # noqa: E402  (peças da transmissão, para conferir a montagem)
import parafusos_iso as pi  # noqa: E402
from check_bbox import check_bbox, check_volume, exigir_tudo_ok  # noqa: E402
from dados_materiais import PLA  # noqa: E402
from exportar_stl_mm import exportar_stl_mm  # noqa: E402
from formas_basicas import box_minmax, cylinder_z_at  # noqa: E402
from massa_inercia import propriedades_solido  # noqa: E402
from render_cad import render  # noqa: E402

PROJ = os.path.dirname(AQUI)
P = json.load(open(os.path.join(PROJ, "plano", "parametros.json"), encoding="utf-8"))
B, SV, CU, ROL = P["base"], P["servo_pan"], P["pan"]["cubo"], P["pecas"]["r6808"]
TP = P["pecas"]["sts3215"]["topo"]
INS = P["pecas"]["inserto_m3"]
STEP = os.path.join(PROJ, "fontes", "STS3215_03a_SO-ARM100.step")
RAPIDO = "--rapido" in sys.argv
Z = cq.Vector(0, 0, 1)
ok = True


def conferir(nome, cond, detalhe=""):
    global ok
    print(f"  [{'OK' if cond else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok &= bool(cond)
    return cond


def cil(r, z0, z1, x=0.0, y=0.0):
    return cylinder_z_at(x, y, z0, 2 * r, z1 - z0).val()


# ---------------------------------------------------------------- frame do servo
# STEP: eixo de saída em (12,5; 0), z ao longo do eixo, traseira para −x.
# Mundo: o eixo vai para o eixo do pinhão; a traseira (−x do STEP) aponta para u.
ANG = math.degrees(math.atan2(SV["u"][1], SV["u"][0])) - 180.0
XE = SV["x_eixo_step"]


def para_mundo(s):
    """Sólido em coordenadas do STEP -> mundo."""
    return (s.translate(cq.Vector(-XE, 0, SV["dz"]))
             .rotate(cq.Vector(), Z, ANG)
             .translate(cq.Vector(SV["eixo"][0], SV["eixo"][1], 0)))


def z_step(z_mundo):
    return z_mundo - SV["dz"]


def caixa_step(x0, x1, y0, y1, z0, z1):
    return box_minmax(x0, x1, y0, y1, z0, z1).val()


# ---------------------------------------------------------------- tampo
def fazer_tampo():
    w, t, f = B["w"], B["tampo"], SV["folga"]
    placa = (cq.Workplane("XY").workplane(offset=-t).rect(w, w).extrude(t)
             .edges("|Z").fillet(B["r"]).val())
    # cubo: ressalto (até d2) + assento Ø40 com chanfro de entrada; sobreposição de 0,3 na placa
    ressalto = cil(CU["r"], -0.3, CU["z1"])
    assento = (cq.Workplane("XY").workplane(offset=CU["z1"] - 0.3).circle(CU["assento_r"])
               .extrude(CU["assento_z1"] - CU["z1"] + 0.3).faces(">Z").edges().chamfer(CU["chanfro"]).val())
    s = placa.fuse(ressalto).fuse(assento)

    # encaixe do servo, desenhado no frame do STEP e levado ao mundo (um corte por vez)
    cx, cy = 22.7, 12.4                       # caixa do STEP (±), contorno do corpo
    tr, nv = TP["tras_elevada"], TP["nervura_central"]
    fe, fg = SV["folga_estatica"], SV["folga_topo"]
    zb = z_step(-t - 1)
    ferr_step = [
        # 1. contorno do corpo: posiciona em x e y; teto 0,3 acima da face da frente
        caixa_step(-cx - f, cx + f, -cy - f, cy + f, zb, TP["frente_topo_z"] + fg),
        # 2. parte elevada de trás: teto EXATO na face dela (é o apoio que dá a altura)
        caixa_step(tr["x"][0] - f, tr["x"][1] + f, tr["y"][0] - f, tr["y"][1] + f, zb, tr["topo_z"]),
        # 3. nervura central: parada, folga estática
        caixa_step(nv["x"][0] - f, nv["x"][1] + f, nv["y"][0] - f, nv["y"][1] + f, zb, nv["topo_z"] + fe),
        # 4. ressalto Ø9 da estria e 5. horn (gira): furo passante acima da base do horn − folga
        cylinder_z_at(XE, 0, zb, TP["ressalto_eixo_d"] + 2 * fg, z_step(1) - zb).val(),
        cylinder_z_at(XE, 0, TP["horn_z"][0] - fg, 2 * (P["pecas"]["sts3215"]["horn_r"] + 0.5),
                      z_step(1) - (TP["horn_z"][0] - fg)).val(),
    ]
    d_m2 = pi.furo_passagem("M2")
    for y in (-10.25, 10.25):                 # furos de trás da tampa de cima (STEP, componentes.json)
        ferr_step.append(cylinder_z_at(-16.5, y, tr["topo_z"] - 0.5, d_m2, z_step(1) - tr["topo_z"] + 0.5).val())
    ferramentas = [para_mundo(x) for x in ferr_step]
    # furo dos cabos e os 4 M3 dos cantos (passagem + rebaixo, cabeça rente ao topo)
    ferramentas.append(cil(CU["furo_r"], -t - 1, CU["assento_z1"] + 1))
    m3 = pi.PARAFUSO_ISO4762[B["rosca_cantos"]]
    presa = t - m3["k_max"]
    c = w / 2 - B["cantos"]
    for sx in (-1, 1):
        for sy in (-1, 1):
            ferramentas += [cil(pi.furo_passagem("M3") / 2, -t - 1, 1, sx * c, sy * c),
                            cil(pi.REBAIXO_DIAMETRO["M3"] / 2, -t + presa, 1, sx * c, sy * c)]
    for fe_ in ferramentas:
        s = s.cut(fe_)
    L, eng = pi.parafuso_em_rosca_cega("M3", presa, prof_max_rosca=INS["L"], folga_fundo=0.5, engajamento_min=4.0)
    info = {"m3": f"4 × M3×{L} ISO 4762 (presa {presa:.1f}, {eng:.1f} mm no inserto da caixa)",
            "pele_nervura": -(nv["topo_z"] + fe + SV["dz"]),
            "pele_tras": -(tr["topo_z"] + SV["dz"]),
            "encaixe": (TP["frente_topo_z"] + fg + SV["dz"]) + t}
    return s, info


def main():
    print("1. Peça")
    tampo, info = fazer_tampo()
    n = len(tampo.Solids())
    conferir("tampo: 1 sólido válido, volume > 0", n == 1 and tampo.isValid() and tampo.Volume() > 0,
             f"{n} sólido(s), {tampo.Volume():.0f} mm³")
    print(f"  parafusos dos cantos: {info['m3']}")

    print("\n2. Caixa e volume contra contas à mão")
    ok_ = check_bbox("tampo", tampo, (-70, 70, -70, 70, -6.0, 8.5), tol=0.06)
    # à mão: placa 140²·6 − cantos (4−π)·7²·6 = 117 348; ressalto π·21²·2 = 2 771;
    # assento π·20²·6,5 = 8 168; − furo Ø20 × 14,5 = 4 555; − contorno 45,7 × 25,1 × 2,4 = 2 753;
    # − trás 30,2 × 25,1 × 1,3 = 985; − nervura 30,5 × 14,3 × 1,3 = 567; − horn π·10,5²·2,3 = 797
    # (menos o que já saiu ≈ 0); − cantos 4 × (π·1,7²·6 + π·3,25²·3) = 616  ->  ≈ 118 014
    ok_ &= check_volume("tampo", tampo, 118014, tol=0.03 * 118014)
    conferir("caixa e volume", ok_)

    print("\n3. Servo no lugar (STEP do SO-ARM100)")
    servo = para_mundo(cq.importers.importStep(STEP).val())
    fatia_horn = servo.intersect(cil(30, -0.5, 0.0, *SV["eixo"]))
    ch = fatia_horn.Center()
    d = math.hypot(ch.x, ch.y)
    conferir("eixo do servo a 45,000 mm do eixo do pan (centro do horn medido no STEP)", abs(d - 45.0) < 0.02,
             f"{d:.3f} mm")
    conferir("topo do horn em z 0,5 (fundo do cubo do pinhão)", abs(servo.BoundingBox().zmax - 2.0) < 0.05,
             f"cabeça do parafuso central até z {servo.BoundingBox().zmax:.2f}")
    v = servo.intersect(tampo).Volume()
    conferir("servo × tampo: sem interferência (só encosta)", v < 0.5, f"{v:.3f} mm³")
    dist = servo.distance(tampo)
    conferir("servo encosta no tampo (apoio da parte de trás)", dist < 0.01, f"distância {dist:.3f} mm")
    sb = servo.BoundingBox()
    lim = B["w"] / 2 - B["parede"]
    conferir("servo dentro das paredes da caixa", max(abs(sb.xmin), abs(sb.xmax), abs(sb.ymin), abs(sb.ymax)) < lim - 1,
             f"x {sb.xmin:.1f}..{sb.xmax:.1f}, y {sb.ymin:.1f}..{sb.ymax:.1f} (paredes a ±{lim})")
    c = B["w"] / 2 - B["cantos"]
    d_cantos = min(servo.distance(cil(5.0, -40, -6, sx * c, sy * c)) for sx in (-1, 1) for sy in (-1, 1))
    conferir("servo longe das colunas dos cantos (Ø10 suposto)", d_cantos > 2.0, f"{d_cantos:.1f} mm")
    conferir("servo longe do furo dos cabos", servo.distance(cil(CU["furo_r"], -40, 0)) > 2.0,
             f"{servo.distance(cil(CU['furo_r'], -40, 0)):.1f} mm")

    print("\n4. Montagem com a transmissão e o rolamento (pan de −90° a +90°)")
    pinhao, _ = pan.fazer_pinhao()
    coroa, _ = pan.fazer_coroa()
    rol = pan.anel(ROL["d"] / 2, ROL["D"] / 2, P["pan"]["rolamento"]["z0"], P["pan"]["rolamento"]["z1"])
    v = rol.intersect(tampo).Volume()
    conferir("rolamento no cubo: assento Ø40 nominal, ressalto só no anel interno", v < 0.5, f"{v:.3f} mm³")
    conferir("ressalto até d2 (não toca a vedação)", CU["r"] <= ROL["d2"] / 2, f"r {CU['r']} ≤ {ROL['d2'] / 2}")
    pior = 0.0
    for a in range(-90, 91, 30 if RAPIDO else 10):
        cc, pp = pan.posar(coroa, pinhao, a)
        pior = max(pior, cc.intersect(tampo).Volume(), pp.intersect(tampo).Volume(), pp.intersect(servo).Volume())
    conferir("coroa × tampo, pinhão × tampo, pinhão × servo: zero", pior < 1e-3, f"máx {pior:.5f} mm³")

    print("\n5. Fabricação")
    bb = tampo.BoundingBox()
    conferir("cabe na Inventor (230 × 150 × 160)", bb.xlen <= 230 and bb.ylen <= 150 and bb.zlen <= 160,
             f"{bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f}")
    conferir("pele sobre a nervura do servo ≥ 1,0 (5 camadas)", info["pele_nervura"] >= 1.0 - 1e-9, f"{info['pele_nervura']:.2f} mm")
    conferir("pele sob os parafusos do servo ≥ 1,6", info["pele_tras"] >= 1.6, f"{info['pele_tras']:.2f} mm")
    conferir("encaixe do contorno do servo ≥ 2 mm", info["encaixe"] >= 2.0, f"{info['encaixe']:.2f} mm")
    exigir_tudo_ok(ok)

    print("\n6. Massa, exportação e render")
    m, _, _ = propriedades_solido(tampo, PLA["densidade_g_cm3"])
    print(f"  tampo: {m * 1000:.0f} g (PLA maciço)")
    saida = os.path.join(PROJ, "cad_export")
    os.makedirs(saida, exist_ok=True)
    exportar_stl_mm(cq.Compound.makeCompound([tampo]), saida, "tampo.stl")
    cq.exporters.export(cq.Workplane().add(tampo), os.path.join(saida, "tampo.step"))
    prev = os.path.join(PROJ, "previews")
    cc, pp = pan.posar(coroa, pinhao, 0)
    cor = {"tampo": (0.85, 0.85, 0.80), "servo": (0.15, 0.15, 0.17), "coroa": (0.20, 0.45, 0.80),
           "pinhao": (0.95, 0.55, 0.15), "aco": (0.70, 0.72, 0.75)}
    render([(tampo, cor["tampo"]), (servo, cor["servo"]), (cc, cor["coroa"]), (pp, cor["pinhao"]), (rol, cor["aco"])],
           os.path.join(prev, "tampo_montagem.png"), foco=(15, 15, -5), posicao=(170, -160, 120))
    render([(tampo, cor["tampo"]), (servo, cor["servo"])],
           os.path.join(prev, "tampo_por_baixo_com_servo.png"), foco=(15, 15, -10), posicao=(140, -150, -140),
           opacidade={1: 0.45})
    render([(tampo, cor["tampo"])],
           os.path.join(prev, "tampo_por_baixo.png"), foco=(15, 15, -6), posicao=(110, -130, -150))
    # corte pelo eixo do pinhão, ao longo do servo: mostra o encaixe, o apoio e o horn
    meio = para_mundo(caixa_step(-300, 300, 5.0, 300, -300, 300))   # tira a metade y_step > 5 (passa nos parafusos)
    cortes = [(tampo.cut(meio), (0.45, 0.62, 0.45)), (servo.cut(meio), cor["servo"]), (pp.cut(meio), cor["pinhao"]),
              (cc.cut(meio), cor["coroa"]), (rol.cut(meio), cor["aco"])]
    e = SV["eixo"]
    a_ = math.radians(ANG)
    n_ = (-math.sin(a_), math.cos(a_))                               # +y do STEP no mundo: o lado que saiu
    render(cortes, os.path.join(prev, "tampo_corte.png"), foco=(e[0] + 18 * SV["u"][0], e[1] + 18 * SV["u"][1], -8),
           posicao=(e[0] + 18 * SV["u"][0] + 140 * n_[0], e[1] + 18 * SV["u"][1] + 140 * n_[1], 12), zoom=1.7)
    print(f"  renders em {prev}/")


if __name__ == "__main__":
    main()
