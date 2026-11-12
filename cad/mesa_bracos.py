#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mesa_bracos.py — peça 3 da parte B: a mesa giratória com a coluna (uma peça,
opção A do Daniel) e os dois braços do tilt, com os fios que sobem.

- Mesa + coluna: a coluna (Ø36, oca Ø20) desce da mesa e assenta no topo da
  coroa; 4 × M3×30 descem pela parede da coluna até os insertos da coroa.
  Imprime com o topo da mesa na cama e a coluna crescendo (sem suporte).
  Duas fendas no alto da coluna (±y) soltam os cabos sob a mesa.
- Braço +y: posiciona e prende o STS3215 do tilt pela face de BAIXO dele,
  num rebaixo do contorno (como o tampo faz com o do pan), com janela para
  os 2 conectores (os plugues entram por fora) e furo para o horn passivo.
  O horn de cima do servo aparafusa na face +y do berço (peça 4).
- Braço −y: alojamento do 608 com anel de encosto no anel EXTERNO (furo
  Ø19,6, entre o D2 19,2 da vedação e o Da máx. 20 da SKF).
- Fios: o barramento sobe pela coluna, sai pela fenda +y, corre sob a mesa,
  sobe pelo furo além do braço +y e entra no conector do servo. O feixe do
  berço (motores, SG90, USB da webcam) sai pela fenda −y e sobe pelo vão
  entre o braço −y e a face do berço, até z 100 (a peça 4 continua).

Lê SÓ plano/parametros.json. O STEP do servo entra só para conferir.
Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/mesa_bracos.py [--rapido]
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
import pan  # noqa: E402
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
M, CO, CR = P["pan"]["mesa"], P["pan"]["coluna"], P["pan"]["coroa"]
TC, ST, INS = P["tilt_cad"], P["pecas"]["sts3215"], P["pecas"]["inserto_m3"]
TOPO, FUNDO = ST["topo"], ST["fundo"]
FI = P["caixa"]["fios"]
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


def anel(r0, r1, z0, z1):
    return cq.Workplane("XY").workplane(offset=z0).circle(r1).circle(r0).extrude(z1 - z0).val()


# ---------------------------------------------------------------- frame do servo do tilt
# STEP: eixo em (12,5; 0), z ao longo do eixo (horn em +z). Mundo: o eixo do servo é o
# eixo do tilt (x 0, z TC["z"]), apontando para −y (o horn encosta na face do berço em y 50);
# a traseira do servo (−x do STEP) aponta para baixo. ex→+z, ey→−x, ez→−y (rotação própria).
# = rotação de +90° em x (ey→ez, ez→−ey) seguida de −90° em y (ex→ez, ez→−ex).
XE = ST["eixo_dx"]


def servo_para_mundo(s):
    o = cq.Vector()
    s = (s.translate(cq.Vector(-XE, 0, -TOPO["horn_z"][1]))
          .rotate(o, cq.Vector(1, 0, 0), 90).rotate(o, cq.Vector(0, 1, 0), -90))
    return s.translate(cq.Vector(0, TC["y_horn"], TC["z"]))


def caixa_step(x0, x1, y0, y1, z0, z1):
    return servo_para_mundo(caixa(x0, x1, y0, y1, z0, z1))


def y_de_zstep(zs):
    return TC["y_horn"] + TOPO["horn_z"][1] - zs


# ---------------------------------------------------------------- peças
def perfil_braco(y0, y1):
    """Placa x ±20 do pé (z0) até o eixo, com topo em arco de raio 20 centrado no eixo."""
    x0, x1 = TC["x"]
    corpo = caixa(x0, x1, y0, y1, TC["z0"], TC["z"])
    arco = cylinder_y_at(0, (y0 + y1) / 2, TC["z"], 2 * TC["topo_r"], y1 - y0).val()
    return corpo.fuse(arco)


def pes_do_braco(s, y0, y1):
    """2 insertos M3 no pé (furo cego vertical), para os M3 que sobem da mesa."""
    yc = (y0 + y1) / 2
    for x in TC["parafusos_mesa"]["x"]:
        s = s.cut(cil(INS["furo_d"], TC["z0"] - 1, TC["z0"] + INS["L"] + INS["fundo_extra"], x, yc))
    return s


def fazer_braco_mais():
    y0, y1 = TC["mais"]["y"]
    s = perfil_braco(y0, y1)
    f = 0.15
    tr, cu, cn, hp = FUNDO["tras_fundo"], FUNDO["cunha"], FUNDO["conectores"], FUNDO["horn_passivo"]
    zin = TC["zstep_in"]
    zfora = zin - TC["esp"] - 1
    ferr = [
        caixa_step(-22.7 - f, 22.7 + f, -12.4 - f, 12.4 + f, FUNDO["frente_fundo_z"] - 0.3, zin + 1),   # contorno: posiciona
        caixa_step(tr["x"][0] - f, tr["x"][1] + f, tr["y"][0] - f, tr["y"][1] + f, tr["z"], zin + 1),     # trás: APOIO (exato)
        caixa_step(cu["x"][0] - f, cu["x"][1] + f, cu["y"][0] - f, cu["y"][1] + f, cu["z"] - 0.2, zin + 1),  # cunha
        caixa_step(TC["janela_conectores"]["x_step"][0], TC["janela_conectores"]["x_step"][1],
                   TC["janela_conectores"]["y_step"][0], TC["janela_conectores"]["y_step"][1], zfora, zin + 1),  # janela
        servo_para_mundo(cylinder_z_at(XE, 0, zfora, hp["d"] + 1.0, zin + 1 - zfora).val()),               # horn passivo (gira)
    ]
    for y in (-10.25, 10.25):           # 2 furos de trás da tampa de baixo (PA3.0): passagem Ø3,4
        ferr.append(servo_para_mundo(cylinder_z_at(-20.3, y, zfora, pi.furo_passagem("M3"), tr["z"] + 0.5 - zfora).val()))
    for t in ferr:
        s = s.cut(t)
    s = pes_do_braco(s, y0, y1)
    info = {"pele_cunha": (cu["z"] - 0.2) - (zin - TC["esp"]), "pele_parafusos": tr["z"] - (zin - TC["esp"])}
    return s, info


def fazer_braco_menos():
    y0, y1 = TC["menos"]["y"]
    s = perfil_braco(y0, y1)
    R = TC["r608"]
    # alojamento do 608 aberto na face de dentro (y1), anel de encosto na de fora
    s = s.cut(cylinder_y_at(0, y1 - R["B"] / 2 + 0.5, TC["z"], R["D"], R["B"] + 1.0).val())
    s = s.cut(cylinder_y_at(0, (y0 + y1) / 2, TC["z"], R["furo_encosto"], y1 - y0 + 2).val())
    s = pes_do_braco(s, y0, y1)
    return s, {"encosto": (y1 - y0) - R["B"]}


def fazer_mesa():
    w, d, r = M["w"], M["d"], M["r"]
    placa = (cq.Workplane("XY").workplane(offset=M["z0"]).center(M["x"] + w / 2, M["y"] + d / 2)
             .rect(w, d).extrude(M["z1"] - M["z0"]).edges("|Z").fillet(r).val())
    coluna = anel(CO["furo_r"], CO["r"], CO["z0"], M["z0"] + 0.3)
    s = placa.fuse(coluna)
    ferr = [cil(2 * M["furo_r"], M["z0"] - 1, M["z1"] + 1), cil(2 * CO["furo_r"], CO["z0"] - 1, M["z0"])]
    # 4 × M3×30 pela parede da coluna, alinhados com os insertos da coroa (r 14, a 45°)
    ins = CR["insertos"]
    reb = CO["parafusos"]["rebaixo"]
    for k in range(ins["n"]):
        a = math.radians(ins["ang0"] + k * 360 / ins["n"])
        x, y = ins["r"] * math.cos(a), ins["r"] * math.sin(a)
        ferr += [cil(pi.furo_passagem("M3"), CO["z0"] - 1, M["z1"] + 1, x, y),
                 cil(pi.REBAIXO_DIAMETRO["M3"], M["z1"] - reb, M["z1"] + 1, x, y)]
    # fendas dos cabos no alto da coluna (±y)
    fe = CO["fendas"]
    for sy in (-1, 1):
        y0, y1 = (0.0, CO["r"] + 1) if sy > 0 else (-CO["r"] - 1, 0.0)     # do eixo para fora: corta a parede inteira
        ferr.append(caixa(-fe["larg"] / 2, fe["larg"] / 2, y0, y1, M["z0"] - fe["alt"], M["z0"] + 0.01))
    # furos dos cabos na mesa
    for f in M["furos_cabos"].values():
        ferr.append(caixa(f["x"] - f["w"] / 2, f["x"] + f["w"] / 2, f["y"] - f["d"] / 2, f["y"] + f["d"] / 2,
                          M["z0"] - 1, M["z1"] + 1))
    # rebaixo de 1 mm que posiciona cada braço + furos dos M3 que sobem até os insertos dos pés
    for nome in ("mais", "menos"):
        y0, y1 = TC[nome]["y"]
        ferr.append(caixa(TC["x"][0] - 0.15, TC["x"][1] + 0.15, y0 - 0.15, y1 + 0.15, TC["z0"], M["z1"] + 1))
        for x in TC["parafusos_mesa"]["x"]:
            ferr.append(cil(pi.furo_passagem("M3"), M["z0"] - 1, M["z1"], x, (y0 + y1) / 2))
    for f in ferr:
        s = s.cut(f)
    return s


# ---------------------------------------------------------------- fios
def fios():
    zsob, topo = 33.5, P["pan"]["cubo"]["assento_z1"]
    L = []
    # barramento até o servo do tilt: plugue no conector (STEP x −1,55; y +5,2), cabo sai por fora do braço
    fm = M["furos_cabos"]["mais"]
    xb = -5.2
    yplug = y_de_zstep(FUNDO["conectores"]["z_pinos"] - 6.0)        # ponta de fora do plugue (~8 mm de corpo)
    zcon = TC["z"] + (-1.55 - XE)
    sx, sy = cx.SOBE["bus"]
    q = [(sx, sy, topo), (sx, sy, zsob), (sx, fm["y"], zsob), (xb, fm["y"], zsob), (xb, fm["y"], zcon),
         (xb, yplug + FI["servo_d"] / 2, zcon)]
    L.append(("barramento: sobe → servo do tilt (plugue por fora do braço +y)", "azul", FI["servo_d"], q))
    # feixe do berço: dentro da coluna cada cabo vai para uma fila (2 andares, para caber na fenda de 14),
    # sai pela fenda −y, desvia em x num y próprio (sem cruzar os outros) e sobe no vão entre o braço −y
    # e a face do berço, até z 100 (a peça 4 continua)
    fmn = M["furos_cabos"]["menos"]
    feixe = [  # código, nome, cor, d, x na fila, andar z, x no furo da mesa, y do desvio
        ("usb", "USB da webcam", "cinza", FI["usb_d"], -4.0, 33.5, -14.0, -22.0),
        ("m+", "motores + (3 V)", "amarelo", FI["potencia_d"], -0.3, 33.5, -10.0, -25.0),
        ("m-", "motores − (dreno)", "preto", FI["potencia_d"], 2.3, 33.5, -7.4, -28.0),
        ("sg1", "SG90 V+", "laranja", FI["sinal_d"], -2.0, 37.3, -4.5, -22.0),
        ("sg2", "SG90 GND", "preto", FI["sinal_d"], 0.0, 37.3, -2.9, -25.0),
        ("sg3", "SG90 sinal", "branco", FI["sinal_d"], 2.0, 37.3, -1.3, -28.0)]
    for k, (cod, nome, cor, d, xf, zn, xa, yd) in enumerate(feixe):
        x0, y0 = cx.SOBE[cod]
        zm = 19.0 + 1.6 * k                               # muda para a fila dentro da coluna
        q = [(x0, y0, topo), (x0, y0, zm), (xf, -2.0, zm), (xf, -2.0, zn), (xf, yd, zn), (xa, yd, zn),
             (xa, fmn["y"], zn), (xa, fmn["y"], 100.0)]
        L.append((f"{nome}: sobe → vão do braço −y (a peça 4 continua)", cor, d, q))
    return L


def main():
    print("1. Peças")
    mesa = fazer_mesa()
    bm, im = fazer_braco_mais()
    bn, inn = fazer_braco_menos()
    for nome, s in (("mesa + coluna", mesa), ("braço +y", bm), ("braço −y", bn)):
        n = len(s.Solids())
        conferir(f"{nome}: 1 sólido válido, volume > 0", n == 1 and s.isValid() and s.Volume() > 0,
                 f"{n} sólido(s), {s.Volume():.0f} mm³")

    print("\n2. Caixa e volume contra contas à mão")
    ok_ = check_bbox("mesa + coluna", mesa, (-40, 40, -65, 99.1, 17.5, 45), tol=0.06)
    ok_ &= check_bbox("braço +y", bm, (-20, 20, 81.1, 89.1, 44, 138), tol=0.06)
    ok_ &= check_bbox("braço −y", bn, (-20, 20, -65, -57, 44, 138), tol=0.06)
    # mesa: 80 × 164,1 × 6 − cantos (4−π)·8²·6 = 78 768 − 330 = 78 438; coluna π(18²−10²)·21,5 = 15 130;
    #  − furo Ø16 × 6 = 1 206; − 4 furos Ø3,4 × 27,5 + rebaixos π·3,25²·3 ≈ 4 × (250 + 100) = 1 400;
    #  − fendas 2 × 12 × 8 × 8 ≈ 1 540 (parede da coluna); − furos dos cabos 70×6 + 108×6 = 1 068;
    #  − rebaixos dos braços 2 × 40,3 × 8,3 × 1 = 669; − 4 furos Ø3,4 × 6 = 218  ->  ≈ 87 470
    ok_ &= check_volume("mesa + coluna", mesa, 87470, tol=0.04 * 87470)
    # braço: 40 × 74 × 8 + π·20²/2 × 8 = 23 680 + 5 027 = 28 707 (corpo até o eixo + meio disco);
    #  − 2 insertos π·2²·6,7 = 168  ->  braço −y − 608 π·11²·7 = 2 661 − encosto π·9,8²·1 = 302 ≈ 25 576
    #  braço +y − contorno 45,7 × 25,1 × 1,7 = 1 950 − trás 30,2 × 25,0 × 1,8 ≈ 1 359 − cunha ≈ 13,6 × 18,7 × 3,8 = 966
    #  − janela 7 × 21 × 6 ≈ 880 − horn passivo π·10,5²·~4 ≈ 1 385 − furos ≈ 60  ->  ≈ 21 940
    ok_ &= check_volume("braço −y", bn, 25576, tol=0.04 * 25576)
    ok_ &= check_volume("braço +y", bm, 21940, tol=0.06 * 21940)
    conferir("caixa e volume", ok_)

    print("\n3. Servo do tilt no braço +y (STEP)")
    servo = servo_para_mundo(cq.importers.importStep(tp.STEP).val())
    fatia = servo.intersect(caixa(-30, 30, TC["y_horn"] + 0.2, TC["y_horn"] + 0.6, 80, 160))
    c = fatia.Center()
    conferir("eixo do servo = eixo do tilt (centro do horn medido no STEP)", abs(c.x) < 0.02 and abs(c.z - TC["z"]) < 0.02,
             f"x {c.x:.3f}, z {c.z:.3f} (eixo em x 0, z {TC['z']})")
    a_dentro = servo.intersect(caixa(-30, 30, TC["y_horn"] + 0.05, TC["y_horn"] + 0.15, 80, 160)).Volume() / 0.1
    a_fora = servo.intersect(caixa(-30, 30, TC["y_horn"] - 0.15, TC["y_horn"] - 0.05, 80, 160)).Volume() / 0.1
    conferir("face de fora do horn em y 50 (fatias: horn inteiro dentro, só a cabeça do parafuso fora)",
             a_dentro > 250 and a_fora < 40, f"{a_dentro:.0f} mm² em y 50,1; {a_fora:.0f} mm² em y 49,9 (cabeça: bolsa no berço)")
    v = servo.intersect(bm).Volume()
    conferir("servo × braço +y: sem interferência", v < 0.5, f"{v:.3f} mm³")
    conferir("servo apoiado no braço (face de trás de baixo)", servo.distance(bm) < 0.01, f"{servo.distance(bm):.3f} mm")
    conferir("servo × mesa: zero", servo.intersect(mesa).Volume() < 1e-3)

    print("\n4. 608 no braço −y, coluna na coroa, giro do pan")
    R = TC["r608"]
    y1 = TC["menos"]["y"][1]
    rol = cylinder_y_at(0, y1 - R["B"] / 2, TC["z"], R["D"], R["B"]).val().cut(
        cylinder_y_at(0, y1 - R["B"] / 2, TC["z"], 8.0, R["B"] + 1).val())
    conferir("608 × braço −y: zero (alojamento Ø22 × 7)", rol.intersect(bn).Volume() < 1e-3, f"{rol.intersect(bn).Volume():.4f} mm³")
    conferir("608 encosta no anel de Ø19,6 (só o anel externo)", rol.distance(bn) < 0.01 and
             P["pecas"]["r608"]["encostos"]["D2_aprox"] < R["furo_encosto"] <= P["pecas"]["r608"]["encostos"]["Da_max"],
             f"D2 {P['pecas']['r608']['encostos']['D2_aprox']} < {R['furo_encosto']} ≤ Da máx {P['pecas']['r608']['encostos']['Da_max']}")
    pinhao, _ = pan.fazer_pinhao()
    coroa, _ = pan.fazer_coroa()
    conferir("coluna × coroa: zero (assenta no topo dela)", mesa.intersect(coroa).Volume() < 1e-3 and mesa.distance(coroa) < 0.01)
    reb = CO["parafusos"]["rebaixo"]
    presa = M["z1"] - reb - CR["z1"]
    eng = CO["parafusos"]["L"] - presa
    conferir("M3×30 da coluna: engajamento no inserto da coroa", 4.0 <= eng <= INS["L"] + INS["fundo_extra"] - 0.5,
             f"presa {presa:.1f}, {eng:.1f} mm no inserto ({pi.fios_engajados('M3', min(eng, INS['L'])):.0f} fios)")
    Lb, eb = pi.parafuso_em_rosca_cega("M3", M["z1"] - M["z0"], prof_max_rosca=INS["L"], folga_fundo=0.5, engajamento_min=4.0)
    print(f"  M3 dos pés dos braços: 4 × M3×{Lb} ISO 4762 de baixo da mesa ({eb:.1f} mm no inserto)")
    tampo_, _ = tp.fazer_tampo()
    gira = mesa.fuse(bm).fuse(bn)
    pior = 0.0
    for a in range(-90, 91, 45 if RAPIDO else 15):
        cc, pp = pan.posar(coroa, pinhao, a)
        g = gira.rotate(cq.Vector(), cq.Vector(0, 0, 1), a)
        sv = servo.rotate(cq.Vector(), cq.Vector(0, 0, 1), a)
        pior = max(pior, g.intersect(pp).Volume(), g.intersect(tampo_).Volume(), sv.intersect(pp).Volume(),
                   sv.intersect(tampo_).Volume())
    conferir("mesa, braços e servo × pinhão e tampo, de −90° a +90°: zero", pior < 1e-3, f"máx {pior:.4f} mm³")

    print("\n5. Fios que sobem")
    lista = fios()
    obst = [("mesa + coluna", mesa), ("braço +y", bm), ("braço −y", bn), ("servo do tilt", servo),
            ("coroa", pan.posar(coroa, pinhao, 0)[0]), ("tampo", tampo_)]
    batidas, pior = [], 0.0
    for nome, cor, d, pts in lista:
        for parte in cx.tubo(pts, d):
            for no, ob in obst:
                v = parte.intersect(ob).Volume()
                if v > 1e-3:
                    batidas.append(f"{nome} × {no}: {v:.2f} mm³")
                pior = max(pior, v)
    for b_ in batidas:
        print("      bate:", b_)
    conferir(f"{len(lista)} fios: nenhum atravessa mesa, coluna, braços, servo, coroa ou tampo", pior < 1e-3,
             f"máx {pior:.3f} mm³")

    print("\n6. Fabricação")
    for nome, s in (("mesa + coluna", mesa), ("braço +y", bm), ("braço −y", bn)):
        bb = s.BoundingBox()
        dims = sorted([bb.xlen, bb.ylen, bb.zlen], reverse=True)
        conferir(f"{nome} cabe na Inventor (230 × 150 × 160)", dims[0] <= 230 and dims[1] <= 150 and dims[2] <= 160,
                 f"{bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f}")
    paredes = {"furo M3 da coluna ↔ furo dos cabos": 14 - pi.furo_passagem("M3") / 2 - CO["furo_r"],
               "furo M3 da coluna ↔ face de fora": CO["r"] - 14 - pi.furo_passagem("M3") / 2,
               "rebaixo do M3 ↔ furo Ø16 da mesa": 14 - pi.REBAIXO_DIAMETRO["M3"] / 2 - M["furo_r"],
               "inserto do pé do braço": (TC["esp"] - INS["furo_d"]) / 2,
               "braço +y: pele sob a cunha do servo": im["pele_cunha"],
               "braço −y: anel de encosto do 608": inn["encosto"]}
    for k, v in paredes.items():
        conferir(f"parede {k} ≥ 1,0", v >= 1.0 - 1e-9, f"{v:.2f} mm")
    exigir_tudo_ok(ok)

    print("\n7. Massa, exportação e render")
    for nome, s in (("mesa + coluna", mesa), ("braço +y", bm), ("braço −y", bn)):
        m, _, _ = propriedades_solido(s, PLA["densidade_g_cm3"])
        print(f"  {nome}: {m * 1000:.0f} g (PLA maciço)")
    saida = os.path.join(PROJ, "cad_export")
    for nome, s in (("mesa_coluna", mesa), ("braco_mais_y", bm), ("braco_menos_y", bn)):
        exportar_stl_mm(cq.Compound.makeCompound([s]), saida, f"{nome}.stl")
        cq.exporters.export(cq.Workplane().add(s), os.path.join(saida, f"{nome}.step"))
    cores = {"vermelho": (0.85, 0.1, 0.1), "preto": (0.12, 0.12, 0.12), "laranja": (1.0, 0.55, 0.0),
             "amarelo": (0.95, 0.85, 0.1), "azul": (0.15, 0.35, 0.95), "verde": (0.1, 0.7, 0.2),
             "branco": (0.92, 0.92, 0.92), "cinza": (0.5, 0.5, 0.55)}
    cc, pp = pan.posar(coroa, pinhao, 0)
    itens = [(mesa, (0.60, 0.75, 0.90)), (bm, (0.55, 0.70, 0.55)), (bn, (0.55, 0.70, 0.55)), (servo, (0.2, 0.2, 0.22)),
             (rol, (0.7, 0.72, 0.75)), (cc, (0.20, 0.45, 0.80)), (pp, (0.95, 0.55, 0.15)), (tampo_, (0.85, 0.85, 0.80))]
    for nome, cor, d, pts in lista:
        itens += [(q, cores[cor]) for q in cx.tubo(pts, d)]
    prev = os.path.join(PROJ, "previews")
    render(itens, os.path.join(prev, "mesa_bracos.png"), foco=(0, 15, 70), posicao=(230, -150, 170))
    render(itens, os.path.join(prev, "mesa_bracos_lado_mais_y.png"), foco=(0, 60, 80), posicao=(120, 260, 130))
    render(itens, os.path.join(prev, "mesa_bracos_por_baixo.png"), foco=(0, 15, 35), posicao=(170, -120, -60),
           opacidade={7: 0.3})
    print(f"  renders em {prev}/")


if __name__ == "__main__":
    main()
