#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pan.py — peça 1 da parte B: a transmissão do pan (pinhão 20T + coroa 40T).

Lê SÓ plano/parametros.json (refazer com `python3 plano/gerar_parametros.py`).
Cota nova entra no gerador, nunca aqui. Nível do CAD: FABRICAÇÃO (PLA, Inventor).

Peças impressas que saem daqui:
  pinhao_pan  — 20T m1,5 com cubo Ø24 que aparafusa no horn do STS3215
                (4 × M3, rebaixo por cima, bolsa por baixo para o parafuso
                central do horn). Frame: eixo do pinhão na origem, z do mundo.
  coroa_pan   — 40T m1,5 com 6 garras que prendem o anel externo do 6808,
                anel de assento no anel externo e 4 insertos M3 no topo.
                Frame: eixo do pan (já é a origem do mundo).
Peças de referência (só para conferir e para o render, não exportam):
  rolamento 6808, cubo fixo, horn do servo, coluna.

Conferências (param antes de exportar): um sólido válido de volume
positivo, caixa e volume contra contas à mão, interferência zero do par em
uma volta de passo inteira (0,1°) e em ±90° de pan com os sólidos inteiros,
folga medida do par, teste negativo (entre-eixos menor TEM de cruzar),
paredes mínimas e cabe na Inventor.

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/pan.py [--rapido]
  --rapido: pula a varredura fina (passo de 1° em vez de 0,1°).
"""

import json
import math
import os
import sys

import cadquery as cq

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import usar_biblioteca  # noqa: E402,F401
import engrenagem as eg  # noqa: E402
import parafusos_iso as pi  # noqa: E402
from check_bbox import check_bbox, check_volume, exigir_tudo_ok  # noqa: E402
from dados_materiais import PLA  # noqa: E402
from exportar_stl_mm import exportar_stl_mm  # noqa: E402
from formas_basicas import cylinder_z_at  # noqa: E402
from massa_inercia import propriedades_solido  # noqa: E402
from render_cad import render  # noqa: E402

PROJ = os.path.dirname(AQUI)
P = json.load(open(os.path.join(PROJ, "plano", "parametros.json"), encoding="utf-8"))
PAN, K = P["pan"], P["pecas"]
CO, PN, GA, ROL = PAN["coroa"], PAN["pinhao"], PAN["garras"], PAN["rolamento"]
R68, ST, INS = K["r6808"], K["sts3215"], K["inserto_m3"]
RAPIDO = "--rapido" in sys.argv
Z = cq.Vector(0, 0, 1)
ok = True


def conferir(nome, cond, detalhe=""):
    global ok
    print(f"  [{'OK' if cond else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok &= bool(cond)
    return cond


def cilindro(r, z0, z1, x=0.0, y=0.0):
    """Atalho por raio e limites para o cylinder_z_at da Biblioteca (formas_basicas)."""
    return cylinder_z_at(x, y, z0, 2 * r, z1 - z0).val()


def anel(r0, r1, z0, z1):
    return cq.Workplane("XY").workplane(offset=z0).circle(r1).circle(r0).extrude(z1 - z0).val()


def perfil(g, z0, z1):
    """Engrenagem como prisma: contorno evolvente da Biblioteca, dente 0 em +x.
    A folga do JSON é a da espessura do dente; a função pede a de cada flanco."""
    pts = eg.perfil_pinhao(g["m"], g["z"], folga_flanco=g["folga"] / 2)
    return cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(z1 - z0).val()


def unico(nome, s):
    """Regra 4 do cad-python.md: um sólido, válido, volume > 0."""
    n = len(s.Solids())
    conferir(f"{nome}: 1 sólido válido, volume > 0", n == 1 and s.isValid() and s.Volume() > 0,
             f"{n} sólido(s), {s.Volume():.0f} mm³")


# ---------------------------------------------------------------- pinhão
def fazer_pinhao():
    z_h = PN["cubo_z0"]                                          # topo do horn
    s = cilindro(PN["cubo_r"], z_h, PN["z0"]).fuse(perfil(PN, PN["z0"], PN["z1"]))
    # parafusos do horn: comprimento e rebaixo pela regra da Biblioteca
    op = pi.rebaixo_para_rosca_cega(PN["rosca_horn"], PN["z1"] - z_h, prof_max_rosca=ST["horn_e"],
                                    folga_fundo=0.0, engajamento_min=PN["engajamento_min"],
                                    fundo_min=PN["fundo_rebaixo_min"])
    esc = min((o for o in op if o["saliencia_cabeca"] <= 0), key=lambda o: o["comprimento"])
    r_furos = ST["horn_furos_circulo"] / 2
    d_pass, d_reb = pi.furo_passagem(PN["rosca_horn"]), pi.REBAIXO_DIAMETRO[PN["rosca_horn"]]
    z_fundo = z_h + esc["presa"]
    ferramentas = [cilindro(PN["bolsa_d"] / 2, z_h - 1, z_h + PN["bolsa_prof"])]
    for k in range(ST["horn_furos_n"]):                           # furos a 45° (±4,95; ±4,95)
        a = math.radians(45 + 90 * k)
        x, y = r_furos * math.cos(a), r_furos * math.sin(a)
        ferramentas += [cilindro(d_pass / 2, z_h - 1, z_fundo + 0.01, x, y),
                        cilindro(d_reb / 2, z_fundo, PN["z1"] + 1, x, y)]
    for f in ferramentas:                                         # um corte por vez (regra 4)
        s = s.cut(f)
    info = {"parafuso": f"{PN['rosca_horn']}×{esc['comprimento']} ISO 4762", "presa": esc["presa"],
            "engajamento": esc["engajamento"], "z_fundo_rebaixo": z_fundo,
            "paredes": {"rebaixo ↔ borda do cubo": PN["cubo_r"] - (r_furos + d_reb / 2),
                        "furo ↔ bolsa central": r_furos - d_pass / 2 - PN["bolsa_d"] / 2,
                        "rebaixo ↔ pé do dente": PN["rf"] - (r_furos + d_reb / 2)}}
    return s, info


# ---------------------------------------------------------------- coroa
def garra(a_centro):
    """Setor de anel, face interna com interferência e chanfro de entrada embaixo."""
    ri, ro, ch = GA["r0"] - GA["interferencia"], GA["r1"], GA["chanfro"]
    z0, z1 = GA["z0"], GA["z1"] + 0.3                             # 0,3 dentro do corpo da coroa (regra 4)
    pts = [(ri, z0 + ch), (ri + ch, z0), (ro, z0), (ro, z1), (ri, z1)]
    w = GA["largura_ang"]
    g = cq.Workplane("XZ").polyline(pts).close().revolve(w, (0, 0, 0), (0, 1, 0)).val()
    return g.rotate(cq.Vector(), Z, a_centro - w / 2)


def fazer_coroa():
    s = perfil(CO, CO["z0"], CO["z1"])
    s = s.cut(cilindro(CO["furo_r"], CO["z0"] - 1, CO["z1"] + 1))
    a = CO["assento"]
    s = s.fuse(anel(a["r0"], a["r1"], a["z0"], a["z1"] + 0.3))
    for i in range(GA["n"]):
        s = s.fuse(garra(30 + i * 360 / GA["n"]))
    prof = INS["L"] + INS["fundo_extra"]
    for k in range(CO["insertos"]["n"]):
        an = math.radians(CO["insertos"]["ang0"] + k * 360 / CO["insertos"]["n"])
        r = CO["insertos"]["r"]
        s = s.cut(cilindro(INS["furo_d"] / 2, CO["z1"] - prof, CO["z1"] + 1, r * math.cos(an), r * math.sin(an)))
    info = {"paredes": {"garra (radial)": GA["r1"] - (GA["r0"] - GA["interferencia"]),
                        "inserto ↔ furo dos cabos": CO["insertos"]["r"] - INS["furo_d"] / 2 - CO["furo_r"],
                        "fundo do inserto": CO["z1"] - prof - CO["z0"]}}
    return s, info


# ---------------------------------------------------------------- referências
def referencias():
    rolamento = anel(R68["d"] / 2, R68["D"] / 2, ROL["z0"], ROL["z1"])
    cubo = anel(10.0, PAN["cubo"]["r"], -3.0, PAN["cubo"]["z1"]).fuse(anel(10.0, R68["d"] / 2, PAN["cubo"]["z1"], ROL["z1"] - 0.5))
    horn = cilindro(ST["horn_r"], PN["cubo_z0"] - ST["horn_e"], PN["cubo_z0"])
    c = PAN["coluna"]
    coluna = anel(CO["furo_r"], c["r"], c["z0"], c["z1"])
    return rolamento, cubo, horn, coluna


# ---------------------------------------------------------------- montagem
EIXO = PN["eixo"]
ANG_LINHA = math.degrees(math.atan2(EIXO[1], EIXO[0]))          # direção coroa → pinhão
RAZAO = PAN["razao"]


def posar(coroa, pinhao, pan_deg, dx=0.0, dpin=0.0):
    """Pan de `pan_deg`: coroa gira pan; pinhão gira −razão·pan (sentido oposto).
    Fase de montagem: dente da coroa na linha dos centros, vão do pinhão nela.
    dx aproxima o pinhão do centro (teste negativo); dpin gira só o pinhão."""
    c = coroa.rotate(cq.Vector(), Z, ANG_LINHA + pan_deg)
    fase_p = ANG_LINHA + 180 + 180 / PN["z"]
    u = (math.cos(math.radians(ANG_LINHA)), math.sin(math.radians(ANG_LINHA)))
    p = pinhao.rotate(cq.Vector(), Z, fase_p - RAZAO * pan_deg + dpin).translate(
        cq.Vector(EIXO[0] - dx * u[0], EIXO[1] - dx * u[1], 0))
    return c, p


def cruza(a, b):
    return a.intersect(b).Volume()


def main():
    print("1. Peças")
    pinhao, ip = fazer_pinhao()
    coroa, ic = fazer_coroa()
    unico("pinhão", pinhao)
    unico("coroa", coroa)
    print(f"  parafusos do horn: 4 × {ip['parafuso']}, presa {ip['presa']:.1f}, "
          f"engajamento {ip['engajamento']:.1f} mm ({pi.fios_engajados(PN['rosca_horn'], ip['engajamento']):.0f} fios)")

    print("\n2. Caixa e volume contra contas à mão (números literais)")
    # caixas: dente 0 em +x; com 20 e 40 dentes há dente em 0°, 90°, 180° e 270° -> ±ra
    ok_ = check_bbox("pinhão", pinhao, (-16.5, 16.5, -16.5, 16.5, 0.5, 17.5), tol=0.06)
    ok_ &= check_bbox("coroa", coroa, (-31.5, 31.5, -31.5, 31.5, 3.0, 17.5), tol=0.06)
    # volume (à mão, área do dentado ≈ π·rp²):
    #  pinhão 5655 (dentado) + 4072 (cubo) − 135 (bolsa) − 200 (furos) − 1526 (rebaixos) = 7866
    #  coroa  22619 − 2513 (furo Ø20) − 337 (insertos) + 56 (assento) + 493 (garras) = 20318
    ok_ &= check_volume("pinhão", pinhao, 7866, tol=0.03 * 7866)
    ok_ &= check_volume("coroa", coroa, 20318, tol=0.03 * 20318)
    conferir("caixa e volume", ok_)

    print("\n3. Engrenamento (perfis de 1 mm, uma volta de passo da coroa = 9°)")
    pp, pc = perfil(PN, 0, 1), perfil(CO, 0, 1)
    passo = 1.0 if RAPIDO else 0.1
    angs = [i * passo for i in range(int(round(9 / passo)) + 1)]
    vols = [cruza(*posar(pc, pp, a)) for a in angs]
    conferir(f"interferência zero em {len(angs)} posições", max(vols) < 1e-4, f"máx {max(vols):.6f} mm³")
    dist = [posar(pc, pp, a)[0].distance(posar(pc, pp, a)[1]) for a in angs[::max(1, len(angs) // 10)]]
    conferir("dentes perto (engrena de verdade)", max(dist) < 0.3, f"menor distância {min(dist):.3f} a {max(dist):.3f} mm")
    # folga do par: coroa parada, girar só o pinhão até encostar (busca binária, nos dois sentidos)

    def encosta(sinal):
        lo, hi = 0.0, 5.0
        for _ in range(22):
            mid = (lo + hi) / 2
            if cruza(*posar(pc, pp, 0.0, dpin=sinal * mid)) > 1e-5:
                hi = mid
            else:
                lo = mid
        return lo
    jogo = encosta(+1) + encosta(-1)
    folga_mm = math.radians(jogo) * PN["rp"]
    # à mão: cada engrenagem afina o dente 0,15 na primitiva -> par 0,30 mm
    conferir("folga do par na primitiva ≈ 0,30 mm", abs(folga_mm - 0.30) < 0.03,
             f"{folga_mm:.3f} mm = {jogo:.2f}° no pinhão = {jogo / RAZAO:.2f}° na torre")
    # negativo: aproximar Δa tira 2·Δa·tg20° da folga -> zera em Δa = 0,30/(2·0,364) = 0,41
    v_02 = cruza(*posar(pc, pp, 0.0, dx=0.2))
    v_06 = max(cruza(*posar(pc, pp, a, dx=0.6)) for a in (0, 2.25, 4.5, 6.75))
    conferir("aproximar 0,2 mm ainda não cruza (folga sobra 0,15)", v_02 < 1e-4)
    conferir("NEGATIVO: aproximar 0,6 mm cruza", v_06 > 1e-3, f"{v_06:.3f} mm³")

    print("\n4. Montagem inteira de −90° a +90° de pan (sólidos completos)")
    rol, cubo, horn, coluna = referencias()
    pior = 0.0
    for a in range(-90, 91, 15 if RAPIDO else 5):
        c, p = posar(coroa, pinhao, a)
        hp = posar(coroa, horn, a)[1]
        pior = max(pior, cruza(c, p), cruza(p, cubo), cruza(p, rol), cruza(c, cubo), cruza(c, hp))
    conferir("pinhão × coroa, pinhão × cubo/rolamento, coroa × cubo/horn: zero", pior < 1e-3, f"máx {pior:.5f} mm³")
    c0 = posar(coroa, pinhao, 0)[0]
    aperto = cruza(c0, rol)
    # à mão: 6 × (14/360)·π·(26² − 25,9²)·(6,5 − 0,5 do chanfro) ≈ 6 × 0,0389 × 16,3 × 6,0 ≈ 22,8 mm³ (+ assento 0)
    conferir("garras × rolamento = só o aperto planejado (0,1 mm)", 15 < aperto < 30, f"{aperto:.1f} mm³ (à mão ≈ 23)")
    folga_baixo = CO["z0"] - ROL["z1"]
    conferir("coroa livre do anel INTERNO (fixo)", folga_baixo >= 0.5 - 1e-9,
             f"{folga_baixo:.2f} mm; só o assento r {CO['assento']['r0']:.2f}..{CO['assento']['r1']:.2f} toca o anel externo")

    print("\n5. Fabricação (PLA na Inventor 230 × 150 × 160)")
    for nome, s, inf in (("pinhão", pinhao, ip), ("coroa", coroa, ic)):
        bb = s.BoundingBox()
        conferir(f"{nome} cabe na mesa", bb.xlen <= 230 and bb.ylen <= 150 and bb.zlen <= 160,
                 f"{bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f}")
        for k, v in inf["paredes"].items():
            if k.startswith("fundo"):   # fundo de furo cego: teto de ponte, sem carga -> 5 camadas de 0,2 (como o gerador)
                conferir(f"{nome}: {k} ≥ 1,0 (5 camadas)", v >= 1.0 - 1e-9, f"{v:.2f} mm")
            else:
                conferir(f"{nome}: parede {k} ≥ 1,6 (4 perímetros)", v >= 1.6 - 1e-9, f"{v:.2f} mm")
    # resistência do dente no TRAVAMENTO do servo (Lewis), informativa
    t_nmm = ST["torque_kgcm"] * 9.80665 * 10 * 7.0 / ST["tensao"]   # stall escalado para 7,0 V (ASSUMIDO linear)
    f, sig = eg.tensao_dente(t_nmm, PN["m"], PN["z"], CO["z1"] - CO["z0"], n_cremalheiras=1)
    print(f"  [info] stall a 7,0 V ≈ {t_nmm / 1000:.2f} N·m → F {f:.0f} N no primitivo, "
          f"σ pinhão {sig:.1f} MPa, FS {PLA['tensao_escoamento_mpa'] / sig:.2f} (PLA {PLA['tensao_escoamento_mpa']} MPa)")

    exigir_tudo_ok(ok)

    print("\n6. Massa, exportação e render")
    for nome, s in (("pinhão", pinhao), ("coroa", coroa)):
        m, _, _ = propriedades_solido(s, PLA["densidade_g_cm3"])
        print(f"  {nome}: {m * 1000:.1f} g (PLA maciço {PLA['densidade_g_cm3']} g/cm³; impresso com preenchimento sai menos)")
    saida = os.path.join(PROJ, "cad_export")
    os.makedirs(saida, exist_ok=True)
    for nome, s in (("pinhao_pan", pinhao), ("coroa_pan", coroa)):
        exportar_stl_mm(cq.Compound.makeCompound([s]), saida, f"{nome}.stl")
        cq.exporters.export(cq.Workplane().add(s), os.path.join(saida, f"{nome}.step"))
    print(f"  STL e STEP em {saida}/")

    prev = os.path.join(PROJ, "previews")
    os.makedirs(prev, exist_ok=True)
    c, p = posar(coroa, pinhao, 0)
    hp = posar(coroa, horn, 0)[1]
    cores = {"coroa": (0.20, 0.45, 0.80), "pinhao": (0.95, 0.55, 0.15), "aco": (0.70, 0.72, 0.75),
             "fixo": (0.55, 0.55, 0.55), "horn": (0.30, 0.30, 0.32), "coluna": (0.6, 0.75, 0.9)}
    render([(c, cores["coroa"]), (p, cores["pinhao"]), (rol, cores["aco"]), (cubo, cores["fixo"]),
            (hp, cores["horn"]), (coluna, cores["coluna"])],
           os.path.join(prev, "pan_montagem.png"), foco=(5, 15, 10), posicao=(150, -120, 110),
           opacidade={3: 0.35, 5: 0.25})
    # peças sozinhas, vistas de baixo: garras, assento, bolsa e furos
    render([(coroa, cores["coroa"]), (pinhao.translate(cq.Vector(55, 0, 0)), cores["pinhao"])],
           os.path.join(prev, "pan_pecas_por_baixo.png"), foco=(25, 0, 9), posicao=(95, -110, -90))
    render([(coroa, cores["coroa"]), (pinhao.translate(cq.Vector(55, 0, 0)), cores["pinhao"])],
           os.path.join(prev, "pan_pecas_por_cima.png"), foco=(25, 0, 9), posicao=(95, -110, 120))
    print(f"  renders em {prev}/")


if __name__ == "__main__":
    main()
