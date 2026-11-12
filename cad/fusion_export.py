#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fusion_export.py — monta a pasta para abrir a torreta no Fusion 360 (padrão do braço:
uma pasta por conjunto com as peças e a montagem dele, e a montagem completa com a
árvore de componentes nomeada).

Saída (gerada, fora do Git): cad_export/torreta_fusion/
  LEIA-ME.md                    o que abrir primeiro e como fazer os mecanismos girarem
  torreta_completa.step         a montagem, árvore: base, pan, tilt, lancador, webcam, compradas, fios
  <grupo>/<peça>.step|.stl      cada peça impressa com nome claro
  <grupo>/<grupo>_montagem.step o conjunto sozinho
  script_fusion/torreta_juntas/ o script da API do Fusion (cópia de cad/fusion/torreta_juntas/)

Nada é redesenhado: as peças vêm de montagem.montar() (os scripts aprovados), na pose zero.
O script do Fusion acha cada peça pelo NOME do componente, então os nomes daqui e de lá
têm de bater: a tabela NOMES é a única fonte deles.

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/fusion_export.py
"""

import json
import math
import os
import shutil
import sys

import cadquery as cq

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import usar_biblioteca  # noqa: E402,F401
import berco as bc  # noqa: E402
import caixa as cxm  # noqa: E402
import mesa_bracos as mb  # noqa: E402
import montagem as mo  # noqa: E402
import webcam as wc  # noqa: E402
from exportar_stl_mm import exportar_stl_mm  # noqa: E402

PROJ = os.path.dirname(AQUI)
P = json.load(open(os.path.join(PROJ, "plano", "parametros.json"), encoding="utf-8"))
SAIDA = os.path.join(PROJ, "cad_export", "torreta_fusion")

# nome no montagem.py -> (nome do componente no Fusion, pasta/grupo da árvore, grupo de MOVIMENTO)
NOMES = {
    "caixa": ("caixa", "base", "base"), "tampo": ("tampo", "base", "base"),
    "pinhão do pan": ("pinhao_pan_20T", "pan", "pinhao_pan"), "coroa": ("coroa_pan_40T", "pan", "pan"),
    "mesa + coluna": ("mesa_coluna", "pan", "pan"), "braço +y": ("braco_mais_y", "pan", "pan"),
    "braço −y": ("braco_menos_y", "pan", "pan"),
    "piso do berço": ("berco_piso", "tilt", "tilt"), "face +y": ("berco_face_mais_y", "tilt", "tilt"),
    "face −y": ("berco_face_menos_y", "tilt", "tilt"), "arruela": ("arruela_m8", "tilt", "tilt"),
    "volante +y": ("volante_mais_y", "lancador", "volante_mais_y"), "volante −y": ("volante_menos_y", "lancador", "volante_menos_y"),
    "copo +y": ("copo_motor_mais_y", "lancador", "tilt"), "copo −y": ("copo_motor_menos_y", "lancador", "tilt"),
    "gaiola": ("gaiola", "lancador", "tilt"), "funil": ("funil", "lancador", "tilt"), "guia": ("guia_cremalheira", "lancador", "tilt"),
    "cremalheira": ("cremalheira_m1", "lancador", "cremalheira"), "pinhão do SG90": ("pinhao_sg90_20T", "lancador", "pinhao_sg90"),
    "suporte da webcam": ("suporte_webcam", "webcam", "tilt"),
    # compradas (referência)
    "servo do pan": ("servo_pan_STS3215", "compradas", "base"), "servo do tilt": ("servo_tilt_STS3215", "compradas", "pan"),
    "6808": ("rolamento_6808", "compradas", "base"), "608": ("rolamento_608", "compradas", "pan"),
    "motor 130 +y": ("motor_130_mais_y", "compradas", "tilt"), "motor 130 −y": ("motor_130_menos_y", "compradas", "tilt"),
    "SG90": ("servo_SG90", "compradas", "tilt"), "porca": ("porca_M8_travante", "compradas", "tilt"),
    "cabeça M8": ("parafuso_M8x20", "compradas", "tilt"), "corpo M8": ("parafuso_M8x20", "compradas", "tilt"),
    "webcam: perna": ("webcam_perna", "compradas", "tilt"), "webcam: aba": ("webcam_aba", "compradas", "tilt"),
    "webcam: dobradiça": ("webcam_dobradica", "compradas", "tilt"), "webcam: corpo": ("webcam_corpo", "compradas", "tilt"),
    "webcam: lente": ("webcam_lente", "compradas", "tilt"),
    "webcam: cabeça 1/4\"": ("parafuso_1_4_20x5_16", "compradas", "tilt"), "webcam: haste 1/4\"": ("parafuso_1_4_20x5_16", "compradas", "tilt"),
    "dardo na câmara": ("dardo_camara", "compradas", "cremalheira"),
}
for k in range(1, 4):
    NOMES[f"dardo do funil {k}"] = (f"dardo_funil_{k}", "compradas", "tilt")
ELETRONICA = {"buck 5": "buck_5V_SG90", "buck 3": "buck_3V_motores", "buck 7": "buck_7V_servos", "Waveshare": "waveshare_bus_servo",
              "placa perfurada": "placa_perfurada", "ESP32": "esp32_s3_devkitc", "jack": "jack_dc_12V"}

# o script do Fusion lê isto: grupos de movimento e as juntas (mm e graus; z do eixo do tilt)
ZT = P["tilt_cad"]["z"]
EP = P["berco"]["empurrador"]


def juntas():
    vo, e = P["berco"]["volante"], P["pan"]["pinhao"]["eixo"]
    xs, ys = EP["x_pinhao"], EP["y_primitiva"] + EP["rp"]
    return [
        {"nome": "pan", "tipo": "revolucao", "eixo": "z", "ponto": [0, 0, 0], "fixo": "tampo", "movel": "coroa_pan_40T",
         "limites": [-90, 90]},
        {"nome": "pinhao_pan", "tipo": "revolucao", "eixo": "z", "ponto": [e[0], e[1], 0], "fixo": "tampo", "movel": "pinhao_pan_20T"},
        # no Fusion, girar + em torno de +y ABAIXA o nariz: o tilt do projeto (+ levanta) é o oposto
        {"nome": "tilt", "tipo": "revolucao", "eixo": "y", "ponto": [0, 0, ZT], "fixo": "braco_mais_y", "movel": "berco_piso",
         "limites": [-P["limites"]["tilt"][1], -P["limites"]["tilt"][0]]},
        {"nome": "volante_mais_y", "tipo": "revolucao", "eixo": "z", "ponto": [vo["x"], vo["y"], ZT], "fixo": "berco_piso", "movel": "volante_mais_y"},
        {"nome": "volante_menos_y", "tipo": "revolucao", "eixo": "z", "ponto": [vo["x"], -vo["y"], ZT], "fixo": "berco_piso", "movel": "volante_menos_y"},
        {"nome": "pinhao_sg90", "tipo": "revolucao", "eixo": "z", "ponto": [xs, ys, ZT], "fixo": "berco_piso", "movel": "pinhao_sg90_20T",
         "limites": [0, math.degrees(EP["curso"] / EP["rp"])]},
        {"nome": "cremalheira", "tipo": "deslizamento", "eixo": "x", "ponto": [P["lancador_cad"]["cremalheira"]["x1"], 0, ZT],
         "fixo": "berco_piso", "movel": "cremalheira_m1", "limites": [0, EP["curso"]]},
    ]


def vinculos():
    """Motion Links: (junta A, valor A, junta B, valor B, invertido). O pinhão gira −2× o pan;
    os volantes giram em sentidos opostos; o pinhão do SG90 anda π·d na cremalheira por volta."""
    passo_volta = math.pi * EP["m"] * EP["z"]
    return [("pinhao_pan", "360 deg", "pan", "180 deg", True),
            ("volante_mais_y", "360 deg", "volante_menos_y", "360 deg", True),
            ("pinhao_sg90", "360 deg", "cremalheira", f"{passo_volta:.3f} mm", False)]


def fios_por_grupo():
    """Os tubos dos fios, nos mesmos percursos conferidos nas peças 2 a 6."""
    u, e = P["servo_pan"]["u"], P["servo_pan"]["eixo"]
    dtras = 22.7 + P["servo_pan"]["x_eixo_step"] + 1.0 + P["caixa"]["fios"]["servo_d"] / 2
    st = (e[0] + dtras * u[0], e[1] + dtras * u[1], -26.0)
    out = {"fios_base": [], "fios_pan": [], "fios_tilt": []}
    for _, _, d, pts in cxm.fios(st):
        out["fios_base"] += cxm.tubo(pts, d)
    for _, _, d, pts in mb.fios():
        out["fios_pan"] += cxm.tubo(pts, d)
    lc, bcd = P["lancador_cad"], P["berco_cad"]
    du = P["caixa"]["fios"]["usb_d"]
    tilt = []
    for _, _, d, pts in bc.fios():
        tilt += cxm.tubo(pts, d)
    uf, up = bcd["piso_furos"]["usb_webcam"], lc["usb_ponta"]
    tilt += cxm.tubo([(uf["x"], uf["y"], P["berco"]["piso"]["z1"] + 6.0), (uf["x"], uf["y"], up[2]), (up[0], uf["y"], up[2])], du)
    _, zc0 = wc.webcam_ref()
    zu = zc0 + P["pecas"]["webcam"]["alt"] / 2
    xa = wc.WX - P["pecas"]["webcam"]["prof"] / 2 - 1.0 - du / 2
    tilt += cxm.tubo([tuple(up), (up[0], up[1], zu), (up[0], 0.0, zu), (xa, 0.0, zu)], du)
    out["fios_tilt"] = [bc.mundo(t, 0.0) for t in tilt]                 # frame do berço -> mundo (tilt 0)
    return {k: cq.Compound.makeCompound(v) for k, v in out.items()}


def main():
    os.makedirs(SAIDA, exist_ok=True)
    pecas = mo.montar()
    pose = {p.nome: mo.posar(p, 0, 0, 0) for p in pecas}
    comp, cores, grupo_arvore, grupo_mov, impressa = {}, {}, {}, {}, {}
    for p in pecas:
        if p.nome in NOMES:
            nome, arv, mov = NOMES[p.nome]
        else:
            chave = next((k for k in ELETRONICA if p.nome.startswith(k)), None)
            if chave is None:
                raise SystemExit(f"peça sem nome no Fusion: {p.nome}")
            nome, arv, mov = ELETRONICA[chave], "compradas", "base"
        comp.setdefault(nome, []).append(pose[p.nome])
        cores[nome], grupo_arvore[nome], grupo_mov[nome] = p.cor, arv, mov
        impressa[nome] = p.impressa
    for nome, s in fios_por_grupo().items():
        comp[nome], cores[nome], grupo_arvore[nome] = [s], (0.4, 0.4, 0.45), "fios"
        grupo_mov[nome] = {"fios_base": "base", "fios_pan": "pan", "fios_tilt": "tilt"}[nome]
        impressa[nome] = False
    solido = {n: (v[0] if len(v) == 1 else cq.Compound.makeCompound(v)) for n, v in comp.items()}

    arvore = ["base", "pan", "tilt", "lancador", "webcam", "compradas", "fios"]
    raiz = cq.Assembly(name="torreta")
    for g in arvore:
        sub = cq.Assembly(name=g)
        gsub = cq.Assembly(name=f"{g}_montagem")
        for n in sorted(n for n in solido if grupo_arvore[n] == g):
            cor = cq.Color(*cores[n])
            sub.add(solido[n], name=n, color=cor)
            gsub.add(solido[n], name=n, color=cor)
            if impressa[n]:
                os.makedirs(os.path.join(SAIDA, g), exist_ok=True)
                cq.exporters.export(cq.Workplane().add(solido[n]), os.path.join(SAIDA, g, f"{n}.step"))
                exportar_stl_mm(cq.Compound.makeCompound([solido[n]]), os.path.join(SAIDA, g), f"{n}.stl")
        os.makedirs(os.path.join(SAIDA, g), exist_ok=True)
        gsub.export(os.path.join(SAIDA, g, f"{g}_montagem.step"))
        raiz.add(sub, name=g)
    raiz.export(os.path.join(SAIDA, "torreta_completa.step"))

    # o MESMO conjunto organizado por MOVIMENTO: um componente de topo por conjunto que se move
    # junto. As juntas do Fusion ligam só esses componentes de topo, sem grupo rígido (a 1ª versão
    # do script usava grupos rígidos entre peças de subconjuntos diferentes, e o Fusion recusou:
    # "A joint in system exists ... over constrained", 2026-11-06).
    mov = cq.Assembly(name="torreta_mecanismos")
    for g in sorted(set(grupo_mov.values())):
        sub = cq.Assembly(name=f"mov_{g}")
        for n in sorted(n for n in solido if grupo_mov[n] == g):
            sub.add(solido[n], name=n, color=cq.Color(*cores[n]))
        mov.add(sub, name=f"mov_{g}")
    mov.export(os.path.join(SAIDA, "torreta_mecanismos.step"))

    # o script do Fusion: copiado para a pasta, com os dados (nomes, grupos, juntas, vínculos) num JSON ao lado
    src = os.path.join(AQUI, "fusion", "torreta_juntas")
    dst = os.path.join(SAIDA, "script_fusion", "torreta_juntas")
    if os.path.isdir(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst, ignore=shutil.ignore_patterns("__pycache__"))
    dados = {"grupos_movimento": {}, "juntas": [], "vinculos": vinculos(), "chao": "mov_base"}
    for n, g in grupo_mov.items():
        dados["grupos_movimento"].setdefault(g, []).append(n)
    for j in juntas():                                  # as juntas ligam os componentes de topo (mov_<grupo>)
        j = dict(j, fixo="mov_" + grupo_mov[j["fixo"]], movel="mov_" + grupo_mov[j["movel"]])
        dados["juntas"].append(j)
    with open(os.path.join(dst, "torreta_juntas.json"), "w", encoding="utf-8") as f:
        json.dump(dados, f, ensure_ascii=False, indent=1)
    shutil.copy(os.path.join(AQUI, "fusion", "LEIA-ME.md"), os.path.join(SAIDA, "LEIA-ME.md"))

    n_imp = sum(1 for n in solido if impressa[n])
    print(f"  {len(solido)} componentes ({n_imp} impressos) em {len(arvore)} grupos; {len(dados['juntas'])} juntas, "
          f"{len(dados['vinculos'])} vínculos")
    for g, membros in dados["grupos_movimento"].items():
        print(f"    movimento '{g}': {len(membros)} componentes")
    print(f"  pasta: {SAIDA}")


if __name__ == "__main__":
    main()
