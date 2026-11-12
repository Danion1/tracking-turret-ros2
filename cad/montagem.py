#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
montagem.py — peça 7 da parte B: a torreta montada.

Junta as peças dos scripts aprovados (pan, tampo, caixa, mesa_bracos, berco,
lancador, webcam), sem redesenhar nada, em 4 grupos que se movem:
  base     fixa (caixa, tampo, servo do pan, eletrônica, fios da base)
  pinhão   gira em torno do próprio eixo: −2 × pan
  pan      gira pan em torno de z (coroa, mesa, braços, servo do tilt, 608, fios)
  tilt     frame do berço: inclina t e depois gira pan (berço, lançador, webcam)
  dentro do tilt, o empurrador anda o curso s (cremalheira, pinhão do SG90, dardo).

Confere:
  1. interferência ZERO entre grupos em pan × tilt × disparo combinados
     (as interferências planejadas ficam fora e estão listadas: garras × 6808 e
     volante × eixo do motor, que são prensagens);
  2. massas: impressas pelo volume do CAD × PLA maciço (limite de cima); compradas
     pelo datasheet;
  3. torque de gravidade no tilt em toda a faixa, contra o STS3215 a 7,0 V;
  4. inércias do tilt e do pan, e a aceleração que os servos dão;
e exporta a montagem (STEP e STL) na pose zero.

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/montagem.py [--rapido]
"""

import json
import math
import os
import sys
import time

import cadquery as cq

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import usar_biblioteca  # noqa: E402,F401
import berco as bc  # noqa: E402
import caixa as cxm  # noqa: E402
import lancador as la  # noqa: E402
import mesa_bracos as mb  # noqa: E402
import pan  # noqa: E402
import tampo as tp  # noqa: E402
import webcam as wc  # noqa: E402
from dados_materiais import PLA  # noqa: E402
from exportar_stl_mm import exportar_stl_mm  # noqa: E402
from formas_basicas import cylinder_y_at  # noqa: E402
from check_bbox import exigir_tudo_ok  # noqa: E402
from massa_inercia import propriedades_solido  # noqa: E402
from render_cad import render  # noqa: E402

PROJ = os.path.dirname(AQUI)
P = json.load(open(os.path.join(PROJ, "plano", "parametros.json"), encoding="utf-8"))
K, BE, TC, LC = P["pecas"], P["berco"], P["tilt_cad"], P["lancador_cad"]
RAPIDO = "--rapido" in sys.argv
G_ = 9.80665
ACO = 7.85                                                       # g/cm³ (aço do M8: ASSUMIDO, valor de livro)
ok = True


def conferir(nome, cond, detalhe=""):
    global ok
    print(f"  [{'OK' if cond else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok &= bool(cond)
    return cond


class Peca:
    def __init__(self, nome, solido, grupo, massa_g=None, impressa=False, cor=(0.7, 0.7, 0.7), conta=True):
        self.nome, self.s, self.grupo, self.impressa, self.cor, self.conta = nome, solido, grupo, impressa, cor, conta
        self.massa_g = solido.Volume() * PLA["densidade_g_cm3"] / 1000 if impressa else massa_g


def montar():
    """Todas as peças nos seus frames (base/pan/pinhão: mundo na pose zero; tilt/emp: frame do berço)."""
    L = []
    imp = dict(impressa=True)
    # --- base
    L += [Peca("caixa", cxm.fazer_caixa(), "base", cor=(0.85, 0.85, 0.80), **imp),
          Peca("tampo", tp.fazer_tampo()[0], "base", cor=(0.85, 0.85, 0.80), **imp),
          Peca("servo do pan", tp.para_mundo(cq.importers.importStep(tp.STEP).val()), "base", K["sts3215"]["massa_g"], cor=(0.2, 0.2, 0.22))]
    for c in P["caixa"]["comps"]:
        L.append(Peca(c["nome"], cxm.caixa(c["x"][0], c["x"][1], c["y"][0], c["y"][1], c["z"][0], c["z"][1]), "base", None,
                      cor=(0.6, 0.3, 0.2), conta=False))
    # --- pinhão e pan (pose zero)
    coroa, pinhao = pan.posar(pan.fazer_coroa()[0], pan.fazer_pinhao()[0], 0.0)
    L.append(Peca("pinhão do pan", pinhao, "pinhao", cor=(0.95, 0.55, 0.15), **imp))
    rol = P["pan"]["rolamento"]
    L += [Peca("coroa", coroa, "pan", cor=(0.20, 0.45, 0.80), **imp),
          Peca("6808", pan.anel(K["r6808"]["d"] / 2, K["r6808"]["D"] / 2, rol["z0"], rol["z1"]), "base", K["r6808"]["massa_g"], cor=(0.7, 0.72, 0.75)),
          Peca("mesa + coluna", mb.fazer_mesa(), "pan", cor=(0.60, 0.75, 0.90), **imp),
          Peca("braço +y", mb.fazer_braco_mais()[0], "pan", cor=(0.55, 0.70, 0.55), **imp),
          Peca("braço −y", mb.fazer_braco_menos()[0], "pan", cor=(0.55, 0.70, 0.55), **imp),
          Peca("servo do tilt", mb.servo_para_mundo(cq.importers.importStep(tp.STEP).val()), "pan", K["sts3215"]["massa_g"], cor=(0.2, 0.2, 0.22))]
    y1, R = TC["menos"]["y"][1], TC["r608"]
    anel608 = cylinder_y_at(0, y1 - R["B"] / 2, TC["z"], R["D"], R["B"]).val().cut(cylinder_y_at(0, y1 - R["B"] / 2, TC["z"], 8.0, R["B"] + 1).val())
    L.append(Peca("608", anel608, "pan", K["r608"]["massa_g"], cor=(0.7, 0.72, 0.75)))
    # --- tilt (frame do berço)
    fm, info = bc.fazer_face_menos()
    L += [Peca("piso do berço", bc.fazer_piso(), "tilt", cor=(0.95, 0.80, 0.55), **imp),
          Peca("face +y", bc.fazer_face_mais(), "tilt", cor=(0.90, 0.65, 0.40), **imp),
          Peca("face −y", fm, "tilt", cor=(0.90, 0.65, 0.40), **imp)]
    for nome, s in bc.pilha_m8(info)[0]:
        massa = s.Volume() * (PLA["densidade_g_cm3"] if nome == "arruela" else ACO) / 1000
        L.append(Peca(nome, s, "tilt", massa, impressa=(nome == "arruela"), cor=(0.45, 0.45, 0.5)))
    m = BE["motor"]
    L += [Peca("motor 130 +y", bc.motor_130(m["x"], m["y"]), "tilt", K["motor130"]["massa_g"], cor=(0.7, 0.72, 0.75)),
          Peca("motor 130 −y", bc.motor_130(m["x"], -m["y"]), "tilt", K["motor130"]["massa_g"], cor=(0.7, 0.72, 0.75)),
          Peca("SG90", bc.sg90(), "tilt", K["sg90"]["massa_g"], cor=(0.15, 0.30, 0.75)),
          Peca("volante +y", la.fazer_volante(BE["volante"]["y"]), "tilt", cor=(0.95, 0.45, 0.15), **imp),
          Peca("volante −y", la.fazer_volante(-BE["volante"]["y"]), "tilt", cor=(0.95, 0.45, 0.15), **imp),
          Peca("copo +y", la.fazer_copo(1), "tilt", cor=(0.55, 0.55, 0.60), **imp),
          Peca("copo −y", la.fazer_copo(-1), "tilt", cor=(0.55, 0.55, 0.60), **imp),
          Peca("gaiola", la.fazer_gaiola(), "tilt", cor=(0.45, 0.65, 0.85), **imp),
          Peca("funil", la.fazer_funil(), "tilt", cor=(0.80, 0.80, 0.55), **imp),
          Peca("guia", la.fazer_guia(), "tilt", cor=(0.5, 0.5, 0.5), **imp),
          Peca("suporte da webcam", wc.fazer_suporte()[0], "tilt", cor=(0.55, 0.80, 0.55), **imp)]
    ref, _ = wc.webcam_ref()
    for nome, s in ref.items():
        massa = K["webcam"]["massa_g"] if nome == "corpo" else (0.0 if nome != "haste 1/4\"" else 3.0)
        L.append(Peca(f"webcam: {nome}", s, "tilt", massa, cor=(0.12, 0.12, 0.13)))
    for k in range(1, BE["funil"]["n"] + 1):
        L.append(Peca(f"dardo do funil {k}", la.dardo(BE["calha"]["x0"] + 0.5, k * BE["funil"]["passo"]), "tilt", K["dardo"]["massa_g"],
                      cor=(0.15, 0.55, 0.95)))
    # --- empurrador (anda dentro do tilt)
    L += [Peca("cremalheira", la.fazer_cremalheira(), "emp", cor=(0.85, 0.25, 0.25), **imp),
          Peca("pinhão do SG90", la.fazer_pinhao_sg90(), "emp", cor=(0.95, 0.75, 0.10), **imp),
          Peca("dardo na câmara", la.dardo(BE["calha"]["x0"] + 0.5), "emp", K["dardo"]["massa_g"], cor=(0.15, 0.55, 0.95))]
    return L


# interferências PLANEJADAS (prensagens), fora da conferência
PRENSADAS = [{"coroa", "6808"}, {"volante +y", "motor 130 +y"}, {"volante −y", "motor 130 −y"}]


def posar(p, pan_deg, tilt_deg, curso):
    s = p.s
    if p.grupo == "pinhao":
        xc, yc = P["pan"]["pinhao"]["eixo"]
        return s.rotate(cq.Vector(xc, yc, 0), cq.Vector(xc, yc, 1), -P["pan"]["razao"] * pan_deg)
    if p.grupo == "emp":
        if p.nome == "cremalheira":
            s = la.posar_empurrador(s, s, curso)[0]
        elif p.nome == "pinhão do SG90":
            s = la.posar_empurrador(s, s, curso)[1]
        else:
            s = s.translate(cq.Vector(max(curso - 1.0, 0.0), 0, 0))     # o dardo anda depois de 1,0 de vão
    if p.grupo in ("tilt", "emp"):
        s = bc.mundo(s, tilt_deg)
    if p.grupo in ("pan", "tilt", "emp"):
        s = s.rotate(cq.Vector(), cq.Vector(0, 0, 1), pan_deg)
    return s


def caixas_cruzam(a, b, tol=0.01):
    A, B_ = a.BoundingBox(), b.BoundingBox()
    return not (A.xmax < B_.xmin - tol or B_.xmax < A.xmin - tol or A.ymax < B_.ymin - tol or B_.ymax < A.ymin - tol
                or A.zmax < B_.zmin - tol or B_.zmax < A.zmin - tol)


def pares_relevantes(pecas):
    """Pares que podem mudar de posição um em relação ao outro (grupos diferentes, ou empurrador × tilt)."""
    out = []
    for i, a in enumerate(pecas):
        for b in pecas[i + 1:]:
            if a.grupo == b.grupo and a.grupo != "emp":
                continue
            if {a.grupo, b.grupo} == {"emp"}:
                continue
            if {a.nome, b.nome} in PRENSADAS:
                continue
            out.append((a, b))
    return out


def main():
    t0 = time.time()
    print("1. Peças da montagem")
    pecas = montar()
    impressas = [p for p in pecas if p.impressa and p.conta and p.nome != "arruela"]
    print(f"  {len(pecas)} corpos: {len(impressas)} impressos, o resto comprado ou referência")

    print("\n2. Interferência em pan × tilt × disparo (zero entre o que se move)")
    pans = [-90, 0, 90] if RAPIDO else [-90, -45, 0, 45, 90]
    tilts = [P["limites"]["tilt"][0], P["limites"]["tilt"][1]] if RAPIDO else [-20, 0, 20, 40]
    cursos = [0.0, BE["empurrador"]["curso"]] if RAPIDO else [0.0, BE["empurrador"]["curso"] / 2, BE["empurrador"]["curso"]]
    pares = pares_relevantes(pecas)
    pior, quem, testados = 0.0, "", 0
    # Planejado (2026-11-10): no fim do curso a ponta do dardo entra 2 mm entre os volantes (pegada) e a espuma
    # comprime; esse par é medido à parte e tem de bater com o lancador.py (~2,8 mm³ por volante), não ser zero.
    PEGADA = ({"dardo na câmara", "volante +y"}, {"dardo na câmara", "volante −y"})
    pegada = 0.0
    for pa in pans:
        for ti in tilts:
            for cu in cursos:
                pose = {p.nome: posar(p, pa, ti, cu) for p in pecas}
                for a, b in pares:
                    sa, sb = pose[a.nome], pose[b.nome]
                    if not caixas_cruzam(sa, sb):
                        continue
                    testados += 1
                    v = sa.intersect(sb).Volume()
                    if {a.nome, b.nome} in PEGADA:
                        pegada = max(pegada, v)
                        continue
                    if v > pior:
                        pior, quem = v, f"{a.nome} × {b.nome} (pan {pa}°, tilt {ti}°, curso {cu:.1f})"
    n_poses = len(pans) * len(tilts) * len(cursos)
    conferir(f"{n_poses} poses × {len(pares)} pares ({testados} interseções com caixas sobrepostas): zero", pior < 1e-3,
             f"máx {pior:.4f} mm³ {quem}")
    conferir("pegada planejada: dardo × cada volante só no fim do curso, espuma comprimida ≤ 3,5 mm³", 0 < pegada <= 3.5,
             f"máx {pegada:.2f} mm³")
    pose0 = {p.nome: posar(p, 0, 0, 0) for p in pecas}
    for par in PRENSADAS:
        pa_, pb_ = [p for p in pecas if p.nome in par]
        v = pose0[pa_.nome].intersect(pose0[pb_.nome]).Volume()
        print(f"  [prensagem planejada] {pa_.nome} × {pb_.nome}: {v:.1f} mm³")

    print("\n3. Massas (impressas: volume do CAD × PLA maciço {:.2f} g/cm³, limite de cima)".format(PLA["densidade_g_cm3"]))
    grupos = {"base": 0.0, "pinhao": 0.0, "pan": 0.0, "tilt": 0.0, "emp": 0.0}
    m_imp = 0.0
    for p in pecas:
        if p.massa_g is None or not p.conta:
            continue
        grupos[p.grupo] += p.massa_g
        if p.impressa:
            m_imp += p.massa_g
    for p in sorted(impressas, key=lambda q: -q.massa_g):
        print(f"    {p.nome:<24} {p.massa_g:7.1f} g")
    print(f"  PLA impresso: {m_imp:.0f} g (maciço; com 3 paredes e 20 % de preenchimento sai algo como 50–60 %: ASSUMIDO)")
    print(f"  por grupo (impresso maciço + comprados; sem a eletrônica da base): "
          + ", ".join(f"{k} {v:.0f} g" for k, v in grupos.items()))

    print("\n4. Torque de gravidade no tilt (−20° a +40°)")
    tilt_pecas = [p for p in pecas if p.grupo in ("tilt", "emp") and p.massa_g]
    mx = sum(p.massa_g * p.s.Center().x for p in tilt_pecas)
    mz = sum(p.massa_g * p.s.Center().z for p in tilt_pecas)
    mt = sum(p.massa_g for p in tilt_pecas)
    print(f"  berço + lançador + webcam: {mt:.0f} g, centro de massa a x {mx / mt:.1f}, z {mz / mt:.1f} mm do eixo (frame do berço)")
    tmin, tmax = P["limites"]["tilt"]
    tau = []
    for i in range(int(tmax - tmin) + 1):
        t = math.radians(tmin + i)
        tau.append((tmin + i, G_ * (mx * math.cos(t) - mz * math.sin(t)) * 1e-6))    # N·m (g·mm → kg·m)
    tmax_ = max(tau, key=lambda x: abs(x[1]))
    st = K["sts3215"]
    nominal = st["torque_nominal_kgcm"] * 7.0 / st["tensao"] * G_ / 100        # escalado linear para 7,0 V (ASSUMIDO)
    stall = st["torque_kgcm"] * 7.0 / st["tensao"] * G_ / 100
    print(f"  maior torque: {abs(tmax_[1]):.3f} N·m a {tmax_[0]:.0f}° (plano estimava ~0,17)")
    conferir("STS3215 a 7,0 V segura o tilt com margem ≥ 2 sobre o NOMINAL", nominal / abs(tmax_[1]) >= 2.0,
             f"nominal {nominal:.3f} N·m → margem {nominal / abs(tmax_[1]):.1f}×; stall {stall:.2f} N·m → {stall / abs(tmax_[1]):.1f}×")
    sinal = "puxa o nariz para baixo (o servo segura para cima)" if tmax_[1] > 0 else "levanta o nariz"
    print(f"  sentido: com o centro de massa à frente do eixo, a gravidade {sinal}; o pior caso é o nariz mais baixo")

    print("\n5. Inércias e aceleração")
    def inercia_z_eixo(lista, eixo_fn):
        J = 0.0
        for p in lista:
            if not p.massa_g:
                continue
            s = eixo_fn(p)
            mkg = p.massa_g / 1000
            if p.impressa:
                _, c, T = propriedades_solido(s, PLA["densidade_g_cm3"])
                J_proprio = T
            else:
                c = tuple(v / 1000 for v in s.Center().toTuple())
                J_proprio = None
            yield p, mkg, c, J_proprio
    # tilt: em torno do eixo y (pelo ponto (0, 0) do berço)
    Jt = 0.0
    for p, mkg, c, T in inercia_z_eixo(tilt_pecas, lambda q: q.s):
        Jt += mkg * (c[0] ** 2 + c[2] ** 2) + (T[1][1] if T else 0.0)
    alfa_t = (nominal - abs(tmax_[1])) / Jt
    print(f"  tilt: J = {Jt * 1e4:.2f} kg·cm²; com o nominal menos a gravidade sobram {nominal - abs(tmax_[1]):.3f} N·m → {math.degrees(alfa_t):.0f} °/s²")
    # pan: em torno de z, tudo o que gira (pan + tilt na pose zero)
    gira = [p for p in pecas if p.grupo in ("pan", "tilt", "emp") and p.massa_g]
    Jp = 0.0
    for p, mkg, c, T in inercia_z_eixo(gira, lambda q: posar(q, 0, 0, 0)):
        Jp += mkg * (c[0] ** 2 + c[1] ** 2) + (T[2][2] if T else 0.0)
    tau_pan = P["pan"]["razao"] * stall / 3.0                     # limite de 1/3 do stall no firmware, ×2 da redução
    alfa_p = tau_pan / Jp
    print(f"  pan: J = {Jp * 1e4:.1f} kg·cm²; com o limite de 1/3 do stall (×2 da redução = {tau_pan:.2f} N·m na torre) → {math.degrees(alfa_p):.0f} °/s²")
    vmax = 60.0 / (st["s60"] * st["tensao"] / 7.0) / P["pan"]["razao"]
    print(f"  velocidade máxima do pan na torre ≈ {vmax:.0f} °/s (servo sem carga, escalado para 7,0 V)")
    conferir("o pan chega a 90 °/s em menos de 0,1 s (rastreio de um drone dentro do quarto: ASSUMIDO como alvo)",
             math.radians(90) / alfa_p < 0.1, f"{math.radians(90) / alfa_p * 1000:.0f} ms")
    exigir_tudo_ok(ok)

    print("\n6. Exportação e render")
    saida = os.path.join(PROJ, "cad_export")
    corpos = [pose0[p.nome] for p in pecas]
    cq.exporters.export(cq.Workplane().add(cq.Compound.makeCompound(corpos)), os.path.join(saida, "montagem.step"))
    exportar_stl_mm(cq.Compound.makeCompound(corpos), saida, "montagem.stl")
    prev = os.path.join(PROJ, "previews")
    for (pa, ti, cu), arq, pos in (((30, 15, 0.0), "montagem.png", (330, -330, 260)),
                                   ((0, 40, BE["empurrador"]["curso"]), "montagem_tilt_40_disparo.png", (60, -420, 160)),
                                   ((-60, -20, 0.0), "montagem_pan_menos_60.png", (300, 260, 220))):
        itens = [(posar(p, pa, ti, cu), p.cor) for p in pecas]
        render(itens, os.path.join(prev, arq), foco=(10, 0, 80), posicao=pos)
    print(f"  STEP e STL da montagem em {saida}/; renders em {prev}/  ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
