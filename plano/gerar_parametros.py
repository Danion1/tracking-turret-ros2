#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gerar_parametros.py — a ÚNICA fonte das cotas da torreta.

Lê os dados de componentes (../fontes/componentes.json, cada número com a
fonte do datasheet) e as ESCOLHAS de projeto abaixo (cada uma com o porquê),
calcula o arranjo (onde fica cada peça), roda as conferências e escreve
parametros.json. A figura (../figura/build.mjs) e, depois, o CAD em CadQuery
leem o MESMO parametros.json: os dois não podem divergir.

Unidades: mm, graus, gramas. Eixos: x = para onde o cano aponta com pan 0,
y = esquerda, z = para cima. Origem no eixo do pan, no tampo da base (z = 0).
Coordenadas do BERÇO (o que inclina): origem no eixo do tilt, x ao longo do
cano, z para cima com tilt 0.

Rodar:  python3 gerar_parametros.py      (sai com 1 se alguma conferência falhar)
"""

import json
import math
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
ERROS = []
ORIGEM = {}  # caminho da cota -> de onde veio


def confere(ok, msg):
    print(("  ok    " if ok else "  FALHA ") + msg)
    if not ok:
        ERROS.append(msg)


def ler_componentes():
    caminho = os.path.join(AQUI, "..", "fontes", "componentes.json")
    with open(caminho, encoding="utf-8") as f:
        return json.load(f)


def val(C, comp, campo, padrao=None, origem_padrao=None):
    """Valor de um campo do componentes.json; registra a origem."""
    item = C.get(comp, {}).get(campo)
    chave = f"{comp}.{campo}"
    if isinstance(item, dict) and item.get("valor") is not None:
        ORIGEM[chave] = f"{item.get('tipo', '?')}: {item.get('fonte', '?')} — {item.get('item', '')}"
        return item["valor"]
    if padrao is None:
        raise KeyError(f"falta {chave} em componentes.json")
    ORIGEM[chave] = origem_padrao or "ASSUMIDO (sem fonte)"
    return padrao


def pecas_da_figura(C):
    """Converte o componentes.json (fonte por campo) no formato que a figura e
    o CAD usam. Onde o datasheet não publica, o valor entra ASSUMIDO, com o
    motivo, e fica listado em ORIGEM."""
    st, sg = "feetech_sts3215_c001", "servo_sg90_towerpro"
    mo, da, la = "motor_mabuchi_fa130ra", "dardo_nerf_elite", "lancador_volante"
    r68, r08, wc = "rolamento_6808_2rs", "rolamento_608_2z", "webcam_logitech_c920"
    K = {}
    vel = val(C, st, "vel_sem_carga_7v4")
    horn = val(C, st, "horn_padrao")
    estria = val(C, st, "estria")
    K["sts3215"] = {
        "caixa": val(C, st, "caixa_mm"), "eixo_dx": val(C, st, "eixo_ao_centro_da_caixa_mm"),
        "horn_r": horn["diametro_mm"] / 2, "horn_e": horn["espessura_mm"],
        "horn_furos_circulo": horn["circulo_furos_d_mm"], "horn_furos_n": horn["furos_n"], "horn_furo_d": horn["furos_d_mm"],
        "estria_r": estria["diametro_externo_mm"] / 2,
        "estria_d": estria["diametro_externo_mm"], "estria_dentes": estria["dentes"],
        "estria_saliencia": val(C, st, "estria_saida_projecao_mm"), "altura_lado_eixo": val(C, st, "altura_corpo_lado_eixo_mm"),
        "furos_cima": val(C, st, "furos_fixacao_posicoes_mm"),
        "torque_kgcm": val(C, st, "torque_stall_7v4_kgfcm"), "tensao": 7.4, "s60": vel["s_por_60graus"],
        "massa_g": val(C, st, "massa_g"), "torque_nominal_kgcm": val(C, st, "torque_nominal_7v4_kgfcm"), "topo": val(C, st, "topo_alturas_step_mm"), "fundo": val(C, st, "fundo_alturas_step_mm"),
        "fonte": "Feetech, especificação A/0 (2020-04-10), via Core Electronics; horn: STEP do SO-ARM100 (ref. cruzada)"}
    dim = val(C, sg, "dim_mm")
    abas = val(C, sg, "cotas_abas_compilacao")
    K["sg90"] = {"caixa": [dim[0], dim[1], abas["C_altura_ate_topo_caixa"] - 6.0],
                 "eixo_dx": val(C, sg, "eixo_posicao_mm", 5.5, "ASSUMIDO: eixo a ~6 mm da ponta (não publicado; medir)"),
                 "torque_kgcm": val(C, sg, "torque_stall_kgfcm"), "s60": val(C, sg, "vel_analog_s_por_60"),
                 "massa_g": val(C, sg, "massa_g"), "fonte": "TowerPro (página do SG90); cotas das abas: compilação sem autor"}
    v18 = val(C, mo, "variante_18100")
    K["motor130"] = {"modelo": "Mabuchi FA-130RA-18100", "L": val(C, mo, "comprimento_lata_mm"),
                     "larg": val(C, mo, "carcaca_diametro_mm"), "plano": val(C, mo, "carcaca_faces_planas_mm"),
                     "eixo_d": val(C, mo, "eixo_diametro_mm"), "rpm": v18["rpm_sem_carga"], "tensao": v18["tensao_nominal_v"],
                     "massa_g": val(C, mo, "massa_g"), "fonte": "Mabuchi FA-130RA (PDF, cópia Pololu)"}
    K["r6808"] = {"d": val(C, r68, "d_mm"), "D": val(C, r68, "D_mm"), "B": val(C, r68, "B_mm"),
                  "massa_g": val(C, r68, "massa_kg") * 1000, "D2": val(C, r68, "D2_aprox_mm"), "d2": val(C, r68, "d2_aprox_mm"), "r12": val(C, r68, "r12_min_mm"),
                  "C_kN": val(C, r68, "C_kN"), "C0_kN": val(C, r68, "C0_kN"), "fonte": "SKF 61808-2RS1 (catálogo 17000)"}
    K["r608"] = {"d": val(C, r08, "d_mm"), "D": val(C, r08, "D_mm"), "B": val(C, r08, "B_mm"),
                 "C_kN": val(C, r08, "C_kN"), "massa_g": val(C, r08, "massa_kg") * 1000, "encostos": val(C, r08, "encostos_mm"), "fonte": "SKF 608-2Z (catálogo 17000)"}
    comp = val(C, la, "compressao_tipica_mm")
    K["dardo"] = {"d": val(C, da, "diametro_mm"), "L": val(C, da, "comprimento_mm"), "massa_g": val(C, da, "massa_g"),
                  "ponta_l": val(C, da, "ponta", 10.0, "ASSUMIDO: ponta de ~10 mm (medir com paquímetro)"),
                  "compressao": round(sum(comp) / 2, 2),
                  "fonte": "medição publicada (BlasterHub: 73,4 mm, 1,02 g; BlasterCentral: 0,5\")"}
    ORIGEM["dardo.compressao"] = "meio da faixa 2,7–4,0 mm (dedução do subagente sobre projetos abertos)"
    K["volante_ref"] = {"d_faixa": val(C, la, "volante_diametro_faixa_gryphon_mm"), "fonte": "Gryphon (CC0) e Stryfe"}
    K["webcam"] = {"modelo": "Logitech C920", "larg": 94.0, "prof": 24.0, "alt": 29.0, "lente_r": 7.0,
                   "fov": val(C, wc, "campo_visao_diagonal_graus"), "massa_g": val(C, wc, "massa_g"),
                   "fonte": "Logitech C920s (largura 94, 162 g, 78°); corpo sem o clipe 24 × 29 ASSUMIDO (medir a sua webcam)"}
    ORIGEM["webcam.corpo"] = "ASSUMIDO: a página só dá as medidas COM o clipe (94 × 71 × 43,3)"
    K["impressora"] = {"mesa": val(C, "impressora_flashforge_inventor", "volume_mm")}
    pl = val(C, "esp32_s3_devkitc_1", "placa_mm")
    K["esp32"] = {"comp": pl[0], "larg": pl[1], "fonte": "Espressif, desenho da ESP32-S3-DevKitC-1 v1"}
    ad = val(C, "adaptador_waveshare_bus_servo_a", "placa_mm")
    K["adaptador"] = {"comp": ad[0], "larg": ad[1], "fonte": "Waveshare Bus Servo Adapter (A), wiki; ATENÇÃO: entrada 9–12,6 V"}
    return K


def fisica_disparo(K, berco, E):
    """Do empurrador ao alvo. Devolve {nome: {"v": valor, "un": unidade,
    "tipo": calculado|datasheet|fonte|ASSUMIDO, "de": origem}} e a tabela
    da trajetória. Modelo simples, para dimensionar; o protótipo mede."""
    D = {}

    def put(nome, v, un, tipo, de):
        D[nome] = {"v": v, "un": un, "tipo": tipo, "de": de}
        return v

    g = 9.81
    mot = K["motor130"]
    dardo = K["dardo"]
    vol = berco["volante"]
    # --- volante + motor (modelo linear do motor CC: torque cai com a rotação)
    rpm0 = put("rpm_sem_carga", mot["rpm"], "rpm", "datasheet", "Mabuchi FA-130RA-18100 a 3 V")
    t_stall = put("torque_stall", 3.53e-3, "N·m", "datasheet", "Mabuchi FA-130RA-18100: 3,53 mN·m")
    rho_pla = put("densidade_pla", 1.24, "g/cm³", "ASSUMIDO", "valor típico de ficha de PLA; o volante sai 100% preenchido")
    r = vol["r"] / 1000
    l = vol["l"] / 1000
    m_vol = put("massa_volante", rho_pla * 1000 * math.pi * r * r * l * 1000, "g", "calculado", "disco maciço Ø36 × 10")
    i_vol = put("inercia_volante", 0.5 * (m_vol / 1000) * r * r, "kg·m²", "calculado", "½·m·r² (rotor do motor desprezado: ASSUMIDO)")
    w0 = rpm0 * 2 * math.pi / 60
    tau = put("tau_aceleracao", i_vol * w0 / t_stall, "s", "calculado", "τ = I·ω₀ / T_stall (motor linear)")
    put("tempo_95", 3 * tau, "s", "calculado", "95% da rotação em 3τ")
    v_borda = put("vel_borda", w0 * r, "m/s", "calculado", "ω·r")
    eta = put("eficiencia_velocidade", 0.85, "", "ASSUMIDO", "fração da velocidade da borda que o dardo leva (escorregamento); faixa 0,75–0,95: medir com cronógrafo")
    v0 = put("vel_saida", eta * v_borda, "m/s", "calculado", "η · velocidade da borda")
    put("vel_referencia", 72 * 0.3048, "m/s", "fonte", "Stryfe original: 72 fps (FONTES.md, lançador de volante)")
    m = dardo["massa_g"] / 1000
    ke = put("energia_dardo", 0.5 * m * v0 * v0, "J", "calculado", "½·m·v²")
    e_vol = put("energia_volantes", 2 * 0.5 * i_vol * w0 * w0, "J", "calculado", "dois volantes a ω₀")
    eta_e = put("eficiencia_energia", 0.5, "", "ASSUMIDO", "metade da energia tirada dos volantes vira energia do dardo (o resto: escorregamento e espuma)")
    w1 = math.sqrt(max(0.0, w0 * w0 - (ke / eta_e) / i_vol))
    put("queda_rpm", 100 * (1 - w1 / w0), "%", "calculado", "rotação que cai em um disparo")
    # --- contato: compressão e força
    delta = vol["compressao"] / 2
    corda = 2 * math.sqrt(vol["r"] ** 2 - (vol["r"] - delta) ** 2)
    put("compressao_por_lado", delta, "mm", "calculado", "(dardo − vão) / 2")
    put("corda_contato", corda, "mm", "calculado", "trecho do volante que aperta o dardo")
    s_min, s_max = corda / 1000, dardo["L"] / 1000
    a_max, a_min = v0 * v0 / (2 * s_min), v0 * v0 / (2 * s_max)
    put("aceleracao", [a_min, a_max], "m/s²", "calculado", "v²/(2s), s entre o comprimento do dardo e a corda de contato")
    f_min, f_max = m * a_min, m * a_max
    put("forca_no_dardo", [f_min, f_max], "N", "calculado", "m·a (soma dos dois volantes)")
    mu = put("atrito_espuma_pla", 0.6, "", "ASSUMIDO", "coeficiente espuma × PLA não publicado: medir (plano inclinado)")
    n_min, n_max = f_min / 2 / mu, f_max / 2 / mu
    put("normal_por_volante", [n_min, n_max], "N", "calculado", "F / 2 / μ")
    put("rigidez_espuma_necessaria", [n_min / delta, n_max / delta], "N/mm", "calculado", "N / compressão por lado (a espuma tem de dar pelo menos isso: medir)")
    # --- empurrador
    sg = K["sg90"]
    t_sg = sg["torque_kgcm"] * 9.80665 / 100
    rp = berco["empurrador"]["rp"] / 1000
    put("forca_empurrador", t_sg / rp, "N", "calculado", "torque de stall do SG90 / raio primitivo do pinhão")
    w_sg = (math.pi / 3) / sg["s60"]
    v_emp = put("vel_empurrador", w_sg * rp * 1000, "mm/s", "calculado", "60° em 0,12 s no raio de 10 mm (sem carga)")
    t_emp = put("tempo_empurrao", berco["empurrador"]["curso"] / v_emp, "s", "calculado", "curso / velocidade")
    put("cadencia_max", 1 / (2 * t_emp + 0.05), "tiros/s", "calculado", "ida e volta do empurrador + 50 ms para o dardo cair (ASSUMIDO)")
    # --- voo (arrasto quadrático)
    cd = put("cd_dardo", 0.62, "", "ASSUMIDO", "faixa publicada 0,46–0,82 (simulação UAF 0,62; vídeo 0,464; cilindro 0,82)")
    rho_ar = put("densidade_ar", 1.2, "kg/m³", "fonte", "ar a ~20 °C")
    area = math.pi * (dardo["d"] / 2000) ** 2
    kk = 0.5 * rho_ar * cd * area / m
    put("k_arrasto", kk, "1/m", "calculado", "½·ρ·Cd·A / m")

    def voo(ang_deg, x_max=10.0, dt=1e-4):
        vx, vz = v0 * math.cos(math.radians(ang_deg)), v0 * math.sin(math.radians(ang_deg))
        x = z = t = 0.0
        pts = [(0.0, 0.0, 0.0, v0)]
        prox = 0.25
        while x < x_max and z > -1.5 and t < 3:
            v = math.hypot(vx, vz)
            vx += -kk * v * vx * dt
            vz += (-g - kk * v * vz) * dt
            x += vx * dt
            z += vz * dt
            t += dt
            if x >= prox:
                pts.append((round(prox, 2), round(z, 4), round(t, 4), round(math.hypot(vx, vz), 2)))
                prox += 0.25
        return pts

    tab = voo(0.0, x_max=6.0)
    D["trajetoria_horizontal"] = {"v": [p for p in tab if p[0] in (1.0, 2.0, 3.0, 4.0, 5.0)], "un": "(x m, z m, t s, v m/s)", "tipo": "calculado", "de": "integração com arrasto, tiro na horizontal"}
    D["trajetoria_pontos"] = {"v": tab, "un": "(x m, z m, t s, v m/s)", "tipo": "calculado", "de": "a cada 0,25 m (a figura usa)"}
    # alcance no chão a partir da altura do cano (~0,2 m), na horizontal
    return D


def opcao_dardo_impresso(K, berco, D1):
    """Opção 2 (Daniel, 2026-11-05): dardo IMPRESSO, rígido, ao lado da espuma.
    Mesmos volantes e motores; o aperto tem de vir de um pneu de TPU no
    volante (o dardo não comprime). Devolve o mesmo formato de fisica_disparo."""
    D = {}

    def put(nome, v, un, tipo, de):
        D[nome] = {"v": v, "un": un, "tipo": tipo, "de": de}
        return v

    d, L = K["dardo"]["d"], K["dardo"]["L"]
    parede = put("parede", 0.8, "mm", "ASSUMIDO", "2 perímetros de 0,4 mm")
    ponta = put("ponta_tpu", 10.0, "mm", "ASSUMIDO", "ponta maciça de TPU 95A (macia e pesada: CG à frente, estável)")
    rho_pla, rho_tpu = 1.24, put("densidade_tpu", 1.22, "g/cm³", "ASSUMIDO", "valor típico de TPU 95A")
    area_anel = math.pi / 4 * (d * d - (d - 2 * parede) ** 2)
    m_pla = area_anel * (L - ponta) * rho_pla / 1000
    m_tpu = math.pi / 4 * d * d * ponta * rho_tpu / 1000
    m_g = put("massa", m_pla + m_tpu, "g", "calculado", f"tubo de PLA Ø{d} parede {parede} ({m_pla:.2f} g) + ponta de TPU ({m_tpu:.2f} g)")
    put("massa_espuma", K["dardo"]["massa_g"], "g", "fonte", "dardo de espuma comercial (BlasterHub: 1,02 g)")
    m = m_g / 1000
    v0 = put("vel_saida", D1["vel_saida"]["v"], "m/s", "ASSUMIDO", "mesma fração da borda da espuma (o dardo rígido escorrega diferente: medir)")
    ke = put("energia_dardo", 0.5 * m * v0 * v0, "J", "calculado", "½·m·v²")
    put("energia_espuma", D1["energia_dardo"]["v"], "J", "calculado", "opção 1, mesma velocidade")
    i_vol = D1["inercia_volante"]["v"]
    w0 = D1["rpm_sem_carga"]["v"] * 2 * math.pi / 60
    eta_e = D1["eficiencia_energia"]["v"]
    w1 = math.sqrt(max(0.0, w0 * w0 - (ke / eta_e) / i_vol))
    queda = put("queda_rpm", 100 * (1 - w1 / w0), "%", "calculado", "rotação que cai em um disparo")
    tau = D1["tau_aceleracao"]["v"]
    put("recuperacao_1pct", tau * math.log(max(1.0, queda)), "s", "calculado", "τ·ln(queda/1%) até voltar a 1% da rotação")
    put("recuperacao_1pct_espuma", tau * math.log(max(1.0, D1["queda_rpm"]["v"])), "s", "calculado", "opção 1")
    # aperto: pneu de TPU no volante
    s_min = D1["corda_contato"]["v"] / 1000
    s_max = L / 1000
    f = [m * v0 * v0 / (2 * s_max), m * v0 * v0 / (2 * s_min)]
    put("forca_no_dardo", f, "N", "calculado", "m·v²/(2s), mesma faixa de s da espuma")
    mu = put("atrito_tpu_pla", 0.5, "", "ASSUMIDO", "TPU × PLA não publicado: medir")
    n = [x / 2 / mu for x in f]
    put("normal_por_volante", n, "N", "calculado", "F / 2 / μ")
    e_tpu = put("modulo_tpu_95a", 26.0, "MPa", "datasheet", "NinjaTek Cheetah 95A: 26 MPa (ASTM D638)")
    esp = put("espessura_pneu", 3.0, "mm", "ASSUMIDO", "banda de TPU no volante")
    a_cont = put("area_contato", 15.0 * 5.0, "mm²", "ASSUMIDO", "corda de ~15 mm × ~5 mm de largura no dardo")
    k_pneu = put("rigidez_pneu", e_tpu * a_cont / esp, "N/mm", "calculado", "E·A/t (compressão simples)")
    put("compressao_pneu", [x / k_pneu for x in n], "mm", "calculado", "N / rigidez: é a interferência que o vão tem de ter")
    put("precisao_impressora", 0.1, "mm", "fonte", "Flashforge Inventor ±0,1 mm (FONTES.md)")
    # segurança (EN 71-1:2014+A1:2018, resumo QIMA)
    a_ponta = math.pi * (d / 2000) ** 2
    put("limite_rigida", 0.08, "J", "fonte", "EN 71-1: ponta rígida ≤ 0,08 J (resumo QIMA)")
    put("limite_resiliente", 0.5, "J", "fonte", "EN 71-1: ponta resiliente ≤ 0,5 J e ≤ 2500 J/m² (resumo QIMA)")
    put("energia_por_area", ke / a_ponta, "J/m²", "calculado", f"energia / área da ponta Ø{d}")
    put("energia_por_area_espuma", D1["energia_dardo"]["v"] / a_ponta, "J/m²", "calculado", "opção 1")
    v_max = math.sqrt(2 * min(0.5, 2500 * a_ponta) / m)
    put("vel_max_segura", v_max, "m/s", "calculado", "maior velocidade com ponta macia que respeita 0,5 J e 2500 J/m²")
    put("vel_max_rigida", math.sqrt(2 * 0.08 / m), "m/s", "calculado", "se a ponta fosse rígida (0,08 J)")
    # voo: mesmo Cd, mais massa → menos desaceleração
    kk = D1["k_arrasto"]["v"] * K["dardo"]["massa_g"] / m_g
    put("k_arrasto", kk, "1/m", "calculado", "mesmo Cd e área, massa maior")
    return D


def main():
    C = ler_componentes()
    K = {}  # peças compradas, já no formato que a figura e o CAD usam

    # ------------------------------------------------------------------ #
    # 1. Peças compradas (números do componentes.json, com fonte)         #
    # ------------------------------------------------------------------ #
    K = pecas_da_figura(C)
    ins = "inserto_m3_ruthex_rx_m3x5_7"
    K["inserto_m3"] = {"furo_d": val(C, ins, "D3_furo_recomendado_mm"), "L": val(C, ins, "L_mm"),
                       "parede_min": val(C, ins, "parede_min_mm"), "fundo_extra": val(C, ins, "furo_cego_profundidade_extra_mm"),
                       "fonte": "Ruthex GE-M3x57-001 (3DJake) + CNC Kitchen (furo cego +1 mm)"}

    # ------------------------------------------------------------------ #
    # 2. ESCOLHAS de projeto (o porquê ao lado)                           #
    # ------------------------------------------------------------------ #
    E = {
        # Daniel, 2026-11-05: pinhão 20T + coroa 40T, módulo 1,5 (2:1), rolamento 6808.
        "m": 1.5, "z_coroa": 40, "z_pinhao": 20,
        "folga_dente": 0.15,      # ASSUMIDO: backlash na primitiva, FDM bico 0,4
        "largura_dente": 8.0,     # ASSUMIDO: ~5 módulos de face (PLA)
        "ang_pinhao": 75.0,       # ASSUMIDO: lado que a figura vê; no real tanto faz
        "folga_giro": 0.5,        # ASSUMIDO: entre o que gira e o que fica parado
        "parede": 2.4,            # ASSUMIDO: 6 perímetros de 0,4 mm
        "tilt": [-20.0, 40.0],    # ASSUMIDO: nariz −20° a +40° (drone no alto do quarto)
        "pan": [-90.0, 90.0],     # consequência: servo 360° ÷ 2 (redução 2:1)
        # lançador impresso (Daniel, 2026-11-05)
        "volante_d": 36.0,        # ASSUMIDO: típico de lançador com motor 130 (ver FONTES.md)
        "volante_l": 10.0,
        "dardos_funil": 3,
        "emp_m": 1.0, "emp_z": 20,  # ASSUMIDO: pinhão 20T módulo 1 no SG90
        "base_alt": 45.0,         # cabe o STS3215 em pé + placas
        # Parte B, peça 2 (2026-11-06): o tampo é placa própria (o cubo fica em cima, então ele não
        # pode imprimir com o topo na mesa) e passa de 3 para 6 mm: o servo do pan fica preso e
        # posicionado PELO TAMPO, a mesma peça do cubo, num rebaixo do contorno dele (2,5 mm de
        # encaixe). Assim o entre-eixos de 45 depende de uma peça só.
        "tampo": 6.0,
        "servo_pan_ang": -15.0,   # ASSUMIDO: direção eixo → traseira do servo; corpo para +x, longe do centro e das paredes
        "folga_rebaixo": 0.15,    # ASSUMIDO: por lado, no contorno do servo (encaixe que posiciona)
        "coluna_alt": 21.5,       # deixa o engrenamento à vista sob a mesa (conta em Plano das peças)
    }
    V = {"escolhas": E, "pecas": K}

    # ------------------------------------------------------------------ #
    # 3. Arranjo                                                          #
    # ------------------------------------------------------------------ #
    m = E["m"]
    zc, zp = E["z_coroa"], E["z_pinhao"]
    rpc, rpp = m * zc / 2, m * zp / 2
    entre = rpc + rpp
    a = math.radians(E["ang_pinhao"])
    rol = K["r6808"]
    g = E["folga_giro"]

    pan = {}
    # ressalto do anel interno: até d2 (ombro do anel interno), para não raspar na vedação,
    # que gira com o anel externo (Ø44 do plano passava de d2 = 42,1)
    pan["cubo"] = {"r": rol["d2"] / 2 - 0.05, "z0": 0.0, "z1": 2.0,
                   "assento_r": rol["d"] / 2, "assento_z1": 2.0 + rol["B"] - 0.5, "chanfro": 0.5, "furo_r": 10.0,
                   # retenção do anel interno: ajuste + cola CA (ASSUMIDO, 2026-11-06). Não cabe arruela
                   # por cima: a coroa passa 0,5 mm acima do anel interno.
                   "retencao": "ajuste no Ø40 + gota de cianoacrilato"}
    pan["rolamento"] = {"z0": 2.0, "z1": 2.0 + rol["B"], "r": rol["D"] / 2}
    z_dente0 = pan["rolamento"]["z1"] + g
    pan["coroa"] = {"m": m, "z": zc, "rp": rpc, "ra": rpc + m, "rf": rpc - 1.25 * m,
                    "z0": z_dente0, "z1": z_dente0 + E["largura_dente"], "folga": E["folga_dente"]}
    pan["pinhao"] = {"m": m, "z": zp, "rp": rpp, "ra": rpp + m, "rf": rpp - 1.25 * m,
                     "eixo": [entre * math.cos(a), entre * math.sin(a)],
                     "z0": z_dente0, "z1": z_dente0 + E["largura_dente"], "folga": E["folga_dente"],
                     # CAD da parte B (2026-11-06): o cubo de Ø16 da figura não cabia os 4 furos
                     # do horn (círculo de 14) com o rebaixo Ø6,5 das cabeças: vira Ø24.
                     "cubo_r": 12.0,
                     "cubo_z0": g,              # face de baixo do cubo = topo do horn, a folga_giro acima do tampo
                     # bolsa por baixo para a cabeça do M3×6 central do horn (o horn vai no servo ANTES do pinhão).
                     # Não é passante: um furo passante deixava 0,25 mm de parede até o rebaixo das cabeças.
                     "bolsa_d": 7.0, "bolsa_prof": 3.5,  # ASSUMIDO: cabeça ISO 4762 M3 (Ø5,5 × 3) + folga; MEDIR
                     "rosca_horn": "M3",        # ASSUMIDO: furo Ø2,5 do STEP = broca do macho M3 (ISO); MEDIR no servo
                     "fundo_rebaixo_min": 4.5,  # bolsa (3,5) + 1 mm: o rebaixo não pode descer até a altura da bolsa
                     "engajamento_min": 2.0}    # ASSUMIDO: 4 fios de M3 no horn de metal
    pan["garras"] = {"n": 6, "r0": rol["D"] / 2, "r1": pan["coroa"]["rf"] - 0.3,
                     "z0": pan["rolamento"]["z0"] + 1.0, "z1": z_dente0, "largura_ang": 14.0,
                     # CAD da parte B: interferência RADIAL de 0,1 mm (dedo de PLA de 1,8 mm flexiona;
                     # deformação 0,65 %, conta no relatório da peça) e chanfro de entrada de 0,5
                     "interferencia": 0.1, "chanfro": 0.5}
    # coroa apoiada no anel EXTERNO (o que gira junto): anel de contato entre D2 e o chanfro r1,2
    pan["coroa"]["assento"] = {"r0": rol["D2"] / 2 + 0.25, "r1": rol["D"] / 2 - rol["r12"] - 0.1,
                               "z0": pan["rolamento"]["z1"], "z1": z_dente0}
    pan["coroa"]["furo_r"] = 10.0                           # Ø20 dos cabos (Plano das peças)
    pan["coroa"]["insertos"] = {"n": 4, "r": 14.0, "ang0": 45.0}   # M3 que prendem a coluna/mesa
    pan["coluna"] = {"r": 18.0, "z0": pan["coroa"]["z1"], "z1": pan["coroa"]["z1"] + E["coluna_alt"]}
    mesa_z0 = pan["coluna"]["z1"]
    pan["mesa"] = {"x": -40.0, "y": -63.0, "w": 80.0, "d": 126.0, "r": 8.0, "z0": mesa_z0, "z1": mesa_z0 + 6.0}
    pan["entre_eixos"] = entre
    pan["razao"] = zc / zp
    V["pan"] = pan

    # Berço (coordenadas do berço: origem no eixo do tilt)
    d = K["dardo"]["d"]
    comp = K["dardo"].get("compressao", 3.5)
    vr = E["volante_d"] / 2
    folga_vol = d - comp
    y_vol = folga_vol / 2 + vr
    mot = K["motor130"]
    # O eixo do tilt fica perto do meio do comprimento do berço (balanço).
    L = K["dardo"]["L"]
    berco = {}
    # folga lateral de 0,1 mm: as paredes guiam o dardo (espuma) e seguram a pilha do funil
    berco["calha"] = {"x0": -29.0, "x1": -29.0 + L + 1.0, "y0": d / 2 + 0.1, "y1": d / 2 + 2.1}
    berco["boca"] = {"x0": 90.0, "x1": 104.0}
    berco["piso"] = {"x0": berco["calha"]["x0"] - 66.0, "x1": berco["boca"]["x1"], "y": 46.0, "z0": -d / 2 - 3.0, "z1": -d / 2}
    berco["face"] = {"y0": 46.0, "y1": 50.0, "x0": -18.0, "x1": 18.0, "z0": berco["piso"]["z0"] - 18.0, "z1": 16.0}
    berco["volante"] = {"r": vr, "l": E["volante_l"], "x": 65.0, "y": y_vol, "folga": folga_vol, "compressao": comp}
    berco["gaiola"] = {"x0": berco["calha"]["x1"] + 2.5 + 0.5, "x1": 90.0, "y": 44.0, "z_baixo": berco["piso"]["z1"], "z0": vr * 0 + E["volante_l"] / 2 + 1.5, "esp": 3.0}
    berco["motor"] = {"x": 65.0, "y": y_vol, "z1": berco["piso"]["z0"], "z0": berco["piso"]["z0"] - mot["L"]}
    berco["funil"] = {"n": E["dardos_funil"], "passo": d + 0.2}
    rp_e = E["emp_m"] * E["emp_z"] / 2
    # Curso do empurrador: até a ponta do dardo tocar a borda dos volantes;
    # dali os volantes puxam o dardo (conferir no protótipo; o pinhão ainda
    # tem giro sobrando até 180° se precisar de mais curso).
    vol = berco["volante"]
    toque = vol["x"] - math.sqrt(vol["r"] ** 2 - (vol["y"] - d / 2) ** 2)
    ponta0 = berco["calha"]["x0"] + 0.5 + L
    # + 1,0: parada, a frente da cremalheira fica 1,0 atrás do dardo (calha x0 − 0,5 contra x0 + 0,5), para o
    # dardo cair do funil; esse 1,0 a cremalheira anda antes de encostar (achado no CAD da peça 5, 2026-11-06)
    # + 2,0 de PEGADA (Daniel, 2026-11-10): a ponta entra 2 mm além do toque, para os volantes pegarem com
    # força (margem ASSUMIDA, conferir no protótipo). Antes era − 0,3: a ponta parava 0,36 mm ANTES do
    # toque e o dardo não saía (medido no Gazebo, ros2/testar_dardo.sh).
    pegada = 2.0
    curso = round(toque - ponta0 + 1.0 + pegada, 1)
    berco["empurrador"] = {"m": E["emp_m"], "z": E["emp_z"], "rp": rp_e, "curso": curso,
                           "giro_deg": math.degrees(curso / rp_e),
                           "x_pinhao": -50.0, "y_primitiva": 4.0}  # lado +y: à vista na figura
    # Webcam (opção A, escolha do Daniel): a perna do clipe dobra 30 mm para trás;
    # a câmera avança até a perna, a aba e a borda do suporte passarem à frente do funil.
    perna, aba, borda, folga_f = 30.0, 4.0, 2.8, 0.5
    frente_funil = berco["calha"]["x1"] + 2.5
    wx = frente_funil + folga_f + aba + perna - 6.0
    berco["webcam"] = {"x": round(wx, 1), "suporte_x1": round(wx + 6.0 + borda, 1)}
    zt = None
    V["berco"] = berco

    # Altura do eixo do tilt: o ponto mais baixo do berço, em toda a faixa,
    # passa a mesa com folga.
    pts = [(berco["piso"]["x0"], berco["piso"]["z0"]), (berco["piso"]["x1"], berco["piso"]["z0"]), (-50.0, berco["piso"]["z0"] - K["sg90"]["caixa"][2]),
           (65.0 + mot["larg"] / 2, berco["motor"]["z0"]), (65.0 - mot["larg"] / 2, berco["motor"]["z0"]),
           (berco["face"]["x0"], berco["face"]["z0"]), (berco["face"]["x1"], berco["face"]["z0"])]
    baixo = 0.0
    for t in [E["tilt"][0] + i * (E["tilt"][1] - E["tilt"][0]) / 60 for i in range(61)]:
        tr = math.radians(t)
        for x, z in pts:
            baixo = min(baixo, z * math.cos(tr) + x * math.sin(tr))
    zt = math.ceil(pan["mesa"]["z1"] - baixo + 4.0)
    V["tilt"] = {"z": zt, "faixa": E["tilt"], "mais_baixo": baixo}
    # Braço −y: rolamento 608 + eixo M8. Braço +y: o STS3215 fica ENTRE o berço
    # e o braço (ele não tem orelhas: prende pela face de baixo no braço, e a
    # estria de 3,4 mm entra no horn aparafusado na face do berço).
    st = K["sts3215"]
    horn_e = 2.5  # espessura do horn (STEP do SO-ARM100, referência cruzada)
    y_face = 50.0
    y_servo0 = y_face + horn_e + st["estria_saliencia"]
    y_servo1 = y_servo0 + st["caixa"][2]
    V["bracos"] = {"y0": 57.0, "y1": 63.0, "x0": -20.0, "x1": 20.0, "z0": pan["mesa"]["z1"], "topo": zt + 20.0,
                   "horn_e": horn_e, "servo_y0": y_servo0, "servo_y1": y_servo1, "ys0": y_servo1, "ys1": y_servo1 + 6.0}
    # ---- CAD da peça 3 (2026-11-06): as alturas reais do STS3215 (STEP) refazem o lado +y.
    # O horn do tilt encosta na face do berço (y 50); o servo é posicionado e preso pelo BRAÇO +y,
    # num rebaixo da face de baixo dele (como o tampo faz com o servo do pan).
    tp_, fd_ = st["topo"], st["fundo"]
    esp_b = 8.0                                      # braços de 8 (o 608 tem 7; o rebaixo do servo come até 3,3)
    y_horn = y_face                                   # face de fora do horn = face do berço
    yb_in = y_horn + tp_["horn_z"][1] + 12.4          # face de dentro do braço +y: 2,0 de encaixe no contorno do servo (corpo até −14,4)
    tc = {"z": zt, "esp": esp_b, "y_horn": y_horn, "zstep_in": -12.4,
          "mais": {"y": [yb_in, yb_in + esp_b]},
          "menos": {"y": [-y_face - 7.0 - esp_b, -y_face - 7.0]},       # 7 de vão para o espaçador do eixo M8
          "x": [-20.0, 20.0], "z0": pan["mesa"]["z1"] - 1.0, "rebaixo_mesa": 1.0, "topo_r": 20.0,
          "parafusos_mesa": {"rosca": "M3", "x": [-12.0, 12.0]},
          "r608": {"D": K["r608"]["D"], "B": K["r608"]["B"], "furo_encosto": 19.6},   # entre D2 19,2 e Da máx 20 (SKF)
          "janela_conectores": {"x_step": [-5.0, 2.0], "y_step": [-10.5, 10.5]},
          "parafusos_servo": "2 × PA3.0 (os de trás da tampa de baixo do servo; original ×5 + 4,5 de braço: ~×10, MEDIR)"}
    V["tilt_cad"] = tc
    # ---- CAD da peça 4 (2026-11-06): berço = piso + 2 faces. Frame do berço: origem no eixo do tilt
    # (mundo (0, 0, zt) com tilt 0), x para o cano, z para cima. As cotas de antes (piso, face) ficam.
    V["berco_cad"] = {
        "piso_furos": {
            "motores": {"d": 6.7, "pontos": [[berco["motor"]["x"], berco["motor"]["y"]], [berco["motor"]["x"], -berco["motor"]["y"]]]},  # ressalto Ø6,15 + folga
            # peça 5 (2026-11-06): as colunas de trás foram de y ±40 para ±42. Em ±40, uma coluna que leva
            # inserto (Ø7,2) entrava 0,3 mm no volante; a de Ø5,2 da figura deixava só 0,7 de folga.
            "gaiola": {"d": 3.4, "pontos": [[berco["gaiola"]["x0"] + 4, s_ * (berco["gaiola"]["y"] - 2)] for s_ in (1, -1)]
                       + [[berco["gaiola"]["x1"] - 4, s_ * (berco["gaiola"]["y"] - 4)] for s_ in (1, -1)]},
            "usb_webcam": {"x": 38.0, "y": -32.0, "w": 18.0, "d": 10.0},     # o USB-A sobe para a webcam (fora do volante, da gaiola e da calha)
        },
        "sg90": {"janela_folga": 0.2, "rasgo_d": 2.4, "rasgo_entre": [26.0, 29.0],   # furos das orelhas NÃO publicados: rasgo
                 "x": berco["empurrador"]["x_pinhao"] - K["sg90"]["eixo_dx"], "y": berco["empurrador"]["y_primitiva"] + berco["empurrador"]["rp"]},
        "entalhe_horn": {"x": [-9.0, 9.0], "y_prof": 4.0},   # cabeças dos M3 do horn (descem a z −7,7, o piso está a −6,5)
        "ressaltos": {"x": [[-18.0, -10.0], [10.0, 18.0]], "esp_y": 8.0, "z1": 6.0, "rosca": "M3"},   # prendem as faces (insertos em y)
        "face_mais": {"bolsa_d": 7.0, "bolsa_prof": 2.0, "horn_rosca": "M3"},    # cabeça do parafuso central sai 1,5 além do horn
        "face_menos": {"ressalto_d": 20.0, "ponta_d": 10.4, "ponta_l": 0.5, "folga_braco": 0.5,
                       "porca_s": 13.0, "porca_m": 8.0, "porca_folga": 0.3, "furo_m8": 8.4,
                       "arruela_d": 10.4, "arruela_e": 1.5,             # arruela impressa: só no anel interno (da 10–10,5)
                       "furo_cabos": {"x": -8.0, "z": -14.0, "w": 18.0, "h": 10.0}},
        "m8": "ISO 4762 M8×20 (cabeça fora do braço −y, porca travante presa no ressalto da face −y)",
    }
    # ---- CAD da peça 5 (2026-11-06): o lançador
    d_ = K["dardo"]["d"]
    mo = "motor_mabuchi_fa130ra"
    V["lancador_cad"] = {
        "volante": {"furo": 1.9, "cubo_r": 5.0, "z0": berco["piso"]["z1"] + 0.5,      # furo: prensado no eixo Ø2 (ASSUMIDO 0,05 por lado)
                    "eixo_topo": berco["motor"]["z1"] + val(C, mo, "eixo_saliencia_mm")},  # ponta do eixo: face da lata + 9,4 (Mabuchi)
        "copo": {"folga": 0.15, "parede": 1.6, "fundo_z": berco["motor"]["z1"] - 15.0, "braco_e": 3.0, "braco_l": 8.0,
                 "fenda": 2.0, "canal_abracadeira": {"z": berco["motor"]["z1"] - 9.0, "l": 3.0, "prof": 0.8}},
        "gaiola": {"coluna_d": 7.2, "tampa_y": berco["piso"]["y"], "tampa_e": 3.0, "parafuso": "M3×25 ISO 4762 + porca (fora da tabela da Biblioteca: conferir)"},
        "funil": {"topo": d_ / 2 + berco["funil"]["n"] * berco["funil"]["passo"] + 2.0, "fundo_z0": d_ / 2 + 1.0, "frente_z0": d_ / 2 + 2.0,
                  "esp": 2.5, "fixacao": "cola cianoacrilato nos pés das paredes (ASSUMIDO)"},
        "cremalheira": {"x0": berco["calha"]["x0"] - 62.0, "x1": berco["calha"]["x0"] - 0.5, "y0": -2.5, "z1": berco["piso"]["z1"] + d_ - 1.0,
                        "dente_z": [-3.5, 2.5], "folga_flanco": 0.075},
        "pinhao_sg90": {"z": [-3.5, 2.5], "furo_d": 5.0, "folga": 0.15,
                        "fixacao": "no horn do SG90 (horn não cotado pela TowerPro: furar os 2 M2 na montagem)"},
        "guia": {"folga": 0.1, "parede": 2.0, "aba": 1.5, "x1": berco["calha"]["x0"] - 4.0},
        "usb_ponta": [58.0, -32.0, 14.0],    # a peça 6 (suporte da webcam) liga daqui
    }
    # ---- CAD da peça 6 (2026-11-06): suporte da webcam, opção A (Daniel, 2026-11-05)
    V["webcam_cad"] = {
        "perna": {"w": 30.0, "t": 12.0, "h": 30.0}, "aba": {"w": 30.0, "t": 4.0, "l": 22.0},
        "dobradica": {"w": 30.0, "t": 8.0, "h": 5.0},          # ASSUMIDO pela foto (medir a webcam do Daniel)
        "folga_rebaixo": 0.3, "borda": 2.5, "base": 6.5, "borda_h": 1.5,
        "placa_e": 3.0,                                         # placa do tamanho da tampa: presa pelos 4 M3 das colunas da gaiola
        "entalhe_usb": {"x": [berco["gaiola"]["x0"] - 1.0, 61.0], "y": [-36.0, -28.0]},
        "rosca_x": None,                                        # ASSUMIDO: a rosca 1/4"-20 no meio da perna (medir)
        "parafuso_14": {"cabeca_d": 11.1, "cabeca_h": 3.35, "furo": 6.6, "folga_cabeca": 0.5, "L": 7.94,
                        "nota": "1/4\"-20 × 5/16\" cabeça abaulada (medidas da cabeça ASSUMIDAS pela ASME B18.3, não lida: conferir); 3/8\" daria 6,6 de rosca e poderia bater no fundo da rosca da perna (~6, ASSUMIDO)"},
        "usb_saida": "ASSUMIDO: o cabo sai do meio da face de trás da webcam",
    }
    mesa_y0 = tc["menos"]["y"][0]
    mesa_y1 = tc["mais"]["y"][1] + 10.0              # 10 além do braço +y: furo do cabo do barramento
    pan["mesa"]["y"] = mesa_y0
    pan["mesa"]["d"] = mesa_y1 - mesa_y0
    pan["mesa"]["furo_r"] = 8.0                       # Ø16 na espessura da mesa (os rebaixos dos M3×30 a r 14 pedem parede)
    pan["coluna"]["furo_r"] = 10.0
    pan["coluna"]["parafusos"] = {"rosca": "M3", "L": 30, "rebaixo": 3.0}   # ISO 4762 M3×30: conferir no fornecedor
    pan["coluna"]["fendas"] = {"larg": 14.0, "alt": 8.0}   # saída dos cabos sob a mesa (±y); 14 deixa 1,2 até os furos M3 a 45°
    pan["mesa"]["furos_cabos"] = {"mais": {"x": -5.2, "y": mesa_y1 - 5.0, "w": 10.0, "d": 7.0},
                                  "menos": {"x": -8.0, "y": -y_face - 3.5, "w": 18.0, "d": 6.0}}  # o −y passa o plugue USB-A
    V["limites"] = {"pan": E["pan"], "tilt": E["tilt"]}
    # ---- caixa (peça 2b): eletrônica e fios. Cada placa é um ENVELOPE (caixa que contém a placa,
    # os componentes e o espaço dos conectores). Fios: pares de tubos com bitola ASSUMIDA (medir).
    xl, jk = val(C, "buck_xl4015_modulo", "dim_mm"), val(C, "jack_dc_pj002ah", "corpo_mm")
    wsd = val(C, "adaptador_waveshare_bus_servo_a", "placa_mm")
    zc0 = -E["base_alt"]                       # fundo da caixa
    zf = zc0 + E["parede"]                     # topo do piso
    pe = 2.5                                   # ASSUMIDO: trilhos/colunas sob as placas (solda por baixo)
    bx0, bx1 = -56.5, -56.5 + xl[0]
    def buck(nome, y0, v):
        return {"nome": nome, "tipo": "buck", "x": [bx0, bx1], "y": [y0, y0 + xl[1]], "z": [zf + pe, zf + pe + xl[2]],
                "saida_v": v, "fixacao": "berço com cerca + fita dupla-face (furação não cotada)", "fonte": "XL4015 Handsontec"}
    pb = {"x": [3.0, 63.0], "y": [-48.0, -2.0]}   # ASSUMIDO: placa perfurada 2,54 cortada em 60 × 46 (atrás do jack sobra 7,6 para os fios), furos Ø2 a 2,5 das bordas
    esp_l, esp_w = K["esp32"]["comp"], K["esp32"]["larg"]
    esp_y = (pb["y"][0] + pb["y"][1]) / 2
    zpb = zf + 4.0                               # placa perfurada sobre colunas de 4
    zesp = zpb + 1.6 + 8.5                       # ASSUMIDO: barra fêmea de 8,5 mm entre as placas
    V["caixa"] = {
        "z0": zc0, "z1": -E["tampo"], "piso": E["parede"], "parede": E["parede"], "z_piso": zf,
        "colunas": {"d": 10.0, "c": 63.0},
        "comps": [
            buck("buck 5 V (SG90)", -61.0, 5.0),
            buck("buck 3 V (motores)", -31.5, 3.0),
            buck("buck 7 V (servos)", -2.0, 7.0),
            {"nome": "Waveshare Bus Servo Adapter (A)", "tipo": "waveshare", "x": [-55.0, -55.0 + wsd[0]], "y": [27.0, 27.0 + wsd[1]],
             "z": [zf + 4.0, zf + 4.0 + 13.0], "furos": [37.0, 28.0], "fixacao": "4 × M2 em colunas de 4 (furo piloto Ø1,6 ASSUMIDO)",
             "fonte": "Waveshare (placa 42 × 33, furos 37 × 28); altura de 13 com o jack DC e o borne ASSUMIDA"},
            {"nome": "placa perfurada (central: MOSFET, diodo, bornes, barras do ESP32)", "tipo": "perfurada", "x": pb["x"], "y": pb["y"],
             "z": [zpb, zpb + 1.6 + 8.5], "fixacao": "4 × M2 em colunas de 4", "fonte": "ASSUMIDO: 60 × 46, a cortar"},
            {"nome": "ESP32-S3-DevKitC-1", "tipo": "esp32", "x": [66.4 - esp_l, 66.4], "y": [esp_y - esp_w / 2, esp_y + esp_w / 2],
             "z": [zesp, zesp + 1.6 + 3.5], "fixacao": "nas barras fêmeas da placa perfurada", "fonte": "Espressif (62,74 × 25,4); altura do módulo ASSUMIDA 3,5"},
            {"nome": "jack DC PJ-002AH (12 V)", "tipo": "jack", "x": [20.5 - jk["largura"] / 2, 20.5 + jk["largura"] / 2],
             "y": [-70.0, -70.0 + jk["comprimento"]], "z": [zf + 6.0, zf + 6.0 + jk["altura"]], "boca_z": zf + 6.0 + jk["boca_centro_altura"],   # 6: pinos de 3,5 + solda por baixo
             "fixacao": "berço impresso, boca rente à face de fora da parede -y", "fonte": "Same Sky PJ-002AH"},
        ],
        "aberturas": {   # na parede: [lado, centro_u, centro_z, largura, altura]
            "usb_esp32": ["+x", esp_y, zesp + 1.6, 22.0, 9.0],          # cobre os 2 conectores (posições não cotadas)
            "usb_webcam": ["+x", 12.0, -24.0, 18.0, 10.0],              # o plugue USB-A (~16 × 8 com a capa) passa
            "jack": ["-y", 20.5, zf + 6.0 + jk["altura"] / 2, jk["largura"] + 0.4, jk["altura"] + 0.4],
            "ventilacao_-x": ["-x", -18.0, -30.0, 70.0, 16.0],          # fendas sobre os bucks (ver caixa.py)
        },
        "fios": {"potencia_d": 2.4, "sinal_d": 1.4, "servo_d": 3.0, "usb_d": 4.5, "z_corredor": -20.0,
                 "fonte": "ASSUMIDO: 18 AWG silicone (Ø2,4), 26 AWG (Ø1,4), cabo do servo (Ø3), USB (Ø4,5); medir"},
        "feixe_central": {"r": 11.0, "z": [-24.0, -E["tampo"]]},   # cabos que sobem pelo furo de Ø20
    }
    # ---- base: tampo (placa com o cubo e o servo do pan) + caixa (piso e paredes)
    tp = K["sts3215"]["topo"]
    V["base"] = {"w": 140.0, "r": 7.0, "alt": E["base_alt"], "tampo": E["tampo"], "parede": E["parede"],
                 "cantos": 7.0,          # centro dos 4 M3 do tampo, a 7 mm das bordas (como a figura)
                 "rosca_cantos": "M3"}
    a_s = math.radians(E["servo_pan_ang"])
    V["servo_pan"] = {"eixo": pan["pinhao"]["eixo"], "ang": E["servo_pan_ang"],
                      "u": [math.cos(a_s), math.sin(a_s)],           # do eixo para a traseira
                      "dz": pan["pinhao"]["cubo_z0"] - tp["horn_z"][1],   # STEP → mundo: topo do horn no fundo do cubo do pinhão
                      "x_eixo_step": K["sts3215"]["eixo_dx"], "folga": E["folga_rebaixo"],
                      "parafusos": "2 × B-PA2.0 (os de trás da tampa de cima do servo, trocados por mais longos: original + 2,3 mm; MEDIR o original)",
                      "folga_topo": 0.3,      # acima do horn, que gira
                      "folga_estatica": 0.2}  # acima do que fica parado (a nervura da tampa do servo): pele de 1,0 mm

    # ------------------------------------------------------------------ #
    # 3b. Física do disparo (cada número: CALCULADO, DATASHEET, FONTE ou  #
    #     ASSUMIDO a medir no protótipo)                                 #
    # ------------------------------------------------------------------ #
    V["disparo"] = fisica_disparo(K, V["berco"], E)
    V["disparo_impresso"] = opcao_dardo_impresso(K, V["berco"], V["disparo"])

    # ------------------------------------------------------------------ #
    # 4. Conferências                                                     #
    # ------------------------------------------------------------------ #
    print("Conferências:")
    confere(abs(entre - 45.0) < 1e-9, f"entre-eixos (40 + 20) × 1,5 / 2 = {entre:.1f} mm")
    confere(pan["coroa"]["rf"] - rol["D"] / 2 >= E["parede"] - 0.7,
            f"anel das garras entre o rolamento (r {rol['D']/2}) e o pé da coroa (r {pan['coroa']['rf']:.3f}): {pan['coroa']['rf'] - rol['D']/2:.2f} mm")
    pc, ga, ins = pan["coroa"], pan["garras"], K["inserto_m3"]
    confere(pc["assento"]["r1"] - pc["assento"]["r0"] >= 0.5, f"assento da coroa no anel externo: r {pc['assento']['r0']:.2f}..{pc['assento']['r1']:.2f} (fora da vedação D2 {rol['D2']})")
    confere(min(pc["insertos"]["r"] - ins["furo_d"] / 2 - pc["furo_r"], pc["rf"] - pc["insertos"]["r"] - ins["furo_d"] / 2) >= ins["parede_min"],
            f"inserto M3 na coroa com parede ≥ {ins['parede_min']} mm")
    confere(ins["L"] + ins["fundo_extra"] <= pc["z1"] - pc["z0"] - 1.0, "furo do inserto cabe na coroa com 1 mm de fundo")
    pp = pan["pinhao"]
    confere(entre - pp["cubo_r"] > ga["r1"] + 2.0, f"cubo do pinhão (r {pp['cubo_r']}) longe das garras: {entre - pp['cubo_r'] - ga['r1']:.1f} mm")
    confere(K["volante_ref"]["d_faixa"][0] <= E["volante_d"] <= K["volante_ref"]["d_faixa"][1], f"volante Ø{E['volante_d']} dentro da faixa de projetos abertos {K['volante_ref']['d_faixa']}")
    confere(0 < berco["empurrador"]["curso"] and berco["empurrador"]["giro_deg"] < 180, f"curso do empurrador {berco['empurrador']['curso']} mm = {berco['empurrador']['giro_deg']:.0f}° do SG90 (< 180°)")
    vo_ = berco["volante"]
    toque_ = vo_["x"] - math.sqrt(vo_["r"] ** 2 - (vo_["y"] - d / 2) ** 2)
    ponta_fim = berco["calha"]["x0"] + 0.5 + L + berco["empurrador"]["curso"] - 1.0
    confere(ponta_fim >= toque_ + 1.0, f"o empurrão leva a ponta do dardo a x {ponta_fim:.2f}, {ponta_fim - toque_:+.2f} mm do toque nos volantes "
            f"(x {toque_:.2f}; tem de passar, senão o dardo não sai: achado no Gazebo em 2026-11-10)")
    confere(berco["webcam"]["suporte_x1"] <= berco["gaiola"]["x1"], f"suporte da webcam (até x {berco['webcam']['suporte_x1']}) cabe na tampa da gaiola (até x {berco['gaiola']['x1']})")
    confere(folga_vol < d, f"vão entre volantes {folga_vol:.1f} mm < dardo {d} mm (compressão {comp} mm)")
    mesa_x, mesa_y, mesa_z = K["impressora"]["mesa"]
    def cabe(nome, a_, b_, c_):
        dims = sorted([a_, b_, c_])
        ok = any(sorted(p)[0] >= dims[0] and sorted(p)[1] >= dims[1] and sorted(p)[2] >= dims[2]
                 for p in [(mesa_x, mesa_y, mesa_z)])
        confere(ok, f"{nome} {a_:.0f}×{b_:.0f}×{c_:.0f} cabe na mesa {mesa_x}×{mesa_y}×{mesa_z}")
    cabe("base", 140, 140, E["base_alt"])
    cabe("mesa giratória", pan["mesa"]["w"], pan["mesa"]["d"], 6)
    cabe("piso do berço", berco["piso"]["x1"] - berco["piso"]["x0"], 2 * berco["piso"]["y"], 3)
    cabe("braço", 40, 6, V["bracos"]["topo"] - V["bracos"]["z0"])
    confere(V["bracos"]["topo"] < 160 + pan["mesa"]["z1"], "braço com altura imprimível")

    assumidos = [k_ for k_, o in ORIGEM.items() if o.startswith("ASSUMIDO")]
    print(f"  info  cotas ASSUMIDAS (sem datasheet): {', '.join(assumidos) or 'nenhuma'}")
    d_ = V["disparo"]
    print("\nFísica do disparo:")
    for nome, x in d_.items():
        if nome.startswith("trajetoria"):
            continue
        v = x["v"]
        txt = (f"{v[0]:.3g}–{v[1]:.3g}" if isinstance(v, list) else f"{v:.4g}")
        print(f"  {nome:28s} {txt:>14s} {x['un']:7s} [{x['tipo']}] {x['de']}")
    print("  trajetória (tiro na horizontal):")
    for x_, z_, t_, v_ in d_["trajetoria_horizontal"]["v"]:
        print(f"    a {x_:.0f} m: queda {abs(z_)*100:5.1f} cm · {t_*1000:5.0f} ms · {v_:4.1f} m/s")
    print("\nOpção 2 — dardo impresso (rígido, ponta de TPU):")
    for nome, x in V["disparo_impresso"].items():
        v = x["v"]
        txt = (f"{v[0]:.3g}–{v[1]:.3g}" if isinstance(v, list) else f"{v:.4g}")
        print(f"  {nome:28s} {txt:>14s} {x['un']:7s} [{x['tipo']}] {x['de']}")
    confere(d_["vel_saida"]["v"] > 15, f"velocidade de saída {d_['vel_saida']['v']:.1f} m/s > 15 m/s (dardo útil a 3 m)")
    confere(d_["forca_empurrador"]["v"] > d_["forca_no_dardo"]["v"][1] / 5, "empurrador com folga de força (só leva o dardo até os volantes)")
    V["origem"] = ORIGEM
    with open(os.path.join(AQUI, "parametros.json"), "w", encoding="utf-8") as f:
        json.dump(V, f, ensure_ascii=False, indent=1)
    print(f"\neixo do tilt a {zt} mm do tampo · ponto mais baixo do berço {baixo:.1f} mm abaixo do eixo")
    print(f"escrito parametros.json · {len(ERROS)} falha(s)")
    return 1 if ERROS else 0


if __name__ == "__main__":
    sys.exit(main())
