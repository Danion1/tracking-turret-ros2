#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
engrenagem.py
=============
Perfil evolvente de engrenagem de dentes retos (externa) e de cremalheira,
como contorno 2D fechado, mais as contas de dimensionamento (dimensões,
Lewis). A peça (cubo, furos, garras) fica no script de cada projeto.

Quando usar: qualquer pinhão, coroa externa ou cremalheira impressa.
Entradas: módulo m (mm), z dentes, `folga_flanco` (mm de arco no primitivo
tirados de CADA flanco: a espessura do dente cai 2·folga_flanco; num par,
a folga total na primitiva é a soma das duas engrenagens).
Saída: lista de pontos (x, y) para `polyline(...).close().extrude(b)`.
Validação: testar_engrenagem.py (contas à mão, perfil lido do sólido,
engrenamento com cremalheira e teste negativo). Num par engrenagem ×
engrenagem, o script do projeto confere interferência zero girando o par.

Cuidados:
- Ângulo de pressão fixo em 20°, dente normal (adendo m, dedendo 1,25 m).
  Sem correção de perfil: z >= 18 para não rebaixar o pé (Z_MIN_SEM_REBAIXO).
- O nome `perfil_pinhao` vale para qualquer z (uma coroa de 40 também).
- Lewis ignora concentração no pé e dinâmica: FS mínimo 3 sobre o
  escoamento (ASSUMIDO no Projeto 2).

Origem: Projeto 2 (garra, 2026-10-13, cad/engrenagem.py); entrou na
biblioteca em 2026-11-06 no 2º uso real (torreta: pinhão 20T + coroa 40T,
m 1,5). Mudança: sem o bloco __main__ (dependia de um módulo do Projeto 2).
O Projeto 2 continua com a cópia própria (está publicado); migrar quando
ele for mexido de novo.

Por que evolvente: é a curva que mantém a força de contato sempre na mesma
direção (ângulo de pressão) enquanto o dente gira, então a cremalheira anda
em velocidade constante para velocidade constante do servo. Ela é gerada
"desenrolando um barbante" de um círculo de base de raio rb:
    x(t) = rb·(cos t + t·sen t),  y(t) = rb·(sen t - t·cos t)
O flanco reto da cremalheira é o caso limite de uma evolvente de raio infinito.

Grandezas (todas em mm, ângulo de pressão 20°, dente normal):
    diâmetro primitivo  d  = m·z,  de base db = d·cos(20°)
    adendo m, dedendo 1,25·m -> d_ext = d + 2m, d_raiz = d - 2,5m
    passo p = pi·m; entre-eixos de um par = m·(z1 + z2)/2

Resistência do dente (Lewis): sigma = F / (b·m·Y), F tangencial no primitivo.
Tabela de Y para 20°, dente normal: Shigley, "Mechanical Engineering
Design", Tabela 14-2 (cópia não guardada; segunda fonte PENDENTE).
"""


import math

ANG_PRESSAO = math.radians(20.0)
Z_MIN_SEM_REBAIXO = math.ceil(2.0 / math.sin(ANG_PRESSAO) ** 2)   # 18 (17,1 arredondado para cima)

# Shigley Tabela 14-2: fator de forma de Lewis Y, 20°, dente normal
LEWIS_Y = {12: 0.245, 13: 0.261, 14: 0.277, 15: 0.290, 16: 0.296, 17: 0.303, 18: 0.309,
           19: 0.314, 20: 0.322, 21: 0.328, 22: 0.331, 24: 0.337, 26: 0.346, 28: 0.353,
           30: 0.359, 34: 0.371, 38: 0.384, 43: 0.397, 50: 0.409, 60: 0.422}
LEWIS_Y_CREMALHEIRA = 0.485   # mesma tabela, linha "rack"


def lewis_y(z):
    """Y da tabela; entre linhas, o da linha de BAIXO (menor Y = conservador)."""
    if z < min(LEWIS_Y):
        raise ValueError(f"z = {z}: fora da tabela de Lewis (mínimo {min(LEWIS_Y)})")
    return LEWIS_Y[max(k for k in LEWIS_Y if k <= z)]


def dimensoes(m, z):
    d = m * z
    return {"m": m, "z": z, "d": d, "db": d * math.cos(ANG_PRESSAO), "d_ext": d + 2 * m,
            "d_raiz": d - 2.5 * m, "passo": math.pi * m, "mm_por_rad": d / 2.0}


def tensao_dente(torque_nmm, m, z, b, n_cremalheiras=2):
    """MPa no pé do dente do PINHÃO (Y do pinhão < Y da cremalheira, ele é o
    crítico). O torque se divide entre as cremalheiras que ele empurra."""
    f = torque_nmm / (m * z / 2.0) / n_cremalheiras
    return f, f / (b * m * lewis_y(z))


def _evolvente(rb, t):
    return (rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t)))


def perfil_pinhao(m, z, folga_flanco=0.0, n=12):
    """Pontos (x, y) do contorno fechado do pinhão, centro na origem, dente 0
    centrado no eixo +x. `folga_flanco` (mm) afina cada flanco no primitivo
    (folga de impressão, ASSUMIDA por quem chama)."""
    g = dimensoes(m, z)
    rb, ra, rf, r = g["db"] / 2, g["d_ext"] / 2, g["d_raiz"] / 2, g["d"] / 2
    inv = lambda a: math.tan(a) - a                       # função evolvente
    # meia espessura angular do dente no primitivo, menos a folga
    meia = (math.pi / (2 * z)) - folga_flanco / r
    # ângulo polar do ponto da evolvente no primitivo
    a_prim = math.acos(rb / r)
    deslocamento = meia + inv(a_prim)                     # gira a evolvente para o flanco cair certo
    t_max = math.sqrt((ra / rb) ** 2 - 1)
    t_min = math.sqrt(max((rf / rb) ** 2 - 1, 0.0))
    flanco = []
    for i in range(n + 1):
        t = t_min + (t_max - t_min) * i / n
        x, y = _evolvente(rb, t)
        ang = math.atan2(y, x)
        rr = math.hypot(x, y)
        flanco.append((rr, deslocamento - ang))           # (raio, ângulo) do flanco de CIMA
    pts = []
    passo = 2 * math.pi / z
    for k in range(z):
        c = k * passo
        if rf < rb:                                       # trecho radial da raiz até a base
            pts.append((rf * math.cos(c - deslocamento), rf * math.sin(c - deslocamento)))
        for rr, a in flanco:                              # flanco de baixo (subindo)
            pts.append((rr * math.cos(c - a), rr * math.sin(c - a)))
        for rr, a in reversed(flanco):                    # flanco de cima (descendo)
            pts.append((rr * math.cos(c + a), rr * math.sin(c + a)))
        if rf < rb:
            pts.append((rf * math.cos(c + deslocamento), rf * math.sin(c + deslocamento)))
    return pts


def perfil_cremalheira(m, n_dentes, altura_corpo, folga_flanco=0.0):
    """Contorno fechado da cremalheira: linha primitiva em y = 0, dentes para
    +y (adendo m), corpo para -y (dedendo 1,25m + altura_corpo). Dente 0 com
    o eixo em x = 0."""
    p = math.pi * m
    tg = math.tan(ANG_PRESSAO)
    meia = p / 4 - folga_flanco                           # meia espessura do dente no primitivo
    ha, hf = m, 1.25 * m
    pts = [(-p / 2, -hf - altura_corpo)]
    for k in range(n_dentes):
        c = k * p
        pts += [(c - p / 2, -hf), (c - meia - hf * tg, -hf), (c - meia + ha * tg, ha),
                (c + meia - ha * tg, ha), (c + meia + hf * tg, -hf)]
    pts += [((n_dentes - 1) * p + p / 2, -hf), ((n_dentes - 1) * p + p / 2, -hf - altura_corpo)]
    return pts
