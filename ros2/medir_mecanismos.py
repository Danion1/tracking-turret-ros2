#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
medir_mecanismos.py — CONFERÊNCIA (não comanda nada): grava o /joint_states do Gazebo e mede
cada transmissão contra a junta do motor, na física (Bullet-Featherstone).

  gravar:   python3 medir_mecanismos.py gravar <arquivo.jsonl> <segundos>
  analisar: python3 medir_mecanismos.py analisar <arquivo.jsonl> <torque_gravidade_tilt_Nm>

Quem move as juntas durante a gravação é o testar_gazebo.sh, com `ros2 topic pub` na linha
de comando (o nó de comando de verdade é do Daniel).

Mede, junta a junta (amostra a amostra, ~100 Hz):
  razão e sinal:  pan / pan_motor = −0,5;  cremalheira / sg90 = 0,010 m/rad;
                  volante_espelho / volante_motor = −1 (posição E velocidade);
  atraso/salto:   o erro |movida − mult × motora| em CADA amostra (se a física deixasse a
                  movida atrasar ou escorregar, o erro cresceria durante o movimento);
                  o maior passo entre amostras seguidas da movida contra o da motora;
  esforço:        o tilt parado na pose zero contra o torque de gravidade das massas do URDF.
"""

import json
import sys
import time

PARES = [("pan", "pan_motor", -0.5, "rad"), ("cremalheira", "sg90", 0.010, "m"), ("volante_espelho", "volante_motor", -1.0, "rad")]


def gravar(arq, dur):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    no = Node("medir_mecanismos")
    f = open(arq, "w")

    def cb(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        f.write(json.dumps({"t": t, "p": dict(zip(m.name, m.position)), "v": dict(zip(m.name, m.velocity)),
                            "e": dict(zip(m.name, m.effort))}) + "\n")

    no.create_subscription(JointState, "/joint_states", cb, 100)
    fim = time.time() + dur
    try:
        while time.time() < fim:
            rclpy.spin_once(no, timeout_sec=0.1)
    finally:
        f.close()
        no.destroy_node()
        rclpy.try_shutdown()


def analisar(arq, tau_g):
    A = [json.loads(l) for l in open(arq)]
    A = [a for a in A if all(k in a["p"] for k in ("pan", "pan_motor", "tilt", "sg90", "cremalheira", "volante_motor", "volante_espelho"))]
    print(f"  {len(A)} amostras em {A[-1]['t'] - A[0]['t']:.1f} s de simulação ({len(A) / max(A[-1]['t'] - A[0]['t'], 1e-9):.0f} Hz)")
    ok = True

    def conf(nome, cond, det):
        nonlocal ok
        print(f"  [{'OK' if cond else 'FALHOU'}] {nome}: {det}")
        ok &= bool(cond)

    dt = (A[-1]["t"] - A[0]["t"]) / (len(A) - 1)
    for movida, motora, mult, un in PARES:
        curso_m = max(abs(a["p"][motora]) for a in A)
        if curso_m < 0.02:
            conf(f"{movida} × {motora}", False, "a motora não se moveu")
            continue
        # 1) atraso: o deslocamento d (amostras) que melhor alinha movida(t) com mult × motora(t − d)
        def erro_com(d):
            return max(abs(A[i]["p"][movida] - mult * A[i - d]["p"][motora]) for i in range(d, len(A)))
        erros = {d: erro_com(d) for d in range(0, 6)}
        d_ok = min(erros, key=erros.get)
        # 2) razão e sinal em REGIME: amostras com a motora parada e longe de zero
        if motora == "volante_motor":         # o volante não para: regime = velocidade constante, razão das velocidades
            reg = [A[i] for i in range(1, len(A)) if abs(A[i]["v"][motora]) > 100 and abs(A[i]["v"][motora] - A[i - 1]["v"][motora]) < 1e-3]
            rz = [a["v"][movida] / a["v"][motora] for a in reg]
            if reg:
                print(f"  (volante em regime: {reg[0]['v'][motora]:+.1f} rad/s no motor, {reg[0]['v'][movida]:+.1f} no espelho)")
        else:
            reg = [a for a in A if abs(a["v"].get(motora, 1)) < 1e-3 and abs(a["p"][motora]) > 0.05]
            rz = [a["p"][movida] / a["p"][motora] for a in reg]
        # 3) salto: o maior passo da movida contra o da motora (× |mult|)
        pm = max(abs(A[i + 1]["p"][movida] - A[i]["p"][movida]) for i in range(len(A) - 1))
        pmo = max(abs(A[i + 1]["p"][motora] - A[i]["p"][motora]) for i in range(len(A) - 1)) * abs(mult)
        escala = abs(mult) * curso_m
        conf(f"{movida} × {motora}: razão e sinal em regime",
             rz and max(abs(r - mult) for r in rz) < 1e-4 * abs(mult),
             f"{min(rz):+.6f}..{max(rz):+.6f} (esperado {mult:+.3f}) em {len(rz)} amostras em regime" if rz else "sem trecho parado")
        conf(f"{movida} × {motora}: atraso de no máximo 1 amostra e nada além dele",
             d_ok <= 1 and erros[d_ok] < 2e-3 * escala,
             f"atraso {d_ok} amostra(s) = {d_ok * dt * 1000:.1f} ms; erro sem alinhar {erros[0]:.2e} {un}, alinhado {erros[d_ok]:.2e} {un} "
             f"({erros[d_ok] / escala * 100:.3f} % do curso)")
        conf(f"{movida} × {motora}: sem salto", pm <= pmo * 1.05 + 1e-9, f"maior passo da movida {pm:.5f} contra {pmo:.5f} da motora × |mult|")
    # comando chegou? (a motora alcançou o alvo)
    fim = A[-1]["p"]
    print(f"  pose final: pan_motor {fim['pan_motor']:+.4f}, pan {fim['pan']:+.4f}, tilt {fim['tilt']:+.4f}, sg90 {fim['sg90']:+.4f}, "
          f"cremalheira {fim['cremalheira'] * 1000:+.3f} mm")
    # esforço do tilt parado na pose zero (primeiro trecho, antes dos comandos)
    parado = [a for a in A[:200] if abs(a["p"]["tilt"]) < 0.01 and abs(a["v"].get("tilt", 1)) < 0.01 and a["e"].get("tilt") is not None]
    if parado:
        e = sum(a["e"]["tilt"] for a in parado) / len(parado)
        conf("esforço do tilt parado na pose zero = gravidade (massas do URDF)", abs(abs(e) - tau_g) < 0.05 * tau_g,
             f"medido {e:+.4f} N·m, calculado {tau_g:.4f} N·m ({len(parado)} amostras)")
    else:
        conf("esforço do tilt parado", False, "sem trecho parado na pose zero")
    return ok


if __name__ == "__main__":
    if sys.argv[1] == "gravar":
        gravar(sys.argv[2], float(sys.argv[3]))
    else:
        sys.exit(0 if analisar(sys.argv[2], float(sys.argv[3])) else 1)
