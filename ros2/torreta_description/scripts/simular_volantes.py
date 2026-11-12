#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simular_volantes.py — SÓ SIMULAÇÃO: faz o papel do atrito volante-espuma, que o Gazebo não simula.

Não é código do robô (no hardware, quem lança o dardo é a física): é uma peça do mundo simulado,
como o plugin do Gazebo. O nó de comando de verdade é o do Daniel (torreta_nos/comando_torreta).

O que faz:
  - lê a pose do dardo solto (/model/dardo/pose, 1 kHz) e a do berço (/model/torreta/pose);
  - leva o dardo para o frame do berço: quando a ponta dele chega ao ponto em que a borda dos
    volantes encosta num cilindro de Ø13 (x_toque, a mesma conta do gerar_parametros.py),
    o dardo foi "pego";
  - lê a velocidade REAL do volante_motor no /joint_states (se o volante estiver devagar, o dardo
    sai devagar; parado, o dardo fica parado na entrada);
  - dá ao dardo, num passo de 1 ms, a força que leva a velocidade dele ao longo do cano até a da
    borda do volante: F = m · (ω·r − v_atual) / Δt  (/world/torreta/wrench, sistema ApplyLinkWrench).

Simplificações declaradas (ficha, seção 7): sem escorregamento nem perda na espuma (a borda é o
limite de cima da velocidade de saída); o volante não perde rotação ao lançar (½Iω² não é tirada
dele); sem arrasto do ar no voo. Um dardo por simulação. Quando o dardo lançado toca o chão, ele é
TIRADO do mundo (/world/torreta/remove): sem arrasto ele cai a dezenas de metros e quica a ~20 m/s,
e o detector de colisão do Gazebo estoura (ODE "aabbBound", medido em 2026-11-10 com tilt 20°).

Desde 2026-11-11, no rastreio: quando o centro do dardo lançado entra na caixa do corpo do drone (/alvo/verdade), o
dardo PARA ali (força que zera a velocidade num passo) e cai; é o "acertou" visível no vídeo.

Uso (o launch já roda):  python3 simular_volantes.py
"""
import json
import os
import threading

import numpy as np
import rclpy
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

PASSO_S = 0.001            # max_step_size do worlds/torreta.sdf: a força do ApplyLinkWrench vale um passo
V_MINIMA = 0.5             # m/s: abaixo disso o volante não lança (o dardo fica parado na entrada)
Z_CHAO = 0.008             # m: centro do dardo deitado no chão fica a 6,4 mm
# meia-caixa do corpo do drone (urdf/alvo.sdf: 60 × 60 × 25 mm) + raio do dardo (6,4 mm): o centro do dardo dentro dela = toque
CORPO_DRONE_MEIO = (0.030 + 0.0064, 0.030 + 0.0064, 0.0125 + 0.0064)

CFG = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "dardo.json"), encoding="utf-8"))


def matriz(p):
    q = p.orientation
    w, x, y, z = q.w, q.x, q.y, q.z
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return r, np.array([p.position.x, p.position.y, p.position.z])


def tempo(msg):
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


class SimularVolantes(Node):
    def __init__(self):
        super().__init__("simular_volantes")
        self.trava = threading.Lock()
        self.omega = 0.0
        self.berco = None                  # (R, p) no mundo
        self.anterior = None               # (t, p) do dardo
        self.lancado = None                # t do lançamento
        self.removido = False
        self.avisou_parado = False
        self.alvo = None                   # centro do drone no mundo (/alvo/verdade; só no rastreio)
        self.parou = False
        self.create_subscription(PointStamped, "/alvo/verdade", self.ao_alvo, 10)
        self.create_subscription(JointState, "/joint_states", self.ao_joint_states, 10)
        self.gz = GzNode()
        topico = f"/world/{CFG['mundo']}/wrench"
        self.pub = self.gz.advertise(topico, EntityWrench)
        self.gz.subscribe(Pose_V, "/model/torreta/pose", self.ao_berco)
        self.gz.subscribe(Pose, "/model/dardo/pose", self.ao_dardo)
        self.get_logger().info(f"pronto: toque em x = {CFG['x_toque_m'] * 1000:.2f} mm do berço; força em {topico}")

    def ao_joint_states(self, msg):
        v = dict(zip(msg.name, msg.velocity))
        if "volante_motor" in v:
            with self.trava:
                self.omega = v["volante_motor"]

    def ao_alvo(self, msg):
        with self.trava:
            self.alvo = np.array([msg.point.x, msg.point.y, msg.point.z])

    def ao_berco(self, msg):
        for p in msg.pose:
            if p.name == "berco" or p.name.endswith("::berco"):
                with self.trava:
                    self.berco = matriz(p)
                return

    def ao_dardo(self, msg):
        t = tempo(msg)
        _, p = matriz(msg)
        with self.trava:
            berco, omega, ant = self.berco, self.omega, self.anterior
            self.anterior = (t, p)
            lancado = self.lancado
            alvo = self.alvo
            if lancado is not None:
                if (not self.parou and alvo is not None and ant is not None and t > ant[0]
                        and all(abs(c) <= m for c, m in zip(p - alvo, CORPO_DRONE_MEIO))):
                    self.parou = True
                    threading.Thread(target=self.parar_no_drone, args=(t, p, (p - ant[1]) / (t - ant[0]), p - alvo),
                                     daemon=True).start()
                if not self.removido and p[2] < Z_CHAO and t > lancado + 0.05:
                    self.removido = True
                    threading.Thread(target=self.remover, args=(t, p), daemon=True).start()
                return
            if berco is None or ant is None or t <= ant[0]:
                return
        Rb, pb = berco
        eixo = Rb[:, 0]                                    # o cano: +x do berço, no mundo
        local = Rb.T @ (p - pb)                            # centro do dardo no frame do berço
        ponta = local[0] + CFG["comprimento_m"] / 2
        if ponta < CFG["x_toque_m"] or abs(local[1]) > 0.003:
            return
        v_borda = abs(omega) * CFG["raio_volante_m"]
        if v_borda < V_MINIMA:
            if not self.avisou_parado:
                self.get_logger().warn(f"dardo na entrada dos volantes, mas a borda está a {v_borda:.2f} m/s: não lança")
                self.avisou_parado = True
            return
        v_atual = float(eixo @ (p - ant[1]) / (t - ant[0]))
        forca = CFG["massa_kg"] * (v_borda - v_atual) / PASSO_S * eixo
        m = EntityWrench()
        m.entity.name = CFG["link"]
        m.entity.type = Entity.LINK
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = (float(c) for c in forca)
        self.pub.publish(m)
        with self.trava:
            self.lancado = t
        self.get_logger().info(
            f"LANÇADO em t = {t:.3f} s: ponta em x = {ponta * 1000:.2f} mm; volante {omega:.1f} rad/s → borda {v_borda:.2f} m/s; "
            f"dardo vinha a {v_atual:.3f} m/s; força {np.linalg.norm(forca):.2f} N por {PASSO_S * 1000:.0f} ms "
            f"(eixo {eixo[0]:+.3f} {eixo[1]:+.3f} {eixo[2]:+.3f})")


    def parar_no_drone(self, t, p, v, rel):
        """O dardo de espuma bate no corpo do drone e PARA (cai logo abaixo dele): força que zera a velocidade num
        passo, como no lançamento. Colisão física no drone foi tentada (2026-11-11): o dardo ricocheteou a 11,9 m e o
        ODE do Gazebo estourou (aabbBound); por isso o toque é decidido aqui, pela caixa do corpo."""
        forca = -CFG["massa_kg"] * v / PASSO_S
        m = EntityWrench()
        m.entity.name = CFG["link"]
        m.entity.type = Entity.LINK
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = (float(c) for c in forca)
        self.pub.publish(m)
        self.get_logger().info(f"ACERTOU o drone em t = {t:.3f} s: dardo a ({rel[0] * 1000:+.0f}, {rel[1] * 1000:+.0f}, "
                               f"{rel[2] * 1000:+.0f}) mm do centro, vinha a {np.linalg.norm(v):.1f} m/s; parado no corpo")

    def remover(self, t, p):
        m = Entity()
        m.name = "dardo"
        m.type = Entity.MODEL
        ok, rep = self.gz.request(f"/world/{CFG['mundo']}/remove", m, Entity, Boolean, 2000)
        # A resposta do serviço pode não chegar no prazo (visto em 2026-11-10) mesmo com o dardo removido:
        # a prova é a pose dele parar de chegar em /model/dardo/pose.
        self.get_logger().info(f"dardo no chão em t = {t:.3f} s, ({p[0]:.2f}, {p[1]:.2f}) m: pedido para tirar do mundo "
                               f"(resposta do serviço: {'confirmou' if ok and rep.data else 'não confirmou no prazo'})")


def main():
    rclpy.init()
    no = SimularVolantes()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
