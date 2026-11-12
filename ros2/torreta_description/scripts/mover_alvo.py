#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mover_alvo.py — SÓ SIMULAÇÃO: faz o "drone" alvo (urdf/alvo.sdf) voar por uma trajetória conhecida.

Não é código do robô: é uma peça do mundo simulado (como o simular_volantes.py), que faz o papel do
drone do Daniel voando no quarto. Quem detecta, rastreia e mira é o nó do Daniel.

O que faz, a 50 Hz no relógio da SIMULAÇÃO:
  - calcula a posição do alvo no instante t (trajetória abaixo) e o põe lá (/world/torreta/set_pose);
  - publica a posição VERDADEIRA em /alvo/verdade (geometry_msgs/PointStamped, frame "world"),
    para medir o erro da detecção e do rastreio. O nó de rastreio NÃO deve ler este tópico:
    no quarto não existe "verdade", só a imagem da webcam.

Trajetórias (parâmetro `modo`), centro C = (distancia, 0, altura) no frame world (a torreta está na origem,
olhando para +x com pan 0):
  parado   fica em C
  vaivem   y = A·sin(2πt/T)                         (de um lado para o outro)
  circulo  y = R·sin(2πt/T), z = altura − R + R·cos(2πt/T)   (R = amplitude/2, plano y-z; começa em C, no topo)
  oito     y = A·sin(2πt/T), z = altura + 0,3·A·sin(4πt/T)
Velocidade máxima no vaivem: 2π·A/T (A 1,0 m, T 8 s: 0,79 m/s, ~ um drone pequeno devagar).

Parâmetros ROS: modo (vaivem), distancia (3.0 m), altura (0.8 m), amplitude (1.0 m), periodo (8.0 s),
espera (2.0 s parado em C antes de começar a se mover).

Uso (o launch/rastreio.launch.py já roda):  python3 mover_alvo.py --ros-args -p modo:=circulo
"""
import math
import threading

import rclpy
from geometry_msgs.msg import PointStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

MODOS = ("parado", "vaivem", "circulo", "oito")


def posicao(modo, t, d, h, a, periodo):
    """Posição (x, y, z) em metros, frame world, no instante t (s) desde o início do movimento."""
    if modo == "parado" or t <= 0.0:
        fase = 0.0
    else:
        fase = 2 * math.pi * t / periodo
    if modo == "vaivem":
        return d, a * math.sin(fase), h
    if modo == "circulo":
        r = a / 2
        return d, r * math.sin(fase), h + r * math.cos(fase) - r     # começa em C (cos 0 = 1)
    if modo == "oito":
        return d, a * math.sin(fase), h + 0.3 * a * math.sin(2 * fase)
    return d, 0.0, h


class MoverAlvo(Node):
    def __init__(self):
        super().__init__("mover_alvo", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.modo = self.declare_parameter("modo", "vaivem").value
        self.d = self.declare_parameter("distancia", 3.0).value
        self.h = self.declare_parameter("altura", 0.8).value
        self.a = self.declare_parameter("amplitude", 1.0).value
        self.periodo = self.declare_parameter("periodo", 8.0).value
        self.espera = self.declare_parameter("espera", 2.0).value
        if self.modo not in MODOS:
            raise SystemExit(f"modo '{self.modo}' não existe; use um destes: {MODOS}")
        self.pub = self.create_publisher(PointStamped, "/alvo/verdade", 10)
        self.gz = GzNode()
        self.t0 = None
        self.ocupado = threading.Lock()
        self.create_timer(0.02, self.passo)
        self.get_logger().info(f"alvo: modo {self.modo}, centro ({self.d}, 0, {self.h}) m, amplitude {self.a} m, "
                               f"período {self.periodo} s, começa depois de {self.espera} s")

    def passo(self):
        agora = self.get_clock().now()
        if agora.nanoseconds == 0:          # o /clock ainda não chegou
            return
        if self.t0 is None:
            self.t0 = agora.nanoseconds * 1e-9
        t = agora.nanoseconds * 1e-9 - self.t0 - self.espera
        x, y, z = posicao(self.modo, t, self.d, self.h, self.a, self.periodo)
        msg = PointStamped()
        msg.header.stamp = agora.to_msg()
        msg.header.frame_id = "world"
        msg.point.x, msg.point.y, msg.point.z = x, y, z
        self.pub.publish(msg)
        if self.ocupado.acquire(blocking=False):      # o serviço é bloqueante: um pedido de cada vez, fora do timer
            threading.Thread(target=self.por, args=(x, y, z), daemon=True).start()

    def por(self, x, y, z):
        try:
            p = Pose()
            p.name = "alvo"
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.w = 1.0
            self.gz.request("/world/torreta/set_pose", p, Pose, Boolean, 500)
        finally:
            self.ocupado.release()


def main():
    rclpy.init()
    no = MoverAlvo()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
