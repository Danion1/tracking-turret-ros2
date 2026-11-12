import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray, Empty
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class ComandoTorreta(Node):
    def __init__(self):
        super().__init__('comando_torreta')

        # Armazenam a posição e a velocidade atual de cada junta lida do /joint_states
        self.posicoes_juntas = {}
        self.velocidade_juntas = {}

        # Estados possiveis: "PARADO", "ACELERADO", "EMPURIANDO"(ou EMPURRANDO), "RECOLHENDO"
        self.estado = "PARADO"

        # Alvo de velocidade para os volantes
        self.vel_volante_alvo = 1200.0
        self.timeout_estado = 2.0
        # Giro do SG90 no empurrão (rad): curso de 15,5 mm / raio primitivo de 10 mm (2026-11-10: era 1.32,
        # e o dardo parava 0,36 mm antes dos volantes)
        self.giro_empurrar = 1.55

        # Variavel pro timeout
        # Guarda o objeto de tempo do ROS 2 de quando entramos no estado atual
        self.tempo_inicio_estado = None

        # Subscribers(Entradas)

        # Alvo de mira da torreta(Pan e Tilt)
        self.sub_alvo = self.create_subscription(
            Float64MultiArray,
            '/torreta/alvo',
            self.listener_alvo_callback,
            10
        )

        # Gatilho de disparo(Sinal vazio que inicia a sequencia)
        self.sub_disparar = self.create_subscription(
            Empty,
            '/torreta/disparar',
            self.listener_disparar_callback,
            10
        )

        # Escuta o feedback continuo(posição e velocidade reais)
        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_joint_states_callback,
            10
        )

        # Publishers(Saídas)

        # Controlador de trajetória da mira(Pan/Tilt)
        self.pub_mira = self.create_publisher(
            JointTrajectory,
            '/pan_tilt_controller/joint_trajectory',
            10
        )

        # Comando de velocidade direta para os volantes de lançamento
        self.pub_volantes = self.create_publisher(
            Float64MultiArray,
            '/volantes_controller/commands',
            10
        )

        # Controlador de trajetória do microservo de gatilho (SG90)
        self.pub_gatilho = self.create_publisher(
            JointTrajectory,
            '/gatilho_controller/joint_trajectory',
            10
        )

        # Timer da maquina de ESTADOS
        # Roda a função passo_disparo 50 vezes por segundo(a cada 0,02s) sem bloquear o nó
        self.timer = self.create_timer(0.02, self.passo_disparo)

    # Funções
    def listener_joint_states_callback(self, msg):
        # Mapeia listas paralelas (name -> position/velocity) em dicionarios seguros
        self.posicoes_juntas = dict(zip(msg.name, msg.position))
        self.velocidade_juntas = dict(zip(msg.name, msg.velocity))

    def listener_alvo_callback(self, msg):
        # Callback original de mira (Pan/Tilt)
        if len(msg.data) < 2:
            self.get_logger().warn("Mensagem de alvo recebida com dados insuficientes!")
            return

        pan_graus = msg.data[0]
        tilt_graus = msg.data[1]

        # Validação de limites físicos de segurança
        if not (-90.0 <= pan_graus <= 90.0):
            self.get_logger().warn(f"Pan fora do limite (-90° a +90°): {pan_graus}°")
            return
            
        if not (-20.0 <= tilt_graus <= 40.0):
            self.get_logger().warn(f"Tilt fora do limite (-20° a +40°): {tilt_graus}°")
            return

        # Conversão cinematic(Pan com redução de coroa -2:1 e inversão de sentido)
        pan_radianos = math.radians(pan_graus) * -2.0
        tilt_radianos = math.radians(tilt_graus)

        # Montagem e envio de trajetória de mira
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["pan_motor", "tilt"]

        point = JointTrajectoryPoint()
        point.positions = [pan_radianos, tilt_radianos]
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj_msg.points = [point]

        self.pub_mira.publish(traj_msg)
        self.get_logger().info(f"Mira ajustada -> Torre [Pan: {pan_graus}°, Tilt: {tilt_graus}°]")

    def listener_disparar_callback(self, msg):
        # O operador apertou o gatilho, so aceita se a torreta estiver ociosa.
        if self.estado == "PARADO":
            self.estado = "ACELERANDO"
            # Registrando o relogio exato da entrada no estado
            self.tempo_inicio_estado = self.get_clock().now()
            self.get_logger().info("=== DISPARO ACIONADO: Estado mudou para ACELERANDO ===")

            # comando enviado uma UNICA VEZ na entrada do estado: liga os volantes na velocidade alvo
            cmd_volante = Float64MultiArray()
            cmd_volante.data = [self.vel_volante_alvo]
            self.pub_volantes.publish(cmd_volante)
        else:
            self.get_logger().warn(f"Disparo ignorado! A torreta já está em execução (Estado atual: {self.estado})")

    def passo_disparo(self):
        # Maquina de estados executados 50x por segundo pelo timer

        if self.estado == "PARADO":
            return # Não faz nada enquanto estiver ocioso

        # Checagem global de timeout
        tempo_atual = self.get_clock().now()
        duracao_no_estado = (tempo_atual - self.tempo_inicio_estado).nanoseconds / 1e9

        if duracao_no_estado > self.timeout_estado:
            self.get_logger().error(f"FALHA CRITICA: Timeout de {self.timeout_estado}s estourado no estado '{self.estado}'! Mecanismo travado. Abortando.")

            # Força o reset de segurança para salvar o robô
            self.estado = "PARADO"
            cmd_volante = Float64MultiArray()
            cmd_volante.data = [0.0]
            self.pub_volantes.publish(cmd_volante)

            # Segurança mecanica: Manda o SG90 voltar imediatamente para 0 para desengatar o travamento
            traj_sg90 = JointTrajectory()
            traj_sg90.joint_names = ["sg90"]
            pt = JointTrajectoryPoint()
            pt.positions = [0.0]
            pt.time_from_start = Duration(sec=0, nanosec=300000000)
            traj_sg90.points = [pt]
            self.pub_gatilho.publish(traj_sg90)

            return

        # Maquina de estados comercial
        if self.estado == "ACELERANDO":
            # Lê a velocidade atual do moto do volante do dicionario de estados
            vel_atual = self.velocidade_juntas.get("volante_motor", 0.0)

            # Critério de avanço: chegou perto de 95% da velocidade pedida
            if vel_atual >= (self.vel_volante_alvo * 0.95):
                self.estado = "EMPURRANDO"
                self.tempo_inicio_estado = self.get_clock().now()
                self.get_logger().info(f"-> Volantes atingiram {vel_atual:.1f} rad/s. Estado: EMPURRANDO")

                # Comando enviado UNICA VEZ: manda o SG90 avançar até o giro do empurrão
                traj_sg90 = JointTrajectory()
                traj_sg90.joint_names = ["sg90"]
                pt = JointTrajectoryPoint()
                pt.positions = [self.giro_empurrar]
                pt.time_from_start = Duration(sec=0, nanosec=300000000) # 0.3s
                traj_sg90.points = [pt]
                self.pub_gatilho.publish(traj_sg90)

        elif self.estado == "EMPURRANDO":
            # lê a posição atual do eixo do SG90
            pos_atual = self.posicoes_juntas.get("sg90", 0.0)

            # Critério de avanço: o servo chegou perto do giro do empurrão (margem de tolerancia)
            if abs(pos_atual - self.giro_empurrar) < 0.05:
                self.estado = "RECOLHENDO"
                self.tempo_inicio_estado = self.get_clock().now()
                self.get_logger().info(f"-> SG90 empurrou (Pos: {pos_atual:.2f} rad). Estado: RECOLHENDO")

                # Comando enviado UNICA VEZ: manda o SG90 voltar para 0 rad
                traj_sg90 = JointTrajectory()
                traj_sg90.joint_names = ["sg90"]
                pt = JointTrajectoryPoint()
                pt.positions = [0.0]
                pt.time_from_start = Duration(sec=0, nanosec=300000000)
                traj_sg90.points = [pt]
                self.pub_gatilho.publish(traj_sg90)

        elif self.estado == "RECOLHENDO":
            # Lê a posição atual do eixo do SG90 de volta
            pos_atual = self.posicoes_juntas.get("sg90", 0.0)

            # Critério de avnaço: o servo retornou proximo de 0.0 rad
            if abs(pos_atual - 0.0) < 0.05:
                self.estado = "PARADO"
                self.get_logger().info("-> SG90 recolhido. Sequência finalizada. Desligando volantes. Estado: PARADO")

                # Desliga os volantes mandando velocidade zero
                cmd_volante = Float64MultiArray()
                cmd_volante.data = [0.0]
                self.pub_volantes.publish(cmd_volante)

def main(args=None):
    rclpy.init(args=args)
    node = ComandoTorreta()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
