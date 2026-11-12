import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class SeguirAlvo(Node):
    def __init__(self):
        super().__init__('seguir_alvo')

        # Constantes fisicas e balisticas
        self.altura_camera = 0.0475 # metros (47.5mm acima do cano)
        self.vel_dardo = 21.6 # m/s
        self.g = 9.81 # m/s²
        self.largura_drone = 0.060 # metros

        # Parametros intrinsecos padrao(atualizados via CameraInfo)
        self.fx = 453.5
        self.fy = 453.5
        self.cx = 320.0
        self.cy = 180.0

        # Variaveis de estado da mira
        # Guardam a posição atual estimada dos eixos para somar o incremento de movimento
        self.torre_pan_atual_rad = 0.0
        self.tilt_atual_rad = 0.0

        # Ganhos do controlador proporcional
        self.ganho_pan = 0.0008
        self.ganho_tilt = 0.0008
        # (correção feita com assistência de IA, 2026-11-11) passo máximo por quadro: 0,02 rad = 0,6 rad/s a 30 Hz.
        # Sem isso, o erro grande da partida (92 px a 4 m) somava 0,074 rad por quadro, mais rápido que o servo:
        # o comando passava do ponto (windup), o drone saía pela borda e a torreta ficava presa olhando o céu.
        # O drone pede no máximo 0,26 rad/s (0,79 m/s a 3 m), então o limite não atrasa o rastreio.
        self.passo_max_rad = 0.02
        # distância plausível: o detector vê o corpo até ~5,7 m; largura de 3 px dava d = 9 m e o v* pulava
        self.d_min, self.d_max = 1.0, 6.0

        # Subscrições e publicações
        self.sub_cam_info = self.create_subscription(
            CameraInfo,
            '/webcam/camera_info',
            self.callback_cam_info,
            10
        )

        self.sub_pixel = self.create_subscription(
            PointStamped,
            '/alvo/pixel',
            self.callback_pixel,
            10
        )

        self.pub_mira = self.create_publisher(
            JointTrajectory,
            '/pan_tilt_controller/joint_trajectory',
            10
        )
            
        # (correção feita com assistência de IA, 2026-11-11) posição MEDIDA das juntas: o comando novo parte dela,
        # não do último comando. Somando ao último comando, o atraso câmera → detecção → servo fazia o
        # comando correr à frente da torreta (windup): o drone saía pela borda e a torreta ficava no céu.
        self.medido = {}
        self.sub_juntas = self.create_subscription(JointState, '/joint_states', self.callback_juntas, 10)
        # sem detecção por mais de 1 s: volta devagar para casa (pan 0, tilt 0) em vez de ficar presa
        self.ultima_deteccao = None
        self.tempo_perdido_s = 1.0
        self.voltando = False
        self.create_timer(0.2, self.verificar_perdido)

        self.get_logger().info("Nó 'seguir_alvo' iniciado. Pronto para rastreio contínuo!")

    def callback_juntas(self, msg):
        self.medido = dict(zip(msg.name, msg.position))

    def verificar_perdido(self):
        if self.ultima_deteccao is None or self.voltando:
            return
        if (self.get_clock().now() - self.ultima_deteccao).nanoseconds * 1e-9 > self.tempo_perdido_s:
            self.get_logger().warn("Alvo perdido há mais de 1 s: voltando para pan 0, tilt 0")
            self.torre_pan_atual_rad, self.tilt_atual_rad = 0.0, 0.0
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ["pan_motor", "tilt"]
            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.0]
            point.time_from_start = Duration(sec=2, nanosec=0)
            traj_msg.points = [point]
            self.pub_mira.publish(traj_msg)
            self.voltando = True

    def callback_cam_info(self, msg):
        # Atualiza dinamicamente os parametros intrinsecos da matriz K
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def callback_pixel(self, msg):
        # Extrai as coordenadas (u, v)
        u = msg.point.x
        v = msg.point.y
        largura_px = msg.point.z

        # Proteçao contra divisao por zero ou detecção espuria
        if largura_px < 2.0:
            return

        # Estima a distancia d usando o tamanho aparente do drone
        d = (self.fx * self.largura_drone) / largura_px
        d = max(self.d_min, min(self.d_max, d))

        # calcula o angulo de correção c
        c = (self.altura_camera / d) + (self.g * d) / (2.0 * (self.vel_dardo ** 2))

        # calcula o pixel-alvo vertical compensando v*
        v_star = self.cy + self.fy * math.tan(c)

        # Cálculo do erro
        erro_u = u - self.cx
        erro_v = v_star - v # o drone aparece abaixo do centro, porque o cano aponta acima dele.

        # Lei de controle proporcional calculada no espaço da torre
        delta_torre_pan = -erro_u * self.ganho_pan
        delta_tilt = erro_v * self.ganho_tilt
        delta_torre_pan = max(-self.passo_max_rad, min(self.passo_max_rad, delta_torre_pan))
        delta_tilt = max(-self.passo_max_rad, min(self.passo_max_rad, delta_tilt))

        # Acumula a posição no espaço lógico da torre
        self.ultima_deteccao = self.get_clock().now()
        self.voltando = False
        # parte da posição MEDIDA (torre "pan" e "tilt" do /joint_states); antes da 1ª leitura, do último comando
        if "pan" in self.medido and "tilt" in self.medido:
            self.torre_pan_atual_rad = self.medido["pan"]
            self.tilt_atual_rad = self.medido["tilt"]
        self.torre_pan_atual_rad += delta_torre_pan
        self.tilt_atual_rad += delta_tilt

        # Limites de segurança fisicos (Clamping)
        limite_torre_pan = math.radians(90.0)
        limite_tilt_min = math.radians(-20.0)
        limite_tilt_max = math.radians(40.0)

        self.torre_pan_atual_rad = max(-limite_torre_pan, min(limite_torre_pan, self.torre_pan_atual_rad))
        self.tilt_atual_rad = max(limite_tilt_min, min(limite_tilt_max, self.tilt_atual_rad))

        # Conversão cinematica para o motor
        # Aplica a mesma coroa (-2.0) usada no comando_torreta para traduzir torre
        motor_pan_rad = self.torre_pan_atual_rad * -2.0
        
        # montagem da trajetória de curtissimo prazo para fluidez continua 
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["pan_motor", "tilt"]

        point = JointTrajectoryPoint()
        # Manda o motor corrigido pela coroa e o tilt direto
        point.positions = [motor_pan_rad, self.tilt_atual_rad]

        # Importante para o movimento continuo
        # um tempo muito curto diz ao controlador para atualizar o motor imediatamente
        # impedindo o efeito de "parar e pensar" que engasgaria o rastreio continuo do alvo
        point.time_from_start = Duration(sec=0, nanosec=50000000)
        traj_msg.points = [point]

        # Publica o comando de movimento continuo
        self.pub_mira.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SeguirAlvo()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()