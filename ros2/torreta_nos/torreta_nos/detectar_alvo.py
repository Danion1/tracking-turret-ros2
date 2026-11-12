import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DetectarAlvo(Node):
    def __init__(self):
        super().__init__('detectar_alvo')

        # Conversor entre mensagens do ROS 2 e a imagens do OpenCV
        self.bridge = CvBridge()

        # Area minima em pixels para considerar que é o drone e ignorar ruidos de fundo
        # (correção feita com assistência de IA, 2026-11-11) mínimo em pixels CONTADOS; a 4 m o drone tem ~20 px.
        # Sem ruído no simulador; na webcam real, medir de novo (ruído pede mínimo maior).
        self.area_minima_px = 8.0

        # Subscrição da camera
        self.sub_imagem = self.create_subscription(
            Image,
            '/webcam/image_raw',
            self.callback_imagem,
            10
        )

        # Publicação da posição em pixels
        self.pub_pixel = self.create_publisher(
            PointStamped,
            '/alvo/pixel',
            10
        )

        self.get_logger().info("Nó 'detectar_alvo' iniciado e aguardando quadros do tópico /webcam/image_raw...")

    def callback_imagem(self, msg_imagem):
        # Converte a mensagem de imagem do ROS para matriz BGR do OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg_imagem, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Erro ao converter imagem via cv_bridge: {e}")
            return

        # Converte do espaço de cor BGR para HSV(para imunidade de variações de tom marrom)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Limiares HSV baseados nas medições: Tom ~5-32, Saturação > 50, Brilho > 25
        limite_inferior = np.array([5, 50, 25], dtype=np.uint8)
        limite_superior = np.array([32, 255, 255], dtype=np.uint8)

        mascara = cv2.inRange(hsv_image, limite_inferior, limite_superior)

        # Operação morfológica para limpar pequenos ruidos e imperfeições 
        # 2x2 (era 3x3): apaga as hélices (linhas de 1 px) mas mantém o corpo até ~2 px de altura,
        # ou seja, até d = 453,5 · 0,025 / 2 = 5,7 m (com 3x3 o limite era 3,8 m e o drone sumia a 4 m)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)

        # Encontra contornos no elemento filtrado
        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contornos:
            return # Drone nao detectado no quadro

        # Seleciona o maior contorno da cena
        maior_contorno = max(contornos, key=cv2.contourArea)
        # Área = pixels CONTADOS dentro da caixa do contorno. O contourArea mede o polígono que passa
        # pelos centros dos pixels da borda: ~(w-1)(h-1), e a 4 m dava 10 para 18 pixels.
        x, y, largura_px, altura_px = cv2.boundingRect(maior_contorno)
        area = cv2.countNonZero(mascara[y:y + altura_px, x:x + largura_px])

        # Filtra manchas insignificantes
        if area < self.area_minima_px:
            return

        # Calculo do centroide usando momentos de imagem
        M = cv2.moments(maior_contorno)
        if M["m00"] == 0:
            return

        u = float(M["m10"] / M["m00"])
        v = float(M["m01"] / M["m00"])

        msg_pixel = PointStamped()
        msg_pixel.header = msg_imagem.header
        msg_pixel.point.x = u
        msg_pixel.point.y = v
        msg_pixel.point.z = float(largura_px) # point z agora carrega a largura

        
        # Publica o resultada da detecção no topico que o 'seguir_alvo" vai escutor
        self.pub_pixel.publish(msg_pixel)

def main(args=None):
    rclpy.init(args=args)
    node = DetectarAlvo()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
