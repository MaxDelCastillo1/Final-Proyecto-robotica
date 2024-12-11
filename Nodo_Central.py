import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray
import math

class Central(Node):
    def __init__(self):
        super().__init__('CentralNode')

        # Suscripciones
        self.yaw_sub = self.create_subscription(
            Float32,
            'mpu6050/yaw',
            self.yaw_callback,
            10
        )

        self.distance_sub = self.create_subscription(
            Float32MultiArray,
            'aruco',
            self.distance_callback,
            10
        )

        # Publicación
        self.publisher = self.create_publisher(
            Int16MultiArray,
            'cmd_vel',
            10
        )

        # Timer para control periódico
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # Estados
        self.state = 'IDLE'  # Estados: IDLE, MOVE_FORWARD, TURN, FINISHED

        # Variables del estado del robot
        self.current_yaw = 0.0
        self.marker_id = None
        self.marker_distance = None

        # Variables adicionales para giro
        self.yaw_objetivo = None
        self.yaw_tolerance = 5.0  # Tolerancia en grados

        # Velocidades predefinidas
        self.VELOCIDAD_AVANCE = 60    # Velocidad para avanzar
        self.VELOCIDAD_GIRO = 60      # Velocidad para girar
        self.VELOCIDAD_STOP = 0       # Velocidad para detenerse

        # IDs válidos de ArUco
        self.valid_ids = [0, 1, 2, 3, 4]

    def yaw_callback(self, msg):
        self.current_yaw = msg.data
        self.get_logger().debug(f"Yaw actual: {self.current_yaw} grados")

    def distance_callback(self, msg):
        if len(msg.data) >= 2:
            self.marker_id = int(msg.data[0])
            self.marker_distance = msg.data[1]

            if self.marker_id in self.valid_ids:
                # Loguear ID y distancia
                self.get_logger().info(f"ID: {self.marker_id}, Distancia: {self.marker_distance}cm")

                if self.state == 'IDLE':
                    self.state = 'MOVE_FORWARD'
                    self.get_logger().info("Comenzando a avanzar hacia el marcador.")
            else:
                # Loguear ID y distancia, y detener el robot
                self.get_logger().warn(f"Aruco detectado con ID inválido: {self.marker_id}. Ignorando.")
                self.stop_robot()
                self.state = 'IDLE'
        else:
            self.get_logger().warn("Mensaje de ArUco incompleto.")

    def control_loop(self):
        if self.state == 'IDLE':
            self.stop_robot()

        elif self.state == 'MOVE_FORWARD':
            if self.marker_distance > 20:
                # Avanzar hacia el marcador con velocidades predefinidas
                self.publish_velocity(self.VELOCIDAD_AVANCE, -70)  # [70, -80]
            else:
                # Detenerse y cambiar al estado 'TURN' inmediatamente
                self.stop_robot()
                self.set_turn_target()
                self.state = 'TURN'

        elif self.state == 'TURN':
            # Realizar el giro hacia el yaw objetivo
            self.perform_turn()

        elif self.state == 'FINISHED':
            # Giro completado, detenerse y volver a IDLE
            self.stop_robot()
            self.state = 'IDLE'

    def set_turn_target(self):
        """Configura el yaw objetivo basado en el ID del marcador."""
        self.initial_yaw = self.current_yaw
        if self.marker_id in [0, 3]:
            # Girar a la izquierda 90 grados
            self.yaw_objetivo = (self.initial_yaw + 90) % 360
            self.get_logger().info(f"Girando 90 grados a la izquierda. Yaw objetivo: {self.yaw_objetivo} grados.")
        elif self.marker_id == 1:
            # Girar a la derecha 90 grados
            self.yaw_objetivo = (self.initial_yaw - 90) % 360
            self.get_logger().info(f"Girando 90 grados a la derecha. Yaw objetivo: {self.yaw_objetivo} grados.")
        elif self.marker_id in [2, 4]:
            # Girar 180 grados
            self.yaw_objetivo = (self.initial_yaw + 180) % 360
            self.get_logger().info(f"Girando 180 grados. Yaw objetivo: {self.yaw_objetivo} grados.")
        else:
            # Por seguridad, no debería llegar aquí
            self.yaw_objetivo = self.initial_yaw
            self.get_logger().warn("Yaw objetivo no configurado correctamente.")

    def perform_turn(self):
        """Publica las velocidades para realizar el giro y verifica si se ha completado."""
        diff = self.angle_difference(self.yaw_objetivo, self.current_yaw)

        if abs(diff) <= self.yaw_tolerance:
            # Alcanzó el yaw objetivo dentro de la tolerancia
            self.stop_robot()
            # Usar nivel INFO para que el mensaje se muestre en color estándar (puede cambiar a otro nivel si se prefiere)
            self.get_logger().info(f"Ángulo alcanzado: {self.current_yaw:.2f} grados.")
            self.state = 'FINISHED'
        else:
            if diff > 0:
                # Girar a la izquierda: [-70, -70]
                self.publish_velocity(60, 60)
            else:
                # Girar a la derecha: [70, 70]
                self.publish_velocity(-60, -60)

    def angle_difference(self, target, current):
        """Calcula la diferencia de ángulo en grados, manejando la envolvente."""
        diff = (target - current + 180) % 360 - 180
        return diff

    def publish_velocity(self, left, right):
        """Publica las velocidades de las ruedas en formato Int16MultiArray."""
        vel_msg = Int16MultiArray()
        vel_msg.data = [left, right]
        self.publisher.publish(vel_msg)
        self.get_logger().debug(f"Publicando cmd_vel: {vel_msg.data}")

    def stop_robot(self):
        """Publica comandos para detener el robot."""
        self.publish_velocity(0, 0)
        self.get_logger().debug("Robot detenido.")

def main(args=None):
    rclpy.init(args=args)
    central = Central()
    try:
        rclpy.spin(central)
    except KeyboardInterrupt:
        central.get_logger().info('Interrumpido por el usuario.')
    finally:
        central.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
