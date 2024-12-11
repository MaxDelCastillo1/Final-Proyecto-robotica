import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import math
import time

class MPU6050YawPublisher(Node):
    def _init_(self):
        super()._init_('mpu6050_yaw_publisher')
        
        # Crear un publicador que publicará el valor de yaw
        self.yaw_publisher = self.create_publisher(Float32, 'mpu6050/yaw', 10)
        
        # Configuración del MPU-6050
        self.MPU_addr = 0x68
        self.A_cal = [265.0, -80.0, -700.0, 0.994, 1.000, 1.014]
        self.G_off = [-499.5, -17.7, -82.0]
        self.gscale = (250.0 / 32768.0) * (math.pi / 180.0)
        
        # Variables para el filtro Mahony
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.Kp = 30.0
        self.Ki = 0.0

        # Inicializar el bus I2C
        self.bus = smbus.SMBus(1)  # Para Raspberry Pi, el bus I2C es normalmente 1

        # Inicializar el MPU-6050
        self.init_mpu()

        # Variables para el cálculo del tiempo
        self.last_time = time.time()
        
        # Establecer la frecuencia de publicación
        self.timer = self.create_timer(0.2, self.timer_callback)  # Publica cada 200ms

    def read_word(self, register):
        high = self.bus.read_byte_data(self.MPU_addr, register)
        low = self.bus.read_byte_data(self.MPU_addr, register + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            val -= 0x10000
        return val

    def init_mpu(self):
        self.bus.write_byte_data(self.MPU_addr, 0x6B, 0)

    def Mahony_update(self, ax, ay, az, gx, gy, gz, deltat):
        recipNorm, vx, vy, vz, ex, ey, ez = 0, 0, 0, 0, 0, 0, 0
        ix, iy, iz = 0.0, 0.0, 0.0
        tmp = ax * ax + ay * ay + az * az
        if tmp > 0.0:
            recipNorm = 1.0 / math.sqrt(tmp)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            vx = self.q[1] * self.q[3] - self.q[0] * self.q[2]
            vy = self.q[0] * self.q[1] + self.q[2] * self.q[3]
            vz = self.q[0] * self.q[0] - 0.5 + self.q[3] * self.q[3]

            ex = (ay * vz - az * vy)
            ey = (az * vx - ax * vz)
            ez = (ax * vy - ay * vx)

            if self.Ki > 0.0:
                ix += self.Ki * ex * deltat
                iy += self.Ki * ey * deltat
                iz += self.Ki * ez * deltat
                gx += ix
                gy += iy
                gz += iz

            gx += self.Kp * ex
            gy += self.Kp * ey
            gz += self.Kp * ez

        deltat *= 0.5
        gx *= deltat
        gy *= deltat
        gz *= deltat
        qa = self.q[0]
        qb = self.q[1]
        qc = self.q[2]
        self.q[0] += (-qb * gx - qc * gy - self.q[3] * gz)
        self.q[1] += (qa * gx + qc * gz - self.q[3] * gy)
        self.q[2] += (qa * gy - qb * gz + self.q[3] * gx)
        self.q[3] += (qa * gz + qb * gy - qc * gx)

        recipNorm = 1.0 / math.sqrt(self.q[0] * self.q[0] + self.q[1] * self.q[1] + self.q[2] * self.q[2] + self.q[3] * self.q[3])
        self.q[0] *= recipNorm
        self.q[1] *= recipNorm
        self.q[2] *= recipNorm
        self.q[3] *= recipNorm

    def timer_callback(self):
        # Leer los datos de acelerómetro y giroscopio
        ax = self.read_word(0x3B)
        ay = self.read_word(0x3D)
        az = self.read_word(0x3F)
        gx = self.read_word(0x43)
        gy = self.read_word(0x45)
        gz = self.read_word(0x47)

        # Calibrar los datos
        Axyz = [(float(ax) - self.A_cal[i]) * self.A_cal[i + 3] for i, ax in enumerate([ax, ay, az])]
        Gxyz = [(float(gx) - self.G_off[i]) * self.gscale for i, gx in enumerate([gx, gy, gz])]

        # Obtener el tiempo actual y calcular el delta tiempo
        now_time = time.time()
        deltat = now_time - self.last_time
        self.last_time = now_time

        # Actualizar los quaterniones usando el filtro Mahony
        self.Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat)

        # Calcular el ángulo yaw
        yaw = -math.atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]), 1.0 - 2.0 * (self.q[2] * self.q[2] + self.q[3] * self.q[3]))

        # Convertir de radianes a grados
        yaw = yaw * 180.0 / math.pi
        if yaw < 0:
            yaw += 360.0

        # Publicar el valor de yaw
        msg = Float32()
        msg.data = yaw
        self.yaw_publisher.publish(msg)

        self.get_logger().info(f"Yaw: {yaw:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050YawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
