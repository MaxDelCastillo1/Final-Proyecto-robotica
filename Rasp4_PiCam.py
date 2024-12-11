import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ImageSubscriber(Node):
    def _init_(self):
        super()._init_('image_subscriber')
        
        self.id_publisher_ = self.create_publisher(Float32MultiArray, 'aruco', 10)
        
        self.br = CvBridge()

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters_create()

        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)  # Usar V4L2 para video en Linux
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara')
        else:
            self.get_logger().info('Cámara abierta correctamente')

        self.marker_real_size = 0.067  
        self.focal_length = 450  

    def estimate_distance(self, corner, focal_length, real_marker_size):
        top_left, top_right, bottom_right, bottom_left = corner[0]
        width = np.linalg.norm(top_right - top_left)
        height = np.linalg.norm(bottom_right - bottom_left)
        avg_marker_size_in_pixels = (width + height) / 2

        distance = (real_marker_size * focal_length) * 100 / avg_marker_size_in_pixels  # convertimos a cm
        return distance
    
    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('No se pudo capturar el frame')
            return None
        return frame

    def process_frame(self, frame):
        corners, ids, _ = aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)
        
        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids):
                distance = self.estimate_distance(marker_corners, self.focal_length, self.marker_real_size)
            
                
                msg = Float32MultiArray()
                msg.data = [float(marker_id[0]), float(distance)]  
                
                self.id_publisher_.publish(msg)

                

        frame = aruco.drawDetectedMarkers(frame, corners, ids)
        return frame

    def listener_callback(self):
        current_frame = self.capture_frame()
        if current_frame is None:
            return

        processed_frame = self.process_frame(current_frame)
        
        processed_frame = cv2.flip(processed_frame, 0)
        
        cv2.imshow("camera", processed_frame)
        cv2.waitKey(1)

    def run(self):
        while rclpy.ok():
            self.listener_callback()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    image_subscriber.run()
    image_subscriber.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
