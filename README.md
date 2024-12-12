# Proyecto Final de Robótica: Robot Resuelve Laberintos con ArUcos y ROS
Descripción
Este proyecto consiste en un robot autónomo diseñado para resolver laberintos mediante la detección de marcadores ArUco. Los pasos principales incluyen:

Detección de ArUcos: La PiCam detecta marcadores ArUco en el entorno.
Procesamiento Central: Los datos son enviados a un nodo central en ROS.
Fusión de Datos: El nodo central utiliza un sensor MPU 6050 para calcular orientaciones.
Control de Motores: En función de los datos del ArUco y el MPU 6050, se generan señales para controlar los motores y dirigir al robot.
Tecnologías y Librerías Usadas
Arduino

cpp
Copy code
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16_multi_array.h>
Python - Nodo Central y Visión por Computadora
python
Copy code
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray
import math
import smbus
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
Instalación
Requisitos
Hardware:
Raspberry Pi con PiCam
Arduino compatible con micro-ROS
Sensor MPU 6050
Motores con controladores
Software:
ROS 2 (versión usada: foxy, humble, etc.)
Python 3.x
OpenCV con soporte para ArUcos
Librerías especificadas anteriormente.
Configuración
Instala las dependencias de Python:
bash
Copy code
pip install rclpy opencv-python cv-bridge
Configura micro-ROS en Arduino.
Conecta y calibra el MPU 6050.
Configura los tópicos y nodos en ROS.
Uso
Inicia ROS 2 y lanza los nodos:
bash
Copy code
ros2 run <nombre_paquete> <nodo>
Conecta el robot al entorno del laberinto.
Observa cómo el robot utiliza la detección de ArUcos y la información del MPU 6050 para navegar.
