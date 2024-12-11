#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16_multi_array.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#define LED_PIN 2

#define encoderA1 34
#define encoderB1 35
#define PWM1 32
#define motorA1 33
#define motorB1 25

#define encoderA2 13
#define encoderB2 12
#define PWM2 14
#define motorA2 27
#define motorB2 26

rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray msg_set;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int target1 = 0, target2 = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void sub_callback(const void * msgin)
{
  const std_msgs__msg__Int16MultiArray * msg_set = (const std_msgs__msg__Int16MultiArray *)msgin;

  Serial.println("Received");

  target1 = msg_set->data.data[0];
  target2 = msg_set->data.data[1];
}

void setup(){
  Serial.begin(115200);

  pinMode(motorA1, OUTPUT);
  pinMode(motorB1, OUTPUT); 
  pinMode(PWM1, OUTPUT);
  pinMode(encoderA1, INPUT);
  pinMode(encoderB1, INPUT);

  pinMode(motorA2, OUTPUT);
  pinMode(motorB2, OUTPUT); 
  pinMode(PWM2, OUTPUT);
  pinMode(encoderA2, INPUT);
  pinMode(encoderB2, INPUT);

  Serial.println("Starting connection...");

  set_microros_wifi_transports("LUNA", "rockosimon", "192.168.1.14", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "ESP32", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_set, &sub_callback, ON_NEW_DATA));

  setMotor1(0);
  setMotor2(0);

  msg_set.data.size = 2;
  msg_set.data.capacity = 2;
  msg_set.data.data = (int16_t*)malloc(2 * sizeof(int16_t));

  Serial.println("Connected");
}

void loop(){
  // Enviar velocidades directamente a los motores
  setMotor1(target1);
  setMotor2(target2);

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  delay(100); // Delay para evitar sobrecarga del sistema
}

void setMotor1(int vel){
  digitalWrite(motorA1, vel < 0);
  digitalWrite(motorB1, vel > 0);
  analogWrite(PWM1, abs(vel));
}

void setMotor2(int vel){
  digitalWrite(motorA2, vel < 0);
  digitalWrite(motorB2, vel > 0);
  analogWrite(PWM2, abs(vel));
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
