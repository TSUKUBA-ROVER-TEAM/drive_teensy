#include "C610Bus.h"
#include "velocity_pid.h"
#include <Arduino.h>
#include <Servo.h>
#include <micro_ros_platformio.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

#include <rcl/logging.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/u_int8_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define PI 3.14
#define LED_PIN       38
#define NUM_LEDS 100

#define SERVO_SCALE 130.0
#define SERVO_ORIGIN 85

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

rcl_subscription_t drive_command_subscriber;
std_msgs__msg__Float64MultiArray drive_command_msg;

rcl_subscription_t steer_command_subscriber;
std_msgs__msg__Float64MultiArray steer_command_msg;

rcl_subscription_t arm_command_subscriber;
std_msgs__msg__Float64MultiArray arm_command_msg;

rcl_publisher_t drive_feedback_publisher;
std_msgs__msg__Float64MultiArray drive_feedback_msg;

rcl_publisher_t arm_feedback_publisher;
std_msgs__msg__Float64MultiArray arm_feedback_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

C610Bus<CAN2> bus;
long last_time = 0;
int32_t target_current[6] = {0, 0, 0, 0, 0, 0};
double target_velocity[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double steer_angle[4] = {0.0, 0.0, 0.0, 0.0};

// ウォッチドッグ用：最終コマンド受信時刻 [ms]
unsigned long last_drive_command_time = 0;
unsigned long last_steer_command_time = 0;
unsigned long last_arm_command_time = 0;

// ウォッチドッグタイムアウト [ms]（この期間コマンドが来なければ停止）
const unsigned long WATCHDOG_TIMEOUT_MS = 200;

VelocityPID drive_left_forward(1000.0, 0.0, 0.0, 1.0);
VelocityPID drive_right_forward(1000.0, 0.0, 0.0, 1.0);
VelocityPID drive_left_backward(1000.0, 0.0, 0.0, 1.0);
VelocityPID drive_right_backward(1000.0, 0.0, 0.0, 1.0);
VelocityPID arm_1_joint(1000.0, 0.0, 0.0, 1.0);
VelocityPID hand_joint(1000.0, 0.0, 0.0, 1.0);

Servo steer_left_forward;
Servo steer_right_forward;
Servo steer_left_backward;
Servo steer_right_backward;

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

// エラーハンドリングループ
void error_loop() {
  while (1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    bus.PollCAN();
    long now = millis();

    // ウォッチドッグ: 一定時間コマンドが来なければ指令値をゼロにする
    if (now - last_drive_command_time > WATCHDOG_TIMEOUT_MS) {
      for (int i = 0; i < 4; i++) target_velocity[i] = 0.0;
    }
    if (now - last_arm_command_time > WATCHDOG_TIMEOUT_MS) {
      target_velocity[4] = 0.0;
      target_velocity[5] = 0.0;
    }
    // ステアはタイムアウト時に中立位置へ戻す
    if (now - last_steer_command_time > WATCHDOG_TIMEOUT_MS) {
      for (int i = 0; i < 4; i++) steer_angle[i] = 0.0;
    }

    if (now - last_time >= 10) {

      target_current[0] =
          drive_left_forward.compute(target_velocity[0] * -1.0, bus.Get(0).Velocity(),
                                     (now - last_time) / 1000.0);
      target_current[1] =
          drive_right_forward.compute(target_velocity[1], bus.Get(1).Velocity(),
                                      (now - last_time) / 1000.0);
      target_current[2] =
          drive_left_backward.compute(target_velocity[2], bus.Get(2).Velocity(),
                                      (now - last_time) / 1000.0);
      target_current[3] = drive_right_backward.compute(
          target_velocity[3] * -1.0, bus.Get(3).Velocity(),
          (now - last_time) / 1000.0);

      target_current[4] = arm_1_joint.compute(target_velocity[4], bus.Get(4).Velocity(), (now - last_time) / 1000.0);
      target_current[5] = hand_joint.compute(target_velocity[5], bus.Get(5).Velocity(), (now - last_time) / 1000.0);

      bus.CommandTorques(target_current[0], target_current[1],
                         target_current[2], target_current[3],
                         C610Subbus::kOneToFourBlinks);
      bus.CommandTorques(target_current[4], target_current[5], 0, 0,
                         C610Subbus::kFiveToEightBlinks);

      drive_feedback_msg.data.size = 4;
      drive_feedback_msg.data.data[0] = bus.Get(0).Velocity();
      drive_feedback_msg.data.data[1] = bus.Get(1).Velocity();
      drive_feedback_msg.data.data[2] = bus.Get(2).Velocity();
      drive_feedback_msg.data.data[3] = bus.Get(3).Velocity();
      arm_feedback_msg.data.size = 2;
      arm_feedback_msg.data.data[0] = bus.Get(4).Velocity();
      arm_feedback_msg.data.data[1] = bus.Get(5).Velocity();

      RCSOFTCHECK(
          rcl_publish(&drive_feedback_publisher, &drive_feedback_msg, NULL));
      RCSOFTCHECK(
          rcl_publish(&arm_feedback_publisher, &arm_feedback_msg, NULL));

      steer_left_forward.write(
          static_cast<int>(SERVO_ORIGIN + (steer_angle[0] / PI) * SERVO_SCALE));
      steer_right_forward.write(
          static_cast<int>(80 + (steer_angle[1] / PI) * SERVO_SCALE * -1.0));
      steer_left_backward.write(
          static_cast<int>(80 + (steer_angle[2] / PI) * SERVO_SCALE * -1.0));
      steer_right_backward.write(
          static_cast<int>(SERVO_ORIGIN + (steer_angle[3] / PI) * SERVO_SCALE));

      last_time = now;
    }
  }
}

void drive_command_callback(const void *msgin) {
  digitalWrite(13, LOW);
  const std_msgs__msg__Float64MultiArray *drive_command_msg =
      (const std_msgs__msg__Float64MultiArray *)msgin;
  for (int i = 0; i < 4; i++) {
    target_velocity[i] = drive_command_msg->data.data[i];
  }
  last_drive_command_time = millis();  // ウォッチドッグ用タイムスタンプ更新
}

void steer_command_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *steer_command_msg =
      (const std_msgs__msg__Float64MultiArray *)msgin;
  for (int i = 0; i < 4; i++) {
    steer_angle[i] = steer_command_msg->data.data[i];
  }
  last_steer_command_time = millis();  // ウォッチドッグ用タイムスタンプ更新
}

void arm_command_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *arm_command_msg =
      (const std_msgs__msg__Float64MultiArray *)msgin;
  for (int i = 0; i < 2; i++) {
    target_velocity[i + 4] = arm_command_msg->data.data[i];
  }
  last_arm_command_time = millis();  // ウォッチドッグ用タイムスタンプ更新
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &drive_command_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "drive_controller/commands"));
  RCCHECK(rclc_subscription_init_default(
      &arm_command_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "arm_controller/commands"));
  RCCHECK(rclc_subscription_init_default(
      &steer_command_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "steer_controller/commands"));
  RCCHECK(rclc_publisher_init_default(
      &drive_feedback_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "drive_controller/feedbacks"));
  RCCHECK(rclc_publisher_init_default(
      &arm_feedback_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "arm_controller/feedbacks"));

  // create timer for 20ms update rate
  const unsigned int timer_timeout = 100; // 100 Hz
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_US_TO_NS(timer_timeout),
                                  timer_callback));

  drive_command_msg.data.capacity = 100;
  drive_command_msg.data.size = 4;
  drive_command_msg.data.data =
      (double *)malloc(drive_command_msg.data.capacity * sizeof(double));

  drive_command_msg.layout.dim.capacity = 100;
  drive_command_msg.layout.dim.size = 0;
  drive_command_msg.layout.dim.data =
      (std_msgs__msg__MultiArrayDimension *)malloc(
          drive_command_msg.layout.dim.capacity *
          sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < drive_command_msg.layout.dim.capacity; i++) {
    drive_command_msg.layout.dim.data[i].label.capacity = 10;
    drive_command_msg.layout.dim.data[i].label.size = 0;
    drive_command_msg.layout.dim.data[i].label.data = (char *)malloc(
        drive_command_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  arm_command_msg.data.capacity = 100;
  arm_command_msg.data.size = 2;
  arm_command_msg.data.data =
      (double *)malloc(arm_command_msg.data.capacity * sizeof(double));
  
    arm_command_msg.layout.dim.capacity = 100;
  arm_command_msg.layout.dim.size = 0;
  arm_command_msg.layout.dim.data =
      (std_msgs__msg__MultiArrayDimension *)malloc(
          arm_command_msg.layout.dim.capacity *
          sizeof(std_msgs__msg__MultiArrayDimension)); 

  for (size_t i = 0; i < arm_command_msg.layout.dim.capacity; i++) {
    arm_command_msg.layout.dim.data[i].label.capacity = 10;
    arm_command_msg.layout.dim.data[i].label.size = 0;
    arm_command_msg.layout.dim.data[i].label.data = (char *)malloc(
        arm_command_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  steer_command_msg.data.capacity = 100;
  steer_command_msg.data.size = 4;
  steer_command_msg.data.data =
      (double *)malloc(steer_command_msg.data.capacity * sizeof(double));

  steer_command_msg.layout.dim.capacity = 100;
  steer_command_msg.layout.dim.size = 0;
  steer_command_msg.layout.dim.data =
      (std_msgs__msg__MultiArrayDimension *)malloc(
          steer_command_msg.layout.dim.capacity *
          sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < steer_command_msg.layout.dim.capacity; i++) {
    steer_command_msg.layout.dim.data[i].label.capacity = 10;
    steer_command_msg.layout.dim.data[i].label.size = 0;
    steer_command_msg.layout.dim.data[i].label.data = (char *)malloc(
        steer_command_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  drive_feedback_msg.data.capacity = 100;
  drive_feedback_msg.data.size = 4;
  drive_feedback_msg.data.data =
      (double *)malloc(drive_feedback_msg.data.capacity * sizeof(double));
  drive_feedback_msg.layout.dim.capacity = 100;
  drive_feedback_msg.layout.dim.size = 0;
  drive_feedback_msg.layout.dim.data =
      (std_msgs__msg__MultiArrayDimension *)malloc(
          drive_feedback_msg.layout.dim.capacity *
          sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < drive_feedback_msg.layout.dim.capacity; i++) {
    drive_feedback_msg.layout.dim.data[i].label.capacity = 10;
    drive_feedback_msg.layout.dim.data[i].label.size = 0;
    drive_feedback_msg.layout.dim.data[i].label.data = (char *)malloc(
        drive_feedback_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }


  arm_feedback_msg.data.capacity = 100;
  arm_feedback_msg.data.size = 2;
  arm_feedback_msg.data.data =
      (double *)malloc(arm_feedback_msg.data.capacity * sizeof(double));
  arm_feedback_msg.layout.dim.capacity = 100;
  arm_feedback_msg.layout.dim.size = 0;
  arm_feedback_msg.layout.dim.data =
      (std_msgs__msg__MultiArrayDimension *)malloc(
          arm_feedback_msg.layout.dim.capacity *
          sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < arm_feedback_msg.layout.dim.capacity; i++) {
    arm_feedback_msg.layout.dim.data[i].label.capacity = 10;
    arm_feedback_msg.layout.dim.data[i].label.size = 0;
    arm_feedback_msg.layout.dim.data[i].label.data = (char *)malloc(
        arm_feedback_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &drive_command_subscriber,
                                         &drive_command_msg,
                                         &drive_command_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &arm_command_subscriber,
                                         &arm_command_msg,
                                         &arm_command_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &steer_command_subscriber,
                                         &steer_command_msg,
                                         &steer_command_callback, ON_NEW_DATA));
                                         
  steer_left_forward.attach(28);
  steer_right_forward.attach(29);
  steer_left_backward.attach(8);
  steer_right_backward.attach(7);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pixels.begin();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}