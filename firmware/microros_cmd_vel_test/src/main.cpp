// Stufe 6: Empfang von geometry_msgs/Twist auf /cmd_vel.
// Jede empfangene Nachricht wird unverändert auf /cmd_vel_echo zurückgesendet.
// Es werden KEINE Motoren angesteuert.
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

rcl_subscription_t cmd_vel_sub;
rcl_publisher_t echo_pub;
geometry_msgs__msg__Twist cmd_vel_msg;   // Puffer für empfangene Nachrichten

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

// Wird vom Executor aufgerufen, sobald eine neue Twist-Nachricht angekommen ist
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // Nachweis des Empfangs: unverändert zurücksenden
  RCSOFTCHECK(rcl_publish(&echo_pub, msg, NULL));
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);  // Serial = Transport zum Agent
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_cmd_vel_test", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &echo_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_echo"));

  // Executor mit 1 Handle (nur die Subscription; Publisher brauchen keinen Handle)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
  // Kein delay(): eingehende Nachrichten sollen zeitnah verarbeitet werden
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
