// Stufe 8: Differentialantrieb.
// Empfängt geometry_msgs/Twist auf /cmd_vel (micro-ROS über USB-Seriell),
// rechnet in Radgeschwindigkeiten um und steuert die ATtiny212-Motorcontroller
// im Dauerbetrieb (mode 1) über UART2 / GPIO 17.
// Sicherheit: Stopp beim Start, bei Timeout (keine Twist-Nachricht) und im Fehlerfall.
// Kontrolltopic /wheel_cmd (geometry_msgs/Vector3):
//   x = linkes Rad, y = rechtes Rad: vorzeichenbehafteter param (+ vorwärts, - rückwärts, 0 Stopp)
//   z = 1 bei Timeout, sonst 0
#include <Arduino.h>
#include <math.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This firmware requires the Arduino serial transport.
#endif

// ---------- Geometrie (gemessen 08.10.2026, ca.-Werte) ----------
static const float WHEEL_DIAMETER_M = 0.082f;
static const float TRACK_WIDTH_M    = 0.165f;
static const float STEPS_PER_REV    = 2048.0f;  // Stufe 7, Versuch F
static const float STEP_DIST_M      = (float)M_PI * WHEEL_DIAMETER_M / STEPS_PER_REV;

// ---------- Motorprotokoll (Stufe 7) ----------
static const int      MOTOR_TX_PIN  = 17;
static const int      MOTOR_RX_PIN  = 16;       // nicht benutzt
static const uint32_t MOTOR_BAUD    = 9600;
static const uint8_t  START_BYTE    = 0xAA;
static const uint8_t  ID_LEFT       = 1;
static const uint8_t  ID_RIGHT      = 2;
static const uint8_t  MODE_CONT     = 1;        // Dauerbetrieb, param = ms pro Schritt, 0 = Stopp
static const uint8_t  DIR_FWD_LEFT  = 1;        // links: dir 1 = vorwärts
static const uint8_t  DIR_FWD_RIGHT = 0;        // rechts: dir 0 = vorwärts

// ---------- Grenzen und Zeitverhalten ----------
static const uint16_t PARAM_MIN         = 2;    // schnellste geprüfte Stufe
static const uint16_t PARAM_MAX         = 100;  // langsamer -> Stillstand
static const uint32_t CMD_TIMEOUT_MS    = 500;
static const uint32_t CONTROL_PERIOD_MS = 50;
static const float    V_MAX             = STEP_DIST_M * 1000.0f / PARAM_MIN;  // ca. 0,063 m/s

struct WheelCmd {
  uint8_t dir;
  uint16_t param;   // 0 = Stopp
};

// ---------- micro-ROS ----------
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t wheel_cmd_pub;
geometry_msgs__msg__Twist cmd_vel_msg;
geometry_msgs__msg__Vector3 wheel_cmd_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ---------- Zustand ----------
float cmdV = 0.0f;              // m/s
float cmdW = 0.0f;              // rad/s
uint32_t lastCmdMs = 0;
bool cmdReceived = false;
WheelCmd lastLeft  = {0, 0};
WheelCmd lastRight = {0, 0};
uint32_t lastControlMs = 0;

// ---------- Motor-UART ----------
void sendMotorFrame(uint8_t id, uint8_t mode, uint8_t dir, uint16_t param) {
  uint8_t f[6] = {START_BYTE, id, mode, dir,
                  (uint8_t)(param >> 8), (uint8_t)(param & 0xFF)};  // Big-Endian
  Serial2.write(f, sizeof(f));
  Serial2.flush();
}

void stopMotors() {
  sendMotorFrame(ID_LEFT, MODE_CONT, 0, 0);
  sendMotorFrame(ID_RIGHT, MODE_CONT, 0, 0);
  lastLeft  = {0, 0};
  lastRight = {0, 0};
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  stopMotors();
  while (1) {
    delay(100);
  }
}

// ---------- Umrechnung ----------
// Radgeschwindigkeit [m/s] -> Richtung und param (ms pro Schritt)
WheelCmd wheelSpeedToCmd(float v, uint8_t dirForward) {
  WheelCmd c = {0, 0};
  float a = fabsf(v);
  if (!isfinite(v) || a < 1e-6f) {
    return c;                               // Stopp
  }
  long p = lroundf(1000.0f * STEP_DIST_M / a);
  if (p < PARAM_MIN) {
    p = PARAM_MIN;
  }
  if (p > PARAM_MAX) {
    return c;                               // zu langsam -> Stopp
  }
  c.param = (uint16_t)p;
  c.dir = (v > 0.0f) ? dirForward : (uint8_t)(1 - dirForward);
  return c;
}

// Für das Kontrolltopic: vorzeichenbehafteter param (+ vorwärts)
float signedParam(const WheelCmd &c, uint8_t dirForward) {
  if (c.param == 0) {
    return 0.0f;
  }
  return (c.dir == dirForward) ? (float)c.param : -(float)c.param;
}

void applyWheel(uint8_t id, const WheelCmd &c, WheelCmd &last) {
  bool same = (c.param == last.param) && (c.param == 0 || c.dir == last.dir);
  if (same) {
    return;                                 // unverändert -> nichts senden
  }
  sendMotorFrame(id, MODE_CONT, c.dir, c.param);
  last = c;
}

// ---------- Regelschritt (alle CONTROL_PERIOD_MS) ----------
void controlStep() {
  uint32_t now = millis();
  bool timedOut = !cmdReceived || (now - lastCmdMs > CMD_TIMEOUT_MS);

  float vl = 0.0f;
  float vr = 0.0f;
  if (!timedOut && isfinite(cmdV) && isfinite(cmdW)) {
    vl = cmdV - cmdW * TRACK_WIDTH_M / 2.0f;
    vr = cmdV + cmdW * TRACK_WIDTH_M / 2.0f;
    float m = fmaxf(fabsf(vl), fabsf(vr));
    if (m > V_MAX) {                        // proportionale Sättigung
      float s = V_MAX / m;
      vl *= s;
      vr *= s;
    }
  }

  WheelCmd l = wheelSpeedToCmd(vl, DIR_FWD_LEFT);
  WheelCmd r = wheelSpeedToCmd(vr, DIR_FWD_RIGHT);
  applyWheel(ID_LEFT, l, lastLeft);
  applyWheel(ID_RIGHT, r, lastRight);

  wheel_cmd_msg.x = signedParam(l, DIR_FWD_LEFT);
  wheel_cmd_msg.y = signedParam(r, DIR_FWD_RIGHT);
  wheel_cmd_msg.z = timedOut ? 1.0 : 0.0;
  RCSOFTCHECK(rcl_publish(&wheel_cmd_pub, &wheel_cmd_msg, NULL));
}

// ---------- Callback: nur speichern ----------
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  cmdV = (float)msg->linear.x;
  cmdW = (float)msg->angular.z;
  lastCmdMs = millis();
  cmdReceived = true;
}

void setup() {
  // Motoren zuerst in definierten Zustand bringen
  Serial2.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
  stopMotors();

  Serial.begin(115200);
  set_microros_serial_transports(Serial);   // Serial = Transport zum Agent
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_base", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &wheel_cmd_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "wheel_cmd"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  lastControlMs = millis();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    controlStep();
  }
}
