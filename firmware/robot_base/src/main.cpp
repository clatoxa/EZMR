// Stufe 8/9: Differentialantrieb.
// Empfängt geometry_msgs/Twist auf /cmd_vel (micro-ROS über USB-Seriell ODER WLAN/UDP),
// rechnet in Radgeschwindigkeiten um und steuert die ATtiny212-Motorcontroller
// im Dauerbetrieb (mode 1) über UART2 / GPIO 17.
// Sicherheit: Stopp beim Start, bei Timeout (keine Twist-Nachricht) und im Fehlerfall;
// aktueller Motorbefehl wird zusätzlich jede Sekunde wiederholt (UART ohne Bestätigung).
// Kontrolltopic /wheel_cmd (geometry_msgs/Vector3):
//   x = linkes Rad, y = rechtes Rad: vorzeichenbehafteter param (+ vorwärts, - rückwärts, 0 Stopp)
//   z = 1 bei Timeout, sonst 0
// Stufe 10: Ultraschallsensor HC-SR04 (Trigger GPIO 26, Echo GPIO 25) – Messung alle 100 ms,
// nicht blockierend über Interrupt am Echo-Pin, Veröffentlichung als sensor_msgs/Range auf /range.
// Stufe 10d: passiver Piezo-Buzzer (GPIO 4 laut Anleitung 4.2.3). Subscriber /buzzer (std_msgs/UInt16):
//   0 = aus, 1 = Dauerton, >= 2 = Pieptakt in ms (Periode, 50 % an); kleinere Werte als 100 -> 100.
//   Ohne neue Nachricht für BUZZER_TIMEOUT_MS -> aus. Tonerzeugung lokal und nicht blockierend.
// Transport wird über die PlatformIO-Umgebung gewählt (env:serial / env:wifi).
#include <Arduino.h>
#include <math.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/u_int16.h>
#include <rmw_microros/rmw_microros.h>

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#include "secrets.h"            // lokal, nicht im Repository (Vorlage: secrets.h.example)
#define DEBUG_ENABLED 1         // Serial ist frei -> Statusausgaben erlaubt
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#define DEBUG_ENABLED 0         // Serial ist micro-ROS-Transport -> KEINE Ausgaben
#else
#error Nur die Transporte serial und wifi werden unterstuetzt.
#endif

#if DEBUG_ENABLED
#define DBG(...) Serial.printf(__VA_ARGS__)
#else
#define DBG(...) do {} while (0)
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
static const uint32_t RESEND_PERIOD_MS  = 1000; // Befehl regelmäßig wiederholen (UART ohne Bestätigung)
static const float    V_MAX             = STEP_DIST_M * 1000.0f / PARAM_MIN;  // ca. 0,063 m/s

// ---------- Ultraschall HC-SR04 (Pins laut Anleitung 4.2.3) ----------
static const int      US_TRIG_PIN        = 26;
static const int      US_ECHO_PIN        = 25;
static const uint32_t US_PERIOD_MS       = 100;    // 10 Hz (Datenblatt-Empfehlung: >= 60 ms Zyklus)
static const uint32_t US_TIMEOUT_US      = 40000;  // ohne fallende Flanke -> Messung beenden
static const float    US_SPEED_OF_SOUND  = 343.0f; // m/s bei ca. 20 °C (Annahme)
static const float    US_MIN_RANGE_M     = 0.02f;  // typische Herstellerangabe HC-SR04
static const float    US_MAX_RANGE_M     = 4.0f;   // typische Herstellerangabe HC-SR04
static const float    US_FIELD_OF_VIEW   = 0.26f;  // rad (ca. 15°, typische Herstellerangabe)

// ---------- Buzzer (passiver Piezo, Pin laut Anleitung 4.2.3) ----------
static const int      BUZZER_PIN            = 4;
static const uint32_t BUZZER_FREQ_HZ        = 2000;   // Annahme; Resonanz des Piezos unbekannt -> ggf. anpassen
static const uint32_t BUZZER_TIMEOUT_MS     = 1000;   // ohne neue Nachricht -> aus
static const uint16_t BUZZER_PERIOD_MIN_MS  = 100;

struct WheelCmd {
  uint8_t dir;
  uint16_t param;   // 0 = Stopp
};

// ---------- micro-ROS ----------
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t wheel_cmd_pub;
geometry_msgs__msg__Twist cmd_vel_msg;
geometry_msgs__msg__Vector3 wheel_cmd_msg;
rcl_publisher_t range_pub;
sensor_msgs__msg__Range range_msg;
static char range_frame_id[] = "ultrasonic_link";
rcl_subscription_t buzzer_sub;
std_msgs__msg__UInt16 buzzer_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ---------- Zustand ----------
float cmdV = 0.0f;              // m/s
float cmdW = 0.0f;              // rad/s
uint32_t lastCmdMs = 0;
bool cmdReceived = false;
bool lastTimedOut = true;
WheelCmd lastLeft  = {0, 0};
WheelCmd lastRight = {0, 0};
uint32_t lastSentLeftMs  = 0;
uint32_t lastSentRightMs = 0;
uint32_t lastControlMs = 0;

// Ultraschall-Zustand (Variablen der ISR: volatile)
volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoFallUs = 0;
volatile bool echoRiseSeen = false;
volatile bool echoDone = false;
enum UsState { US_IDLE, US_WAIT_ECHO };
UsState usState = US_IDLE;
uint32_t usLastTriggerMs = 0;
uint32_t usTriggerUs = 0;

// Buzzer-Zustand
uint16_t buzzerPeriodMs = 0;      // 0 = aus, 1 = Dauerton, sonst Periode
uint32_t lastBuzzerMs = 0;
bool buzzerReceived = false;
uint32_t buzzerPhaseStartMs = 0;  // Beginn des Pieptakts (für gleichmäßiges Muster)
bool buzzerOn = false;

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

void setBuzzer(bool on) {
  if (on == buzzerOn) {
    return;                                 // nur bei Zustandswechsel umschalten
  }
  if (on) {
    tone(BUZZER_PIN, BUZZER_FREQ_HZ);
  } else {
    noTone(BUZZER_PIN);
  }
  buzzerOn = on;
}

void error_loop(int line) {
  stopMotors();
  setBuzzer(false);
  DBG("FEHLER: micro-ROS-Aufruf in Zeile %d fehlgeschlagen (Agent erreichbar?). Neustart mit EN.\n", line);
  while (1) {
    delay(100);
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(__LINE__);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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

void applyWheel(uint8_t id, const WheelCmd &c, WheelCmd &last, uint32_t &lastSentMs) {
  uint32_t now = millis();
  bool same = (c.param == last.param) && (c.param == 0 || c.dir == last.dir);
  if (same && (now - lastSentMs < RESEND_PERIOD_MS)) {
    return;                                 // unverändert und kürzlich gesendet
  }
  sendMotorFrame(id, MODE_CONT, c.dir, c.param);
  last = c;
  lastSentMs = now;
}

// ---------- Regelschritt (alle CONTROL_PERIOD_MS) ----------
void controlStep() {
  uint32_t now = millis();
  bool timedOut = !cmdReceived || (now - lastCmdMs > CMD_TIMEOUT_MS);

  if (timedOut != lastTimedOut) {
    DBG(timedOut ? "Timeout -> Stopp\n" : "Befehle empfangen -> Fahrt\n");
    lastTimedOut = timedOut;
  }

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
  applyWheel(ID_LEFT, l, lastLeft, lastSentLeftMs);
  applyWheel(ID_RIGHT, r, lastRight, lastSentRightMs);

  wheel_cmd_msg.x = signedParam(l, DIR_FWD_LEFT);
  wheel_cmd_msg.y = signedParam(r, DIR_FWD_RIGHT);
  wheel_cmd_msg.z = timedOut ? 1.0 : 0.0;
  RCSOFTCHECK(rcl_publish(&wheel_cmd_pub, &wheel_cmd_msg, NULL));
}

// ---------- Ultraschall ----------
// Interrupt bei jeder Flanke am Echo-Pin: nur Zeitstempel merken (kurz halten!)
void IRAM_ATTR echoIsr() {
  uint32_t t = micros();
  if (digitalRead(US_ECHO_PIN)) {
    echoRiseUs = t;
    echoRiseSeen = true;
  } else if (echoRiseSeen) {
    echoFallUs = t;
    echoDone = true;
  }
}

void publishRange(float rangeM) {
  if (rmw_uros_epoch_synchronized()) {
    int64_t ns = rmw_uros_epoch_nanos();
    range_msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
    range_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  }
  range_msg.range = rangeM;
  RCSOFTCHECK(rcl_publish(&range_pub, &range_msg, NULL));
}

// Nicht blockierender Messablauf, wird in jedem loop()-Durchlauf aufgerufen
void ultrasonicStep() {
  uint32_t nowMs = millis();
  if (usState == US_IDLE) {
    if (nowMs - usLastTriggerMs < US_PERIOD_MS) {
      return;
    }
    usLastTriggerMs += US_PERIOD_MS;                 // Soll-Takt halten
    if (nowMs - usLastTriggerMs >= US_PERIOD_MS) {
      usLastTriggerMs = nowMs;
    }
    echoRiseSeen = false;
    echoDone = false;
    digitalWrite(US_TRIG_PIN, HIGH);                 // 10-µs-Triggerimpuls (einzige kurze Wartezeit)
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN, LOW);
    usTriggerUs = micros();
    usState = US_WAIT_ECHO;
    return;
  }

  // US_WAIT_ECHO
  if (echoDone) {
    uint32_t pulseUs = echoFallUs - echoRiseUs;
    float d = (float)pulseUs * 1e-6f * US_SPEED_OF_SOUND / 2.0f;   // Hin- und Rückweg
    if (d > US_MAX_RANGE_M) {
      d = INFINITY;                                  // REP 117: kein Objekt im Messbereich
    } else if (d < US_MIN_RANGE_M) {
      d = -INFINITY;                                 // REP 117: zu nah
    }
    publishRange(d);
    usState = US_IDLE;
  } else if (micros() - usTriggerUs > US_TIMEOUT_US) {
    // Echo begann, endete aber nicht rechtzeitig -> außerhalb Messbereich;
    // gar kein Echo -> Messung ungültig
    publishRange(echoRiseSeen ? INFINITY : NAN);
    usState = US_IDLE;
  }
}

void setupRangeMsg() {
  range_msg.header.frame_id.data = range_frame_id;
  range_msg.header.frame_id.size = strlen(range_frame_id);
  range_msg.header.frame_id.capacity = sizeof(range_frame_id);
  range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view = US_FIELD_OF_VIEW;
  range_msg.min_range = US_MIN_RANGE_M;
  range_msg.max_range = US_MAX_RANGE_M;
}

// ---------- Callback: nur speichern ----------
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  cmdV = (float)msg->linear.x;
  cmdW = (float)msg->angular.z;
  lastCmdMs = millis();
  cmdReceived = true;
}

void buzzer_callback(const void *msgin) {
  const std_msgs__msg__UInt16 *msg = (const std_msgs__msg__UInt16 *)msgin;
  if (msg->data != buzzerPeriodMs) {
    buzzerPhaseStartMs = millis();          // neues Muster beginnt mit "an"
  }
  buzzerPeriodMs = msg->data;
  lastBuzzerMs = millis();
  buzzerReceived = true;
}

// Nicht blockierend, wird in jedem loop()-Durchlauf aufgerufen
void buzzerStep() {
  uint32_t now = millis();
  uint16_t p = buzzerPeriodMs;
  if (!buzzerReceived || now - lastBuzzerMs > BUZZER_TIMEOUT_MS) {
    p = 0;                                  // Timeout -> aus
  }
  if (p == 0) {
    setBuzzer(false);
    return;
  }
  if (p == 1) {
    setBuzzer(true);
    return;
  }
  if (p < BUZZER_PERIOD_MIN_MS) {
    p = BUZZER_PERIOD_MIN_MS;
  }
  setBuzzer((now - buzzerPhaseStartMs) % p < p / 2);
}

void setupTransport() {
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  Serial.begin(115200);
  set_microros_serial_transports(Serial);   // Serial = Transport zum Agent
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
  Serial.begin(115200);
  delay(200);
  static char ssid[] = WIFI_SSID;           // Funktion erwartet char* (nicht const)
  static char psk[]  = WIFI_PASSWORD;
  IPAddress agent_ip(AGENT_IP);
  DBG("\n=== robot_base (WLAN) ===\nVerbinde mit WLAN '%s' ...\n", ssid);
  set_microros_wifi_transports(ssid, psk, agent_ip, AGENT_PORT);  // blockiert bis WLAN verbunden
  WiFi.setSleep(false);                     // Energiesparmodus aus (geringere Latenz, Annahme)
  DBG("WLAN verbunden, eigene IP: %s\n", WiFi.localIP().toString().c_str());
  DBG("Agent: %s, Port %d\n", agent_ip.toString().c_str(), (int)AGENT_PORT);
#endif
}

void setup() {
  // Motoren zuerst in definierten Zustand bringen
  Serial2.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
  stopMotors();

  // Ultraschall-Pins
  pinMode(US_TRIG_PIN, OUTPUT);
  digitalWrite(US_TRIG_PIN, LOW);
  pinMode(US_ECHO_PIN, INPUT);
  noTone(BUZZER_PIN);                       // Buzzer definiert aus

  setupTransport();
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

  RCCHECK(rclc_publisher_init_default(
    &range_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "range"));
  setupRangeMsg();

  RCCHECK(rclc_subscription_init_default(
    &buzzer_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    "buzzer"));

  // Zeit mit dem Agent abgleichen (für Zeitstempel in /range); Fehler ist nicht kritisch
  rmw_uros_sync_session(1000);
  DBG("Zeitabgleich mit Agent: %s\n", rmw_uros_epoch_synchronized() ? "ok" : "nicht verfuegbar");

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));  // 2 Subscriber
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &buzzer_sub, &buzzer_msg, &buzzer_callback, ON_NEW_DATA));

  DBG("micro-ROS bereit: Node /robot_base, Subscriber /cmd_vel, /buzzer, Publisher /wheel_cmd, /range\n");
  lastControlMs = millis();
  usLastTriggerMs = millis();
  attachInterrupt(digitalPinToInterrupt(US_ECHO_PIN), echoIsr, CHANGE);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs += CONTROL_PERIOD_MS;              // Soll-Takt halten (kein Drift)
    if (now - lastControlMs >= CONTROL_PERIOD_MS) {
      lastControlMs = now;                           // nach langer Blockade nicht nachholen
    }
    controlStep();
  }

  ultrasonicStep();
  buzzerStep();
}
