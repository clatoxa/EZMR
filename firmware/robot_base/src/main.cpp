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
// Stufe 11 (Anleitung 2.6, FreeRTOS): OLED-Display SSD1306 (I2C 0x3C, GPIO 21/22, 400 kHz).
//   Eigene Task auf Kern 0 (loop() läuft auf Kern 1). Die Task besitzt den I2C-Bus allein.
//   Gemeinsame Anzeigedaten sind durch einen Mutex geschützt; die Task kopiert sie unter dem Mutex
//   und zeichnet/überträgt danach OHNE Mutex (Übertragung ca. 32 ms, gemessen in Stufe 11a).
//   Ereignissteuerung: Zustandswechsel vom Host weckt die Task sofort (Task-Notification),
//   sonst Aktualisierung alle DISPLAY_PERIOD_MS.
//   micro-ROS wird ausschließlich aus loop() verwendet (nicht für mehrere Tasks ausgelegt).
//   Subscriber /robot_state (std_msgs/UInt8, von obstacle_stop): 1 = FREI, 2 = HINDERNIS, 3 = keine Sensordaten.
//   Messung (Stufe 11d): Publisher /loop_diag (geometry_msgs/Vector3) jede Sekunde:
//     x = längster loop()-Durchlauf [us], y = größter / z = kleinster Abstand zwischen Regelschritten [us].
//   DISPLAY_MODE (Vergleich Stufe 11d): 0 = Display nach dem Startbild nicht aktualisiert (Grundlinie),
//                                       1 = eigene FreeRTOS-Task (Standard), 2 = direkt in loop().
// v5.1: Fehleranzeige auf dem Display (Agent nicht erreichbar), Buzzer-Pin beim Start als Ausgang LOW
//       (statt noTone() -> keine LEDC-Fehlermeldung mehr), eindeutige Anzeige im Messmodus 0.
// v6 (Anleitung 2.6 Punkt 3): Bumper links GPIO 12 (JP4), rechts GPIO 14 (JP3), ermittelt mit hw_probe.
//   Öffner nach VCC, Platine zieht auf GND: nicht gedrückt = HIGH, gedrückt = LOW, abgezogen = LOW (fail-safe).
//   ACHTUNG GPIO 12 ist Strapping-Pin: linker Bumper beim Start/Reset/Upload ABGEZOGEN (Anleitung 4.2.3).
//   Interrupt (CHANGE) -> ISR gibt binäre Semaphore frei -> bumperTask (Kern 0, Prio 2) wacht auf,
//   entprellt asymmetrisch (LOW sofort = gedrückt, HIGH erst nach BUMPER_RELEASE_MS stabil = frei),
//   stoppt bei Vorwärtsfahrt sofort lokal (Motor-UART durch Mutex geschützt) und meldet den Zustand.
//   loop() veröffentlicht /bumper (std_msgs/UInt8, Bit 0 links, Bit 1 rechts) bei Änderung und alle 200 ms;
//   Auswertung auf dem Host (obstacle_stop). Lokal zusätzlich: keine Vorwärtsfahrt bei gedrücktem Bumper.
//   Serielle Ausgaben aus mehreren Tasks durch Mutex geschützt (2.6 Punkt 2).
// v7 (Stufe 11f): IMU MPU-6500 (I2C 0x68, WHO_AM_I 0x70) – Gyroskop ±250 °/s, Beschleunigung ±2 g.
//   imuTask (Kern 0, Prio 3) tastet periodisch mit vTaskDelayUntil alle 10 ms ab, korrigiert den Nullpunkt
//   (Kalibrierung 3 s im Stillstand beim Start) und integriert den Gierwinkel mit der tatsächlichen Zeit.
//   I2C wird jetzt von zwei Tasks genutzt -> i2cMutex. Das Display überträgt sein Bild seitenweise
//   (8 x 128 Byte) und gibt den Bus zwischen den Seiten frei, damit die IMU höchstens eine Seite warten muss.
//   loop() veröffentlicht /imu (sensor_msgs/Imu, frame imu_link) alle 50 ms: Gier als Quaternion,
//   Drehraten [rad/s], Beschleunigungen [m/s^2]. Roll/Nick werden nicht geschätzt (große Kovarianz).
// v7.1: STREAM_BEST_EFFORT 1 -> /range, /wheel_cmd, /imu als "best effort" (keine Bestätigung, kein Warten);
//       /bumper und /loop_diag bleiben "reliable". Host-Subscriber müssen best effort akzeptieren
//       (qos_profile_sensor_data), sonst kommt keine Verbindung zustande. 0 = alles reliable (wie v7).
// v7.2: Display zeigt den Gierwinkel auf 0…360° begrenzt (nur Anzeige). Intern, auf /imu und im
//       seriellen Log bleibt der Winkel fortlaufend (unverändert gegenüber v7.1, getestet in 11f-c).
//       Stand 09.10.2026: v7.2 getestet (Anzeige bleibt beim Drehen in 0…360°).
// v7.3 (Stufe 11i): Subscriber /imu_reset (std_msgs/UInt8): 1 = Gyro-Nullpunkt 3 s neu messen und
//       Gierwinkel auf 0 setzen, 0 = nur Winkel auf 0. Nur im Stillstand (beide Räder param 0); fährt
//       der Roboter an, wird die Messung verworfen. Neuer Nullpunkt wird nur übernommen, wenn er um
//       weniger als IMU_RECAL_MAX_DPS vom alten abweicht (sonst wurde der Roboter bewegt). Während der
//       Messung bleibt der Winkel stehen, Display zeigt "IMU kalibriert". Anlass: Drift ≈ −0,3 °/s nach
//       längerer Betriebszeit (Test O1), Nullpunkt wurde bisher nur beim Einschalten bestimmt.
//       Stand 09.10.2026: v7.3 UNGETESTET.
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
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/u_int8.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <rmw_microros/rmw_microros.h>

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#include "secrets.h"            // lokal, nicht im Repository (Vorlage: secrets.h.example)
#define DEBUG_ENABLED 1         // Serial ist frei -> Statusausgaben erlaubt
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#define DEBUG_ENABLED 0         // Serial ist micro-ROS-Transport -> KEINE Ausgaben
#else
#error Nur die Transporte serial und wifi werden unterstuetzt.
#endif

SemaphoreHandle_t serialMutex = NULL;   // schützt Serial bei Ausgaben aus mehreren Tasks (v6)

#if DEBUG_ENABLED
#define DBG(...) do { \
    if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY); \
    Serial.printf(__VA_ARGS__); \
    if (serialMutex) xSemaphoreGive(serialMutex); \
  } while (0)
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

// ---------- Display (Stufe 11) ----------
#ifndef DISPLAY_MODE
#define DISPLAY_MODE 1                                // 0 = aus (Grundlinie), 1 = Task, 2 = in loop()
#endif
#define DISPLAY_IN_TASK (DISPLAY_MODE == 1)
#define DISPLAY_IN_LOOP (DISPLAY_MODE == 2)

#ifndef STREAM_BEST_EFFORT
#define STREAM_BEST_EFFORT 1                          // 1 = Sensorströme best effort, 0 = reliable (v7)
#endif
#if STREAM_BEST_EFFORT
#define STREAM_PUB_INIT rclc_publisher_init_best_effort
#else
#define STREAM_PUB_INIT rclc_publisher_init_default
#endif
static const int      I2C_SDA_PIN           = 21;     // Anleitung 4.2.3
static const int      I2C_SCL_PIN           = 22;
static const uint8_t  DISPLAY_ADDR_7BIT     = 0x3C;   // I2C-Scan + Lötbrücke "0x78"
static const uint32_t DISPLAY_BUS_HZ        = 400000; // Stufe 11a: ca. 32 ms pro Bild (100 kHz: ca. 113 ms)
static const uint32_t DISPLAY_PERIOD_MS     = 250;    // Aktualisierung ohne Ereignis (4 Hz)
static const uint32_t ROBOT_STATE_TIMEOUT_MS = 1500;  // ohne /robot_state -> "Host: --"
static const uint32_t DIAG_PERIOD_MS        = 1000;

// ---------- Bumper (v6) ----------
static const int      BUMPER_LEFT_PIN       = 12;     // JP4, Strapping-Pin!
static const int      BUMPER_RIGHT_PIN      = 14;     // JP3
static const uint32_t BUMPER_RELEASE_MS     = 100;    // HIGH muss so lange stabil sein (Prellen gemessen: 55 ms LOW)
static const uint32_t BUMPER_PUB_PERIOD_MS  = 200;

// ---------- IMU (v7, Stufe 11f-a) ----------
static const uint8_t  IMU_ADDR              = 0x68;
static const float    GYRO_LSB_PER_DPS      = 131.0f;    // ±250 °/s
static const float    ACCEL_LSB_PER_G       = 16384.0f;  // ±2 g
static const float    G_MPS2                = 9.80665f;
static const uint32_t IMU_PERIOD_MS         = 10;        // 100 Hz
static const uint32_t IMU_CALIB_MS          = 3000;      // Stillstand beim Start
static const float    IMU_RECAL_MAX_DPS     = 5.0f;      // max. Änderung des Nullpunkts bei /imu_reset
static const uint32_t IMU_PUB_PERIOD_MS     = 50;        // 20 Hz
static const uint32_t I2C_WAIT_MS           = 20;        // max. Wartezeit auf den Bus

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
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
static char imu_frame_id[] = "imu_link";
rcl_subscription_t buzzer_sub;
std_msgs__msg__UInt16 buzzer_msg;
rcl_subscription_t state_sub;
std_msgs__msg__UInt8 state_msg;
rcl_subscription_t imu_reset_sub;
std_msgs__msg__UInt8 imu_reset_msg;
rcl_publisher_t diag_pub;
rcl_publisher_t bumper_pub;
std_msgs__msg__UInt8 bumper_msg;
geometry_msgs__msg__Vector3 diag_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ---------- Zustand ----------
volatile float cmdV = 0.0f;     // m/s; volatile: wird auch von bumperTask gelesen
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

// ---------- Anzeige (gemeinsame Daten loop() <-> Display-Task) ----------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

struct DisplayData {
  float    rangeM;          // letzter Messwert (inf/-inf/nan möglich)
  uint8_t  hostState;       // 0 = unbekannt, 1 = FREI, 2 = HINDERNIS, 3 = keine Sensordaten
  uint32_t hostStateMs;     // Empfangszeit des Zustands
  uint16_t buzzerPeriod;    // 0 aus, 1 Dauerton, sonst ms
  int16_t  wheelLeft;       // vorzeichenbehafteter param
  int16_t  wheelRight;
  bool     cmdTimedOut;
  uint8_t  bumper;          // Bit 0 links, Bit 1 rechts (gedrückt)
  uint32_t loopMaxUs;       // aus der letzten Diagnoseperiode
  uint32_t ctrlMaxUs;
};
DisplayData dispData = {NAN, 0, 0, 0, 0, 0, true, 0, 0, 0};

// IMU-Daten: geschrieben von imuTask, gelesen von loop() und Display
struct ImuData {
  bool     valid;
  float    yawDeg;          // integrierter Gierwinkel seit Start [°], gegen Uhrzeigersinn positiv
  float    gx, gy, gz;      // Drehraten, nullpunktkorrigiert [°/s]
  float    ax, ay, az;      // Beschleunigung [g]
  int64_t  stampNs;         // Zeitstempel (Epoch, nach Zeitabgleich) der Messung
  uint32_t maxDtUs;         // größter Abtastabstand seit letzter Abfrage
  uint32_t errors;          // I2C-Lesefehler gesamt
  bool     calibrating;     // v7.3: Nullpunktmessung nach /imu_reset läuft
};
ImuData imuData = {false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false};
SemaphoreHandle_t imuMutex = NULL;   // schützt imuData
SemaphoreHandle_t i2cMutex = NULL;   // schützt den I2C-Bus (Display + IMU)
TaskHandle_t imuTaskHandle = NULL;
float gyroBias[3] = {0, 0, 0};       // °/s (nur setup() und imuTask)
volatile uint8_t imuResetRequest = 0;   // v7.3: 0 keine, 1 nur Winkel 0, 2 Nullpunkt + Winkel 0 (loop -> imuTask)
volatile bool wheelsStopped = true;     // v7.3: beide Räder param 0 (controlStep -> imuTask)
SemaphoreHandle_t dispMutex = NULL;   // schützt dispData
TaskHandle_t displayTaskHandle = NULL;

// Diagnose (nur in loop() benutzt)
uint32_t diagLoopMaxUs = 0;
uint32_t diagCtrlMaxUs = 0;
uint32_t diagCtrlMinUs = UINT32_MAX;
uint32_t lastCtrlUs = 0;
uint32_t lastDiagMs = 0;

// ---------- Motor-UART ----------
SemaphoreHandle_t uartMutex = NULL;     // Motor-UART: loop() und bumperTask senden (v6)

void sendMotorFrame(uint8_t id, uint8_t mode, uint8_t dir, uint16_t param) {
  uint8_t f[6] = {START_BYTE, id, mode, dir,
                  (uint8_t)(param >> 8), (uint8_t)(param & 0xFF)};  // Big-Endian
  if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);   // Paket nie zerstückeln
  Serial2.write(f, sizeof(f));
  Serial2.flush();
  if (uartMutex) xSemaphoreGive(uartMutex);
}

// Bumper-Zustand: geschrieben nur von bumperTask, gelesen von loop() (8 Bit -> atomar)
volatile uint8_t bumperState = 0;
SemaphoreHandle_t bumperSem = NULL;     // binäre Semaphore: ISR -> bumperTask
TaskHandle_t bumperTaskHandle = NULL;
volatile uint32_t bumperIsrUs = 0;      // Zeitpunkt der letzten Flanke (für Reaktionszeit)

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

// Einfache Textanzeige (bis zu 4 Zeilen). Nur aufrufen, solange die Display-Task NICHT läuft
// (in setup() bzw. error_loop vor dem Task-Start), sonst greifen zwei Kontexte auf I2C zu.
void showScreen(const char *l1, const char *l2, const char *l3, const char *l4) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  if (l1) oled.drawStr(0, 10, l1);
  if (l2) oled.drawStr(0, 24, l2);
  if (l3) oled.drawStr(0, 38, l3);
  if (l4) oled.drawStr(0, 52, l4);
  oled.sendBuffer();
}

void error_loop(int line) {
  stopMotors();
  setBuzzer(false);
  DBG("FEHLER: micro-ROS-Aufruf in Zeile %d fehlgeschlagen (Agent erreichbar?). Neustart mit EN.\n", line);
  // Alle RCCHECK-Aufrufe liegen in setup() vor dem Start der Display-Task -> direkter Zugriff zulässig
  if (displayTaskHandle == NULL) {
    char buf[24];
    snprintf(buf, sizeof(buf), "micro-ROS Zeile %d", line);
    showScreen("FEHLER", buf, "Agent erreichbar?", "Agent starten, EN");
  }
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
    float v = cmdV;
    if (bumperState != 0 && v > 0.0f) {
      v = 0.0f;                             // lokale Schutzfunktion: Bumper gedrückt -> nicht vorwärts
    }
    vl = v - cmdW * TRACK_WIDTH_M / 2.0f;
    vr = v + cmdW * TRACK_WIDTH_M / 2.0f;
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
  wheelsStopped = (l.param == 0) && (r.param == 0);

  wheel_cmd_msg.x = signedParam(l, DIR_FWD_LEFT);
  wheel_cmd_msg.y = signedParam(r, DIR_FWD_RIGHT);
  wheel_cmd_msg.z = timedOut ? 1.0 : 0.0;
  if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    dispData.wheelLeft = (int16_t)wheel_cmd_msg.x;
    dispData.wheelRight = (int16_t)wheel_cmd_msg.y;
    dispData.cmdTimedOut = timedOut;
    xSemaphoreGive(dispMutex);
  }
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
  if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(2)) == pdTRUE) {   // kurz halten, nie lange warten
    dispData.rangeM = rangeM;
    xSemaphoreGive(dispMutex);
  }
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
  if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    dispData.buzzerPeriod = msg->data;
    xSemaphoreGive(dispMutex);
  }
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

// /robot_state vom Host (obstacle_stop): Zustandswechsel weckt die Display-Task sofort
// v7.3: Anfrage nur vormerken; Ausführung und Stillstandsprüfung in imuTask
void imu_reset_callback(const void *msgin) {
  const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msgin;
  if (imuResetRequest == 0) {
    imuResetRequest = msg->data ? 2 : 1;
    DBG("IMU-Reset angefordert (%s)\n", msg->data ? "Nullpunkt + Winkel" : "nur Winkel");
  }
}

void state_callback(const void *msgin) {
  const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msgin;
  bool changed = false;
  if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    changed = (dispData.hostState != msg->data);
    dispData.hostState = msg->data;
    dispData.hostStateMs = millis();
    xSemaphoreGive(dispMutex);
  }
#if DISPLAY_IN_TASK
  if (changed && displayTaskHandle != NULL) {
    xTaskNotifyGive(displayTaskHandle);      // Ereignis -> Task wacht sofort auf
  }
#endif
}

// Zeichnet ein Bild aus einer KOPIE der Daten (kein Mutex während der Übertragung)
void drawDisplay(const DisplayData &d) {
  char line[32];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);

  // Zeile 1: Abstand
  if (isnan(d.rangeM)) {
    snprintf(line, sizeof(line), "Abstand: --");
  } else if (isinf(d.rangeM)) {
    snprintf(line, sizeof(line), "Abstand: %s", d.rangeM > 0 ? "frei (>4m)" : "zu nah");
  } else {
    snprintf(line, sizeof(line), "Abstand: %.3f m", d.rangeM);
  }
  oled.drawStr(0, 10, line);

  // Zeile 2: Zustand vom Host (Auswertung auf dem Host)
  const char *st = "--";
  bool fresh = (millis() - d.hostStateMs) <= ROBOT_STATE_TIMEOUT_MS;
  if (fresh) {
    switch (d.hostState) {
      case 1: st = "FREI"; break;
      case 2: st = "HINDERNIS"; break;
      case 3: st = "keine Sensordaten"; break;
      case 4: st = "BUMPER"; break;
      default: st = "?"; break;
    }
  }
  snprintf(line, sizeof(line), "Host: %s", st);
  oled.drawStr(0, 22, line);
  if (fresh && (d.hostState == 2 || d.hostState == 4)) {
    oled.drawFrame(0, 13, 128, 11);          // Hindernis hervorheben
  }

  // Zeile 3: Warnstufe (Buzzer)
  if (d.buzzerPeriod == 0) {
    snprintf(line, sizeof(line), "Warnung: aus");
  } else if (d.buzzerPeriod == 1) {
    snprintf(line, sizeof(line), "Warnung: Dauerton");
  } else {
    snprintf(line, sizeof(line), "Warnung: %u ms", d.buzzerPeriod);
  }
  oled.drawStr(0, 34, line);

  // Zeile 4: Bumper (hat Vorrang) bzw. Radbefehle
  if (d.bumper != 0) {
    snprintf(line, sizeof(line), "BUMPER %s!",
             d.bumper == 3 ? "BEIDE" : (d.bumper == 1 ? "LINKS" : "RECHTS"));
    oled.drawBox(0, 37, 128, 11);
    oled.setDrawColor(0);                   // invertierte Schrift
    oled.drawStr(2, 46, line);
    oled.setDrawColor(1);
    line[0] = '\0';
  } else if (d.cmdTimedOut) {
    snprintf(line, sizeof(line), "Rad: Stopp (Timeout)");
  } else {
    snprintf(line, sizeof(line), "Rad L %+d R %+d", d.wheelLeft, d.wheelRight);
  }
  if (line[0] != '\0') {
    oled.drawStr(0, 46, line);
  }

  // Zeile 5: Gierwinkel (IMU) und Laufzeitdiagnose
  {
    ImuData im;
    bool ok = false;
    if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      im = imuData;
      ok = true;
      xSemaphoreGive(imuMutex);
    }
    if (ok && im.valid && im.calibrating) {
      snprintf(line, sizeof(line), "IMU kalibriert...");
    } else if (ok && im.valid) {
      // Nur für die Anzeige auf 0…360° abbilden (fmodf liefert für negative Winkel negative Reste)
      float shown = fmodf(im.yawDeg, 360.0f);
      if (shown < 0.0f) shown += 360.0f;
      if (shown >= 359.95f || shown == 0.0f) shown = 0.0f;  // "360.0" (Rundung) und "-0.0" vermeiden
      snprintf(line, sizeof(line), "Gier %5.1f  L%3.0fms", shown, d.loopMaxUs / 1000.0f);
    } else {
      snprintf(line, sizeof(line), "IMU --  L%3.0fms", d.loopMaxUs / 1000.0f);
    }
  }
  oled.drawStr(0, 60, line);

  // Seitenweise Übertragung (8 Zeilen à 8 Pixel = 128 Byte, je ca. 4 ms), Bus zwischen den Seiten frei
  for (uint8_t row = 0; row < 8; row++) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      oled.updateDisplayArea(0, row, 16, 1);
      xSemaphoreGive(i2cMutex);
    }
  }
}

void copyDisplayData(DisplayData &out) {
  if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    out = dispData;                          // nur kopieren, dann sofort freigeben
    xSemaphoreGive(dispMutex);
  }
}

#if DISPLAY_IN_TASK
// FreeRTOS-Task auf Kern 0: einziger Nutzer von I2C/U8g2
void displayTask(void *arg) {
  (void)arg;
  DisplayData local = dispData;
  for (;;) {
    // blockiert ohne CPU-Last bis Ereignis (Notification) oder Ablauf der Periode
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    copyDisplayData(local);
    drawDisplay(local);
  }
}
#endif

void setupDisplay() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  oled.setI2CAddress(DISPLAY_ADDR_7BIT * 2);   // U8g2 erwartet 8-Bit-Schreibweise
  oled.setBusClock(DISPLAY_BUS_HZ);
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.sendBuffer();
  showScreen("robot_base v7.3", "verbinde WLAN", "und Agent ...", "Bumper L erst danach!");
}

// ---------- Bumper: Interrupt -> Semaphore -> Task (v6) ----------
void IRAM_ATTR bumperIsr() {
  bumperIsrUs = micros();
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(bumperSem, &woken);   // weckt bumperTask, sonst nichts in der ISR
  if (woken) {
    portYIELD_FROM_ISR();
  }
}

void bumperTask(void *arg) {
  (void)arg;
  const int pins[2] = {BUMPER_LEFT_PIN, BUMPER_RIGHT_PIN};
  bool pressed[2] = {false, false};
  uint32_t lastLowMs[2] = {0, 0};
  uint8_t lastState = 0xFF;                 // erzwingt erste Meldung
  for (;;) {
    // Warten ohne CPU-Last: Flanke (Semaphore) oder Zeitablauf. Solange ein Bumper gedrückt ist,
    // kurz pollen, um das Loslassen nach BUMPER_RELEASE_MS zu erkennen; sonst seltene Kontrolle.
    xSemaphoreTake(bumperSem, pdMS_TO_TICKS(bumperState ? 20 : 200));
    uint32_t now = millis();
    for (int i = 0; i < 2; i++) {
      if (digitalRead(pins[i]) == LOW) {    // gedrückt (oder abgezogen) -> sofort
        pressed[i] = true;
        lastLowMs[i] = now;
      } else if (pressed[i] && now - lastLowMs[i] >= BUMPER_RELEASE_MS) {
        pressed[i] = false;                 // erst nach stabilem HIGH frei
      }
    }
    uint8_t state = (pressed[0] ? 1 : 0) | (pressed[1] ? 2 : 0);
    if (state == lastState) {
      continue;
    }
    bool newPress = (state & ~lastState & 0x03) != 0 && lastState != 0xFF;
    lastState = state;
    bumperState = state;
    if (newPress && cmdV > 0.0f) {
      // sofortiger lokaler Stopp, ohne auf den nächsten Regelschritt zu warten
      sendMotorFrame(ID_LEFT, MODE_CONT, 0, 0);
      sendMotorFrame(ID_RIGHT, MODE_CONT, 0, 0);
      DBG("Bumper %u -> Stopp, %lu us nach Flanke\n", state, (unsigned long)(micros() - bumperIsrUs));
    } else {
      DBG("Bumper-Zustand %u\n", state);
    }
    if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      dispData.bumper = state;
      xSemaphoreGive(dispMutex);
    }
#if DISPLAY_IN_TASK
    if (displayTaskHandle != NULL) {
      xTaskNotifyGive(displayTaskHandle);   // Anzeige sofort aktualisieren
    }
#endif
  }
}

// /bumper veröffentlichen (nur aus loop(): micro-ROS hat genau einen Nutzer)
uint8_t lastPubBumper = 0xFF;
uint32_t lastBumperPubMs = 0;
void bumperPublishStep() {
  uint8_t st = bumperState;
  uint32_t now = millis();
  if (st != lastPubBumper || now - lastBumperPubMs >= BUMPER_PUB_PERIOD_MS) {
    bumper_msg.data = st;
    RCSOFTCHECK(rcl_publish(&bumper_pub, &bumper_msg, NULL));
    lastPubBumper = st;
    lastBumperPubMs = now;
  }
}

// ---------- IMU (v7) ----------
// Registerzugriffe: nur unter i2cMutex bzw. vor dem Start der Tasks aufrufen
bool imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool imuReadRaw(int16_t v[7]) {              // ax ay az temp gx gy gz
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)IMU_ADDR, 14) != 14) return false;
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

// Initialisierung und Nullpunkt (in setup(), vor dem Start der Tasks, Roboter muss stillstehen)
bool setupImu() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)IMU_ADDR, 1) != 1) {
    DBG("IMU: keine Antwort\n");
    return false;
  }
  uint8_t who = Wire.read();
  bool ok = imuWrite(0x6B, 0x00);            // wecken
  delay(100);
  ok &= imuWrite(0x6B, 0x01);                // Takt vom Gyroskop (PLL)
  ok &= imuWrite(0x1A, 0x03);                // DLPF_CFG 3
  ok &= imuWrite(0x1B, 0x00);                // ±250 °/s
  ok &= imuWrite(0x1C, 0x00);                // ±2 g
  delay(100);
  showScreen("IMU kalibriert ...", "Roboter NICHT", "bewegen (3 s)", NULL);
  double sum[3] = {0, 0, 0};
  int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < IMU_CALIB_MS) {
    int16_t v[7];
    if (imuReadRaw(v)) {
      for (int i = 0; i < 3; i++) sum[i] += v[4 + i] / GYRO_LSB_PER_DPS;
      n++;
    }
    delay(5);
  }
  if (n > 0) {
    for (int i = 0; i < 3; i++) gyroBias[i] = (float)(sum[i] / n);
  }
  DBG("IMU: WHO_AM_I 0x%02X, Konfiguration %s, Nullpunkt gz %+.4f °/s (n = %d)\n",
      who, ok ? "ok" : "FEHLER", gyroBias[2], n);
  return ok && n > 0;
}

// Periodische Task: feste Abtastrate mit vTaskDelayUntil (kein Drift)
void imuTask(void *arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastUs = micros();
  float yaw = 0.0f;
  bool calibrating = false;                  // v7.3
  double csum[3] = {0, 0, 0};
  int cn = 0;
  uint32_t cstart = 0;
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    int16_t v[7];
    bool ok = false;
    uint32_t nowUs = 0;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_WAIT_MS)) == pdTRUE) {
      nowUs = micros();                      // Zeitpunkt der Messung (nach Erhalt des Busses)
      ok = imuReadRaw(v);
      xSemaphoreGive(i2cMutex);
    }
    if (!ok) {
      if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        imuData.errors++;
        xSemaphoreGive(imuMutex);
      }
      continue;
    }
    uint32_t dtUs = nowUs - lastUs;
    lastUs = nowUs;

    // v7.3: Rücksetzen / Nullpunkt neu messen (nur im Stillstand)
    uint8_t req = imuResetRequest;
    if (req != 0 && !calibrating) {
      imuResetRequest = 0;
      if (!wheelsStopped) {
        DBG("IMU-Reset verworfen: Roboter faehrt\n");
      } else if (req == 1) {
        yaw = 0.0f;
        DBG("IMU: Gierwinkel auf 0 gesetzt\n");
      } else {
        calibrating = true;
        csum[0] = csum[1] = csum[2] = 0.0;
        cn = 0;
        cstart = millis();
      }
    } else if (req != 0) {
      imuResetRequest = 0;                   // weitere Anfragen während der Messung ignorieren
    }
    if (calibrating) {
      if (!wheelsStopped) {
        calibrating = false;
        DBG("IMU-Nullpunktmessung abgebrochen: Roboter faehrt (alter Nullpunkt bleibt)\n");
      } else {
        for (int i = 0; i < 3; i++) csum[i] += v[4 + i] / GYRO_LSB_PER_DPS;
        cn++;
        if (millis() - cstart >= IMU_CALIB_MS && cn > 0) {
          float nb[3];
          for (int i = 0; i < 3; i++) nb[i] = (float)(csum[i] / cn);
          if (fabsf(nb[2] - gyroBias[2]) < IMU_RECAL_MAX_DPS) {
            DBG("IMU: Nullpunkt gz %+.4f -> %+.4f °/s (n = %d), Gierwinkel 0\n", gyroBias[2], nb[2], cn);
            for (int i = 0; i < 3; i++) gyroBias[i] = nb[i];
            yaw = 0.0f;
          } else {
            DBG("IMU: neuer Nullpunkt %+.3f °/s unplausibel (alt %+.3f) – verworfen\n", nb[2], gyroBias[2]);
          }
          calibrating = false;
        }
      }
    }

    float gx = v[4] / GYRO_LSB_PER_DPS - gyroBias[0];
    float gy = v[5] / GYRO_LSB_PER_DPS - gyroBias[1];
    float gz = v[6] / GYRO_LSB_PER_DPS - gyroBias[2];
    if (dtUs < 100000 && !calibrating) {     // nach langer Pause bzw. während der Messung nicht integrieren
      yaw += gz * dtUs * 1e-6f;
    }
    int64_t stamp = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : 0;
    if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      imuData.valid = true;
      imuData.yawDeg = yaw;
      imuData.gx = gx; imuData.gy = gy; imuData.gz = gz;
      imuData.ax = v[0] / ACCEL_LSB_PER_G;
      imuData.ay = v[1] / ACCEL_LSB_PER_G;
      imuData.az = v[2] / ACCEL_LSB_PER_G;
      imuData.stampNs = stamp;
      imuData.calibrating = calibrating;
      if (dtUs > imuData.maxDtUs) imuData.maxDtUs = dtUs;
      xSemaphoreGive(imuMutex);
    }
  }
}

void setupImuMsg() {
  imu_msg.header.frame_id.data = imu_frame_id;
  imu_msg.header.frame_id.size = strlen(imu_frame_id);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);
  // Kovarianzen: Roll/Nick nicht geschätzt -> sehr groß; Gier und Raten grob aus Stufe 11f-a
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.orientation_covariance[0] = 1e6;
  imu_msg.orientation_covariance[4] = 1e6;
  imu_msg.orientation_covariance[8] = 1e-3;                 // Annahme (Drift ~0,3 °/min)
  double gyroVar = pow(0.1 * M_PI / 180.0, 2);               // Std 0,1 °/s (gemessen)
  imu_msg.angular_velocity_covariance[0] = gyroVar;
  imu_msg.angular_velocity_covariance[4] = gyroVar;
  imu_msg.angular_velocity_covariance[8] = gyroVar;
}

// /imu veröffentlichen (nur aus loop())
uint32_t lastImuPubMs = 0;
uint32_t lastImuDiagMs = 0;
void imuPublishStep() {
  uint32_t now = millis();
  if (now - lastImuPubMs < IMU_PUB_PERIOD_MS) return;
  lastImuPubMs = now;
  ImuData im;
  if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(2)) != pdTRUE) return;
  im = imuData;
  bool printDiag = (now - lastImuDiagMs >= 10000);
  if (printDiag) {
    imuData.maxDtUs = 0;                     // Maximum für die nächsten 10 s neu bestimmen
  }
  xSemaphoreGive(imuMutex);
  if (printDiag) {
    lastImuDiagMs = now;
    DBG("IMU: Gier %+.2f°, größter Abtastabstand %lu us (Soll %lu), Lesefehler %lu\n",
        im.yawDeg, (unsigned long)im.maxDtUs, (unsigned long)(IMU_PERIOD_MS * 1000),
        (unsigned long)im.errors);
  }
  if (!im.valid) return;
  imu_msg.header.stamp.sec = (int32_t)(im.stampNs / 1000000000LL);
  imu_msg.header.stamp.nanosec = (uint32_t)(im.stampNs % 1000000000LL);
  double yawRad = im.yawDeg * M_PI / 180.0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = sin(yawRad / 2.0);
  imu_msg.orientation.w = cos(yawRad / 2.0);
  imu_msg.angular_velocity.x = im.gx * M_PI / 180.0;
  imu_msg.angular_velocity.y = im.gy * M_PI / 180.0;
  imu_msg.angular_velocity.z = im.gz * M_PI / 180.0;
  imu_msg.linear_acceleration.x = im.ax * G_MPS2;
  imu_msg.linear_acceleration.y = im.ay * G_MPS2;
  imu_msg.linear_acceleration.z = im.az * G_MPS2;
  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
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
  serialMutex = xSemaphoreCreateMutex();
  uartMutex = xSemaphoreCreateMutex();
  bumperSem = xSemaphoreCreateBinary();

  // Motoren zuerst in definierten Zustand bringen
  Serial2.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
  stopMotors();

  // Ultraschall-Pins
  pinMode(US_TRIG_PIN, OUTPUT);
  digitalWrite(US_TRIG_PIN, LOW);
  pinMode(US_ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);              // Buzzer definiert aus (noTone() ohne vorherigen Ton
  digitalWrite(BUZZER_PIN, LOW);            // erzeugte in v5 die LEDC-Fehlermeldung)
  pinMode(BUMPER_LEFT_PIN, INPUT);          // externe Beschaltung vorhanden (hw_probe), keine internen Pulls
  pinMode(BUMPER_RIGHT_PIN, INPUT);

  dispMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  imuMutex = xSemaphoreCreateMutex();
  setupDisplay();                           // vor dem Task-Start: nur dieser Kontext nutzt I2C
  bool imuOk = setupImu();                  // Kalibrierung: Roboter muss 3 s stillstehen
  showScreen("robot_base v7.3", imuOk ? "IMU ok" : "IMU FEHLER", "verbinde WLAN", "und Agent ...");

  setupTransport();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_base", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(STREAM_PUB_INIT(
    &wheel_cmd_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "wheel_cmd"));

  RCCHECK(STREAM_PUB_INIT(
    &range_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "range"));
  setupRangeMsg();

  RCCHECK(rclc_subscription_init_default(
    &buzzer_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    "buzzer"));

  RCCHECK(rclc_subscription_init_default(
    &state_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "robot_state"));

  RCCHECK(rclc_subscription_init_default(
    &imu_reset_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "imu_reset"));

  RCCHECK(rclc_publisher_init_default(
    &diag_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "loop_diag"));

  RCCHECK(rclc_publisher_init_default(
    &bumper_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "bumper"));

  RCCHECK(STREAM_PUB_INIT(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"));
  setupImuMsg();

  // Zeit mit dem Agent abgleichen (für Zeitstempel in /range); Fehler ist nicht kritisch
  rmw_uros_sync_session(1000);
  DBG("Zeitabgleich mit Agent: %s\n", rmw_uros_epoch_synchronized() ? "ok" : "nicht verfuegbar");

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));  // 4 Subscriber (v7.3)
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &buzzer_sub, &buzzer_msg, &buzzer_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &state_sub, &state_msg, &state_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &imu_reset_sub, &imu_reset_msg, &imu_reset_callback, ON_NEW_DATA));

  DBG("micro-ROS bereit: Node /robot_base, Subscriber /cmd_vel, /buzzer, /robot_state, /imu_reset, Publisher /wheel_cmd, /range, /loop_diag, /bumper, /imu\n");
  DBG("Sensorströme: %s\n", STREAM_BEST_EFFORT ? "best effort" : "reliable");
#if DISPLAY_IN_TASK
  // Display-Task auf Kern 0 (loop() läuft auf Kern 1), Priorität 1, 4 KB Stack
  xTaskCreatePinnedToCore(displayTask, "display", 4096, NULL, 1, &displayTaskHandle, 0);
  DBG("Display: eigene Task auf Kern 0 (loop auf Kern %d)\n", xPortGetCoreID());
#elif DISPLAY_IN_LOOP
  DBG("Display: Vergleichsvariante in loop() (Kern %d)\n", xPortGetCoreID());
#else
  showScreen("Messmodus 0:", "Display aus", "(Grundlinie 11d)", NULL);
  DBG("Display: AUS (Grundlinie, nur Startbild)\n");
#endif
  lastControlMs = millis();
  usLastTriggerMs = millis();
  attachInterrupt(digitalPinToInterrupt(US_ECHO_PIN), echoIsr, CHANGE);
  lastDiagMs = millis();

  // Bumper-Task auf Kern 0, Priorität 2 (über der Display-Task), dann Interrupts aktivieren
  xTaskCreatePinnedToCore(bumperTask, "bumper", 3072, NULL, 2, &bumperTaskHandle, 0);
  if (imuOk) {
    // IMU-Task auf Kern 0, Priorität 3 (höchste unserer Tasks: feste Abtastrate)
    xTaskCreatePinnedToCore(imuTask, "imu", 4096, NULL, 3, &imuTaskHandle, 0);
    DBG("IMU: Task auf Kern 0, %lu Hz\n", (unsigned long)(1000 / IMU_PERIOD_MS));
  }
  attachInterrupt(digitalPinToInterrupt(BUMPER_LEFT_PIN), bumperIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUMPER_RIGHT_PIN), bumperIsr, CHANGE);
  DBG("Bumper: Task auf Kern 0, Interrupts GPIO %d/%d\n", BUMPER_LEFT_PIN, BUMPER_RIGHT_PIN);
}

// Diagnose: einmal pro Sekunde Maxima/Minima veröffentlichen und zurücksetzen
void diagStep() {
  uint32_t now = millis();
  if (now - lastDiagMs < DIAG_PERIOD_MS) {
    return;
  }
  lastDiagMs = now;
  diag_msg.x = (double)diagLoopMaxUs;
  diag_msg.y = (double)diagCtrlMaxUs;
  diag_msg.z = (diagCtrlMinUs == UINT32_MAX) ? 0.0 : (double)diagCtrlMinUs;
  RCSOFTCHECK(rcl_publish(&diag_pub, &diag_msg, NULL));
  if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    dispData.loopMaxUs = diagLoopMaxUs;
    dispData.ctrlMaxUs = diagCtrlMaxUs;
    xSemaphoreGive(dispMutex);
  }
  diagLoopMaxUs = 0;
  diagCtrlMaxUs = 0;
  diagCtrlMinUs = UINT32_MAX;
}

#if DISPLAY_IN_LOOP
uint32_t lastDisplayMs = 0;
#endif

void loop() {
  uint32_t loopStartUs = micros();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs += CONTROL_PERIOD_MS;              // Soll-Takt halten (kein Drift)
    if (now - lastControlMs >= CONTROL_PERIOD_MS) {
      lastControlMs = now;                           // nach langer Blockade nicht nachholen
    }
    uint32_t nowUs = micros();
    if (lastCtrlUs != 0) {
      uint32_t dt = nowUs - lastCtrlUs;      // tatsächlicher Abstand der Regelschritte
      if (dt > diagCtrlMaxUs) diagCtrlMaxUs = dt;
      if (dt < diagCtrlMinUs) diagCtrlMinUs = dt;
    }
    lastCtrlUs = nowUs;
    controlStep();
  }

  ultrasonicStep();
  buzzerStep();
  diagStep();
  bumperPublishStep();
  imuPublishStep();

#if DISPLAY_IN_LOOP
  // Vergleichsvariante: Display blockiert loop() für die Dauer der Übertragung
  if (millis() - lastDisplayMs >= DISPLAY_PERIOD_MS) {
    lastDisplayMs = millis();
    DisplayData local = dispData;            // Vorbelegung, falls der Mutex nicht frei ist
    copyDisplayData(local);
    drawDisplay(local);
  }
#endif

  uint32_t loopUs = micros() - loopStartUs;
  if (loopUs > diagLoopMaxUs) diagLoopMaxUs = loopUs;
}
