// Stufe 11a: Inbetriebnahme OLED-Display (128x64, I2C 0x3C) und Messung der Übertragungszeit.
// Kein micro-ROS, keine Motorbefehle. Ausgabe im seriellen Monitor (115200 Baud).
//
// Annahme: Controller SSD1306 (übliche Bauart bei 0,96"-Modulen, auf dem Modul nicht angegeben).
// Zeigt das Display verschobenes Bild/Rauschen -> USE_SH1106 auf 1 setzen und neu hochladen.
//
// Messung: Zeit für Zeichnen in den RAM-Puffer (draw) und für die Übertragung des
// vollständigen Bildes (1024 Byte) über I2C (send), abwechselnd bei 100 kHz und 400 kHz.
// Stand 08.10.2026: UNGETESTET.
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#define USE_SH1106 0

static const int I2C_SDA = 21;      // Anleitung 4.2.3
static const int I2C_SCL = 22;
static const int FRAMES_PER_SERIES = 40;

#if USE_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
static const char *CONTROLLER = "SH1106";
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
static const char *CONTROLLER = "SSD1306";
#endif

static const uint32_t CLOCKS[] = {100000, 400000};
int clockIndex = 0;
int frame = 0;
uint32_t drawSum = 0, sendSum = 0, sendMin = UINT32_MAX, sendMax = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== Display-Test (%s, I2C 0x3C) ===\n", CONTROLLER);
  Wire.begin(I2C_SDA, I2C_SCL);
  display.setI2CAddress(0x3C * 2);  // U8g2 erwartet die 8-Bit-Schreibweise (0x78)
  display.setBusClock(CLOCKS[clockIndex]);
  if (!display.begin()) {
    Serial.println("FEHLER: display.begin() fehlgeschlagen");
  }
  Serial.println("Erwartung: Rahmen, Text, laufender Zähler und wandernder Balken.");
}

void loop() {
  uint32_t t0 = micros();
  display.clearBuffer();
  display.drawFrame(0, 0, 128, 64);
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(4, 12, "Display-Test");
  display.drawStr(4, 24, CONTROLLER);
  char line[32];
  snprintf(line, sizeof(line), "I2C %lu kHz", (unsigned long)(CLOCKS[clockIndex] / 1000));
  display.drawStr(4, 36, line);
  snprintf(line, sizeof(line), "Bild %d", frame);
  display.drawStr(4, 48, line);
  display.drawBox(4 + (frame % 20) * 6, 54, 6, 6);   // wandernder Balken
  uint32_t t1 = micros();
  display.sendBuffer();                                // vollständige Übertragung
  uint32_t t2 = micros();

  uint32_t drawUs = t1 - t0;
  uint32_t sendUs = t2 - t1;
  drawSum += drawUs;
  sendSum += sendUs;
  sendMin = min(sendMin, sendUs);
  sendMax = max(sendMax, sendUs);
  frame++;

  if (frame % FRAMES_PER_SERIES == 0) {
    Serial.printf("I2C %3lu kHz: draw Mittel %4lu us | send Mittel %6lu us, min %6lu, max %6lu (n=%d)\n",
                  (unsigned long)(CLOCKS[clockIndex] / 1000),
                  (unsigned long)(drawSum / FRAMES_PER_SERIES),
                  (unsigned long)(sendSum / FRAMES_PER_SERIES),
                  (unsigned long)sendMin, (unsigned long)sendMax, FRAMES_PER_SERIES);
    drawSum = sendSum = 0;
    sendMin = UINT32_MAX;
    sendMax = 0;
    clockIndex = 1 - clockIndex;                       // Takt wechseln
    display.setBusClock(CLOCKS[clockIndex]);
  }
  delay(50);
}
