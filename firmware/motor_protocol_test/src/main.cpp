// Stufe 7: Testkonsole für das UART-Protokoll der ATtiny212-Motorcontroller.
// Kein micro-ROS: Serial (USB) dient als Eingabekonsole.
//
// Paketformat laut Anleitung 4.2.2: 0xAA, id, mode, dir, param (uint16).
// Durch Versuche ermittelt (07.10.2026, Sichtprüfung/Handstoppung, Räder frei).
// Konvention: vorne = Ultraschallsensor; links/rechts von hinten in Fahrtrichtung.
//   - UART:   9600 Baud, 8N1 (Format angenommen, funktioniert), nur TX an GPIO 17
//   - id:     1 = linkes Rad, 2 = rechtes Rad, 0 = keine Reaktion
//   - dir:    links 1 = vorwärts, rechts 0 = vorwärts (Motoren spiegelbildlich)
//   - param:  Big-Endian (High-Byte zuerst)
//   - mode 0: feste Schrittzahl; 2048 Schritte = 1 Radumdrehung;
//             Befehle werden gepuffert und nacheinander ausgeführt (nicht abbrechbar)
//   - mode 1: Dauerbetrieb; param = Wartezeit pro Schritt in ms (kleiner = schneller);
//             neuer mode-1-Befehl ersetzt den laufenden sofort; param 0 = Stopp
//   - Kein eigener Timeout im ATtiny beobachtet (> 20 s Lauf ohne neuen Befehl)
//
// Sicherheitsprüfungen: Grenze 4200 für param in BEIDEN Byte-Reihenfolgen,
// mode nur 0/1, dir nur 0/1. Es wird NUR auf ausdrücklichen Befehl gesendet.
// Not-Aus: USB-Kabel trennen (Motoren werden auch über USB versorgt).
#include <Arduino.h>

static const int MOTOR_TX_PIN = 17;     // laut Anleitung 4.2.3 (nur TX benutzt)
static const int MOTOR_RX_PIN = 16;     // laut Anleitung nicht belegt; nur, weil begin() einen RX-Pin erwartet
static const uint8_t START_BYTE = 0xAA;
static const uint16_t MAX_STEPS = 4200; // Obergrenze für param (Sicherheitsgrenze)
static const uint8_t ID_LEFT = 1;       // ermittelt
static const uint8_t ID_RIGHT = 2;      // ermittelt

uint32_t motorBaud = 9600;              // ermittelt: 9600 Baud
bool paramBigEndian = true;             // bestätigt: High-Byte zuerst
bool uartStarted = false;

char line[96];
size_t lineLen = 0;
bool lineInvalid = false;               // Zeile enthielt unzulässige Zeichen

void startMotorUart(uint32_t baud) {
  if (uartStarted) {
    Serial2.flush();
    Serial2.end();
  }
  Serial2.begin(baud, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
  uartStarted = true;
  motorBaud = baud;
  Serial.printf("Motor-UART: %lu Baud, 8N1, TX=GPIO%d\n", (unsigned long)motorBaud, MOTOR_TX_PIN);
}

// Prüft ein vollständiges 6-Byte-Paket, bevor es gesendet wird.
// Gibt true zurück, wenn es zulässig ist.
bool frameIsSafe(const uint8_t *f) {
  if (f[0] != START_BYTE) {
    Serial.println("Abgelehnt: Paket muss mit AA beginnen");
    return false;
  }
  if (f[2] > 1) {
    Serial.println("Abgelehnt: nur mode 0 und mode 1 bekannt");
    return false;
  }
  if (f[3] > 1) {
    Serial.println("Abgelehnt: dir nur 0 oder 1");
    return false;
  }
  // Grenze in beiden Lesarten prüfen: schützt auch bei vertauschter Byte-Reihenfolge
  uint16_t asLe = (uint16_t)f[4] | ((uint16_t)f[5] << 8);
  uint16_t asBe = ((uint16_t)f[4] << 8) | (uint16_t)f[5];
  if (asLe > MAX_STEPS || asBe > MAX_STEPS) {
    Serial.printf("Abgelehnt: param als le=%u, als be=%u Schritte; Grenze %u in beiden Lesarten\n",
                  asLe, asBe, MAX_STEPS);
    return false;
  }
  return true;
}

void sendBytes(const uint8_t *buf, size_t n) {
  Serial2.write(buf, n);
  Serial2.flush();                      // warten, bis alle Bytes hinausgeschoben sind
  Serial.printf("TX @%lu Baud:", (unsigned long)motorBaud);
  for (size_t i = 0; i < n; i++) {
    Serial.printf(" %02X", buf[i]);
  }
  Serial.println();
}

void sendFrame(uint8_t id, uint8_t mode, uint8_t dir, uint16_t param) {
  uint8_t f[6];
  f[0] = START_BYTE;
  f[1] = id;
  f[2] = mode;
  f[3] = dir;
  if (paramBigEndian) {
    f[4] = (uint8_t)(param >> 8);
    f[5] = (uint8_t)(param & 0xFF);
  } else {
    f[4] = (uint8_t)(param & 0xFF);
    f[5] = (uint8_t)(param >> 8);
  }
  if (frameIsSafe(f)) {
    sendBytes(f, sizeof(f));
  }
}

void printStatus() {
  Serial.printf("Status: %lu Baud, Byte-Reihenfolge param: %s, param-Grenze %u, mode 0/1\n",
                (unsigned long)motorBaud, paramBigEndian ? "be (High zuerst)" : "le (Low zuerst)",
                MAX_STEPS);
}

void printHelp() {
  Serial.println("Befehle:");
  Serial.println("  send <id> <mode> <dir> <param>   z.B. send 1 0 0 1028");
  Serial.println("  baud <rate>                      z.B. baud 9600");
  Serial.println("  order le | order be              Byte-Reihenfolge von param");
  Serial.println("  raw <6 Hex-Bytes>                z.B. raw AA 01 00 00 04 04");
  Serial.println("  stop                             beide Motoren: mode 1, param 0");
  Serial.println("  status | help");
  Serial.println("Ermittelt: id 1 = links (vorwaerts dir 1), id 2 = rechts (vorwaerts dir 0)");
  Serial.println("mode 0: param = Schritte (2048 = 1 Umdrehung), mode 1: param = ms/Schritt, 0 = Stopp");
}

void handleLine(char *s) {
  // Zeigt genau den (bereinigten) Text, der im Folgenden ausgewertet wird
  Serial.printf("RX: '%s'\n", s);
  long a, b, c, d;

  if (sscanf(s, "send %ld %ld %ld %ld", &a, &b, &c, &d) == 4) {
    if (a < 0 || a > 255 || b < 0 || b > 255 || c < 0 || c > 255 || d < 0 || d > 65535) {
      Serial.println("Fehler: id/mode/dir 0..255, param 0..65535");
      return;
    }
    sendFrame((uint8_t)a, (uint8_t)b, (uint8_t)c, (uint16_t)d);
  } else if (sscanf(s, "baud %ld", &a) == 1) {
    if (a < 1200 || a > 2000000) {
      Serial.println("Fehler: Baudrate 1200..2000000");
      return;
    }
    startMotorUart((uint32_t)a);
  } else if (strcmp(s, "order le") == 0) {
    paramBigEndian = false;
    printStatus();
  } else if (strcmp(s, "order be") == 0) {
    paramBigEndian = true;
    printStatus();
  } else if (strncmp(s, "raw", 3) == 0) {
    uint8_t buf[6];
    size_t n = 0;
    char *tok = strtok(s + 3, " ");
    while (tok != NULL) {
      if (n >= sizeof(buf)) {
        Serial.println("Abgelehnt: raw erlaubt genau 6 Bytes");
        return;
      }
      char *end;
      long v = strtol(tok, &end, 16);
      if (*end != '\0' || v < 0 || v > 255) {
        Serial.printf("Fehler: ungueltiges Byte '%s'\n", tok);
        return;
      }
      buf[n++] = (uint8_t)v;
      tok = strtok(NULL, " ");
    }
    if (n != sizeof(buf)) {
      Serial.println("Abgelehnt: raw erlaubt genau 6 Bytes");
      return;
    }
    if (frameIsSafe(buf)) {
      sendBytes(buf, n);
    }
  } else if (strcmp(s, "stop") == 0) {
    sendFrame(ID_LEFT, 1, 0, 0);
    sendFrame(ID_RIGHT, 1, 0, 0);
  } else if (strcmp(s, "status") == 0) {
    printStatus();
  } else if (strcmp(s, "help") == 0) {
    printHelp();
  } else {
    Serial.println("Unbekannter Befehl (help fuer Hilfe)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Motorprotokoll-Testkonsole (Stufe 7, abgesichert) ===");
  startMotorUart(motorBaud);            // Leitung sofort definiert auf Ruhepegel
  printStatus();
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (lineInvalid) {
        Serial.println("Zeile verworfen: unzulaessiges Zeichen (bitte neu eingeben)");
      } else if (lineLen > 0) {
        line[lineLen] = '\0';
        handleLine(line);
      }
      lineLen = 0;
      lineInvalid = false;
    } else if (ch == 0x08 || ch == 0x7F) {
      // Backspace/Entfernen: letztes Zeichen aus dem Puffer löschen
      if (lineLen > 0) {
        lineLen--;
      }
    } else if ((uint8_t)ch < 0x20 || (uint8_t)ch > 0x7E) {
      // Steuerzeichen oder Nicht-ASCII (z. B. geschütztes Leerzeichen) -> Zeile ungültig
      lineInvalid = true;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = ch;
    } else {
      lineInvalid = true;               // Zeile zu lang
    }
  }
}
