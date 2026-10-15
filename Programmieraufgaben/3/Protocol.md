# Dokumentation und Kommunikationsprotokoll: Netzwerk-Sockets & P2P

Dieses Dokument beschreibt den Aufbau, das Kommunikationsprotokoll sowie die schrittweise Entwicklung der Netzwerkanwendung von einem klassischen Client-Server-Modell bis hin zu einer dezentralen Peer-to-Peer-Architektur in C.

---

## 1. Architektur und Entwicklungsschritte

Die Entwicklung gliedert sich in folgende aufeinanderfolgende Phasen:
1. **Server (`socket.c`):** Grundlegendes Socket-Setup, TCP-Verbindungsannahme (`listen`/`accept`) und Implementierung der Kernbefehle inklusive zustandsbehafteter Liste.
2. **Netcat-Testumgebung:** Erste Funktionstests des Servers ohne eigenen Client mittels des Standard-Netzwerktools `netcat`.
3. **Clients (`client.c` & `client1.c`):** 
   - `client.c`: Einfacher Einmal-Client zum automatisierten Senden einer Anfrage.
   - `client1.c`: Interaktiver Client mit Kommandozeileneingabe (`> `) und Unterstützung für mehrzeilige Antworten (z. B. bei `LIST`).
4. **Peer-to-Peer-Erweiterung (`peer.c`):** Auflösung der festen Client-Server-Rollen durch Multithreading (`pthread`), sodass jede Instanz gleichzeitig Server (Lauschen auf Verbindungen) und Client (Verbinden zu anderen Peers) sein kann.

---

## 2. Server (`socket.c`) & Testen mit Netcat

Der Server läuft standardmäßig auf Port `5071` (Loopback `127.0.0.1` oder `INADDR_ANY`). Er verwaltet eine interne Liste von Anfragen und Antworten.

### Testen mit Netcat (`nc`)
Für erste Funktionstests kann der Server im Terminal gestartet und über `netcat` angesprochen werden:

1. Server kompilieren und starten:
   ```bash
   gcc socket.c -o socket
   ./socket
   ```
2. In einem zweiten Terminal Verbindung per Netcat aufbauen:
   ```bash
   nc 127.0.0.1 5071
   ```
3. Befehle (z. B. `ADD 10 20`) direkt eingeben und die Serverantwort prüfen. Zum Beenden von `nc` nutze `Ctrl + C`.

---

## 3. Kommunikationsprotokoll (Nachrichtenformat)

Die Kommunikation basiert auf reinem Text (TCP-Stream). Anfragen und Antworten sind zeilenbasiert bzw. durch Protokollkonventionen definiert.

### Anfragen (Requests)
Jede Anfrage beginnt mit einem Großbuchstaben-Kommando, gefolgt von Parametern, die durch Leerzeichen getrennt sind:
```text
<KOMMANDO> [Parameter1] [Parameter2] ...
```

### Unterstützte Kommandos

#### 1. ADD
Addiert zwei Ganzzahlen.

* **Anfrage:**  `ADD <Zahl1> <Zahl2>`
* **Beispiel:** `ADD 5 10`
* **Antwort:** `RESULT <Ergebnis>` (z. B. `RESULT 15`)

#### 2. PRIME
Prüft, ob eine Zahl eine Primzahl ist.

* **Anfrage:**  `PRIME <Zahl>`
* **Beispiel:** `PRIME 17`
* **Antwort:** `PRIME YES` oder `PRIME NO` (bei ungültigen Parametern oder Zusatzargumenten erfolgt eine entsprechende Fehlermeldung).

#### 3. LIST
Ruft die gesamte Historie aller bisherigen Anfragen und Antworten ab, die auf dem Server/Peer gespeichert sind.

* **Anfrage:** `LIST`
* **Antwort (mehrzeilig):**
  ```text
  ENTRY REQUEST: ADD 5 10 | RESPONSE: RESULT 15
  ENTRY REQUEST: PRIME 17 | RESPONSE: PRIME YES
  LIST_END
  ```
  *(Die Übertragung endet explizit mit dem Kontrollwort `LIST_END`).*

#### 4. CLEAR
Löscht die gespeicherte Nachrichten-Historie.

* **Anfrage:** `CLEAR`
* **Antwort:** `OK`

---

## 4. Clients (`client.c` & `client1.c`)

Die Client-Anwendungen sind so konfiguriert, dass sie sich automatisch mit dem lokalen Socket auf Port `5071` (`127.0.0.1`) verbinden.

### 1. Statischer Client (`client.c`)
Stellt eine Verbindung her, sendet fix den Befehl `ADD 5 10`, empfängt die Antwort und beendet sich.

* **Kompilieren & Starten:**
  ```bash
  gcc client.c -o client
  ./client
  ```

### 2. Interaktiver Client (`client1.c`)
Bietet eine interaktive Konsole (`> `) für Live-Eingaben und unterstützt mehrzeilige Antworten.

* **Kompilieren & Starten:**

  ```bash
  gcc client1.c -o client1
  ./client1
  ```
* **Beispiel-Nutzung:**
  ```text
  > ADD 20 22
  Server: RESULT 42
  > PRIME 17
  Server: PRIME YES
  > exit
  ```
  *(Mit dem Befehl `exit` wird der Client beendet).*

---

## 5. Peer-to-Peer-Erweiterung (`peer.c`)

In der P2P-Variante gibt es keine strikte Trennung mehr zwischen Client und Server.

### Funktionsweise & Befehle
1. **Start:** Jede Instanz wird mit einem eigenen Port gestartet (z. B. Port `5050` oder `5071`):
   ```bash
   gcc peer.c -lpthread -o peer
   ./peer 5050
   ```
2. **Eingehende Verbindungen:** Ein eigener Hintergrund-Thread (`accept_connections`) akzeptiert dauerhaft eingehende Verbindungen von anderen Peers und startet für jeden verbundenen Peer einen eigenen Handler-Thread (`handle_peer`).
3. **Ausgehende Verbindungen:** Über das Konsolenkommando `CONNECT <Port>` baut der Peer aktiv eine TCP-Verbindung zu anderen Peers auf.
4. **Protokoll-Erweiterung im P2P-Modus:**
   Damit der empfangene Peer weiß, von welchem Port die Nachricht stammt, wird jeder ausgehenden Nachricht automatisch der eigene Port vorangestellt:
   ```text
   PORT <sender_port> <KOMMANDO> [Parameter]
   ```
   Der empfangende Peer extrahiert diesen Port und speichert ihn in der gemeinsamen Historie ab.

### Verfügbare Konsolenkommandos im Peer
Sobald der Peer läuft, stehen dir in der Konsole (`> `) folgende Befehle zur Verfügung:

* **`CONNECT <Port>`**
  Baut eine aktive TCP-Verbindung zu einem anderen Peer auf derselben Maschine auf (z. B. `CONNECT 5051`).
* **`ADD <Zahl1> <Zahl2>`**
  Sendet eine Additionsaufgabe an den verbundenen Peer.
* **`PRIME <Zahl>`**
  Prüft, ob eine Zahl eine Primzahl ist.
* **`LIST`**
  Fragt die gemeinsame Historie ab. Jeder Eintrag zeigt transparent, welcher Peer (welcher Port) die Anfrage gestellt und welcher sie beantwortet hat.
* **`CLEAR`**
  Löscht die komplette Nachrichten-Historie des aktuellen Peers.
* **`exit`**
  Beendet das Peer-Programm.
