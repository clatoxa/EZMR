# IPC & P2P Projekt in C (System V)

Diese Dokumentation beschreibt die Umsetzung der interprozessualen Kommunikation (IPC) mittels System V Mechanismen (Message Queues, Shared Memory, Semaphoren) sowie alle unterstützten Protokollbefehle.

## 1. Unterstützte Befehle (Protokoll)

Sowohl im klassischen Client-Server-Modell als auch im P2P-Modell versteht das System folgende Befehle:

* **`ADD <Zahl1> <Zahl2>`**  
  Addiert zwei Ganzzahlen und gibt das Ergebnis zurück (z. B. `ADD 4 3` $\rightarrow$ `RESULT 7`).
* **`PRIME <Zahl>`**  
  Prüft, ob die angegebene Zahl eine Primzahl ist (Antwort: `PRIME YES` oder `PRIME NO`).
* **`LIST`**  
  Ruft die gemeinsame Nachrichten-Historie ab. Jeder Eintrag zeigt transparent, wer angefragt und geantwortet hat. Die Ausgabe endet mit `LIST_END`.
* **`CLEAR`**  
  Löscht die komplette Nachrichten-Historie (Antwort: `OK`).

---

## 2. Klassisches Client-Server-Modell (`server_ipc.c` & `client_ipc.c`)

### Kompilieren & Starten
```bash
gcc server_ipc.c -o server
gcc client_ipc.c -o client

# Terminal 1: Server starten
./server

# Terminal 2: Client starten
./client
```

---

## 3. Gleichberechtigtes Peer-to-Peer-Modell (`peer_shm.c`)

### Kompilieren & Starten
```bash
gcc peer_shm.c -lpthread -o peer_shm

# Beispiel: Peer mit ID 2 starten
./peer_shm 2
```

### Ziel-Auswahl nach dem Start:
Direkt nach dem Start wählst du aus, an wen deine Nachrichten gesendet werden sollen:
* `PEER <id>` – Leitet Nachrichten an einen bestimmten Peer weiter (z. B. `PEER 1`).
* `ALL` – Schaltet auf Broadcast um (Sendet an alle aktiven Peers).

### Beispiel-Sitzung:
```text
Peer 2 gestartet
Befehle: PEER <id> | ALL | exit
[Peer 2 → ?] > PEER 1
Nachrichten gehen jetzt an Peer 1
[Peer 2 → Peer 1] > ADD 4 3
RESULT 7
```

---

## 4. Speicher-Bereinigung (Cleanup)

Falls Programme abstürzen oder unsauber beendet werden, bleiben System V Ressourcen im Kernel erhalten.

1. **Aktive Ressourcen anzeigen:**
   ```bash
   ipcs
   ```
2. **Ressourcen manuell freigeben (Beispiel-IDs):**
   ```bash
   ipcrm -q 1234   # Message Queue löschen
   ipcrm -m 5678   # Shared Memory löschen
   ipcrm -s 9012   # Semaphor löschen
   ```

### Beispiel für IPC-Bereinigung (`ipcs` und `ipcrm`)

#### 1. Aktive IPC-Ressourcen anzeigen
Führe den Befehl `ipcs` aus, um deine aktiven Queues, Shared-Memory-Segmente und Semaphoren zu sehen:

```bash
$ ipcs

------ Nachrichtenwarteschlangen --------
Schlüssel       msqid       Besitzer    Rechte      Benutzt-Bytes Nachrichten 
0x0000270f      19          clara       666         0             0           
0x000004d2      20          clara       666         0             0           
0x00000002      21          clara       666         0             0           
0x00000001      22          clara       666         0             0           

------ Gemeinsamer Speicher: Segmente --------
Schlüssel       shmid       Besitzer    Rechte      Bytes       nattch      Status      
0x000022b8      1605693     clara       666         40452       0                       

------ Semaphorenfelder --------
Schlüssel       SemID       Besitzer    Rechte      nsems       
0x00001e61      5           clara       666         1
```

#### 1. IPC-Ressourcen löschen
Message Queues löschen (-q)
```
ipcrm -q 19
ipcrm -q 20
ipcrm -q 21
ipcrm -q 22
```


Shared-Memory-Segment löschen (-m)
```
ipcrm -m 1605693
```
Semaphor löschen (-s)
```
ipcrm -s 5
```
