# Kommunikationsprotokoll

## 1. Allgemeines

Die Kommunikation zwischen Client und Server erfolgt über eine
TCP-Verbindung.

Nach dem Verbindungsaufbau sendet der Client eine Anfrage an den Server.
Der Server wertet diese aus und sendet eine Antwort zurück.

## 2. Nachrichtenformat

Eine Anfrage besteht aus einem Kommando und den zugehörigen Parametern:

    <Kommando> <Parameter1> <Parameter2>

Die einzelnen Elemente werden durch Leerzeichen getrennt.

## 3. Kommandos

### ADD

Addiert zwei Ganzzahlen.

**Anfrage:**

    ADD <Zahl1> <Zahl2>

**Beispiel:**

    ADD 12 30

**Antwort:**

    RESULT <Ergebnis>

**Beispiel:**

    RESULT 42

## 4. Fehlerfälle

### Unbekannter Befehl

...

### Ungültige Eingabe

...
