# UWB Localization System - Hardware Setup Guide

## System Architecture
- **3 Anchors**: Stationary reference points (measure distance to TAG)
- **1 TAG**: Mobile device (calculates its position)

---

## Board Configuration (main/main.c Zeile 20-21)

### TAG Board:
```c
#define BOARD_MODE_TAG    1
#define ANCHOR_ID         0  // (ignored)
```

### Anchor Board 0:
```c
#define BOARD_MODE_TAG    0
#define ANCHOR_ID         0
```

### Anchor Board 1:
```c
#define BOARD_MODE_TAG    0
#define ANCHOR_ID         1
```

### Anchor Board 2:
```c
#define BOARD_MODE_TAG    0
#define ANCHOR_ID         2
```

---

## Anchor Positions (main/main.c Zeile 36-42)

**WICHTIG**: Die Positionen müssen GENAU gemessen sein!

```c
static const float g_anchor_positions[NUM_ANCHORS][3] = {
    {0.0f,  0.0f, 1.0f},   // Anchor 0: X=0m, Y=0m, Z=1m (Höhe)
    {5.0f,  0.0f, 1.0f},   // Anchor 1: X=5m, Y=0m, Z=1m
    {0.0f,  5.0f, 1.0f},   // Anchor 2: X=0m, Y=5m, Z=1m
};
```

**Beispiel-Setup**: 3 Anker an den Ecken eines 5m × 5m Raumes, alle auf 1m Höhe.

---

## Flashing Instructions

### 1. Bearbeite main.c
Setze BOARD_MODE_TAG und ANCHOR_ID entsprechend dem Board.

### 2. Kompilieren und Flashen

Öffne Terminal im Projektverzeichnis:

```bash
cd c:\Users\bolsm\Documents\GitHub\LLLL

# Board aufspüren
idf.py monitor

# Kompilieren
idf.py build

# Auf Anchor 0 flashen (z.B. COM3)
idf.py -p COM3 flash

# Auf Anchor 1 flashen (z.B. COM4)
idf.py -p COM4 flash

# Auf Anchor 2 flashen (z.B. COM5)
idf.py -p COM5 flash

# Auf TAG flashen (z.B. COM6)
idf.py -p COM6 flash
```

### 3. Serielle Ausgabe prüfen

Nach dem Flashing:
```bash
idf.py -p COMx monitor
```

---

## LED Blink Patterns

Damit kannst du erkennen, welche Boards aktiv sind:

- **TAG**: 2 kurze Blinks pro 2 Sekunden  
- **Anchor**: 1 Blink pro 2 Sekunden

---

## Erwartete Log-Ausgabe

### TAG Board:
```
[UWB_COMM] >>> BOARD CONFIGURED AS TAG <<<
[UWB_COMM] Initialized 3 anchors for localization
[UWB_COMM] Anchor 0: (0.00, 0.00, 1.00)
[UWB_COMM] Anchor 1: (5.00, 0.00, 1.00)
[UWB_COMM] Anchor 2: (0.00, 5.00, 1.00)
[UWB_COMM] === POSITION (2D) - Update #1 ===
[UWB_COMM] Position: (2.451, 2.523) m
[UWB_COMM] Error estimate: 0.045 m
```

### Anchor Board:
```
[UWB_COMM] >>> BOARD CONFIGURED AS ANCHOR #0 <<<
[UWB_COMM] === ANCHOR MODE - Board #0 ===
[UWB_COMM] Anchor position: (0.00, 0.00, 1.00)
[UWB_COMM] Initializing DW3000...
[UWB_COMM] Anchor address: 0x0081
```

---

## Troubleshooting

### "DW3000 initialization failed"
- Überprüfe DW3000 Verbindung und GPIO Pins
- Stelle sicher, dass alle Pins korrekt zugeordnet sind

### Keine Positionsberechnung auf TAG
- Mindestens 3 aktive Anker nötig
- Überprüfe LED Blink Pattern (sollten alle blinken)
- Logs auf Fehlerausgabe überprüfen

### Ungenaue Positionen
- DW3000 Calibration durchführen (antenna delay)
- Anchor-Positionen genau messen
- Ausreichend Abstand zwischen Anchors (mindestens 3m)

---

## DW3000 Calibration (Optional aber Wichtig)

Für ±10cm Genauigkeit müssen Anker **kalibriert** werden:

1. Setze ein Anchor als Reference (antenna delay = 16384)
2. Platziere Reference und zu-kalibrierenden Anchor 7m auseinander
3. Starte Auto-Calibration-Skript
4. Trage resultierende antenna delay in Code ein

(Weitere Details siehe UWB-Indoor-Localization_Arduino Repository)

---

## Performance

**In Test-Modus:**
- Positionsupdates: alle 500ms
- Genauigkeit (2D): ±5cm mit 3 Ankern
- Reichweite: bis 50m (abhängig von DW3000 Konfiguration)

