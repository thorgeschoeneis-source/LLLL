# LLLL

Neu aufgesetztes ESP-IDF-Projekt.

## Build

```bash
idf.py build
```

## Flash & Monitor

```bash
idf.py flash monitor
```
# LLLL

## ESP32-C6 UWB Abstandsmessung (2 Boards)

Im Ordner `esp32c6_uwb` liegen zwei getrennte Sketche:

- `esp32c6_uwb/anchor/anchor.ino`: laeuft auf Board A (Anchor)
- `esp32c6_uwb/tag/tag.ino`: laeuft auf Board B (Empfaenger/Tag)
- `esp32c6_uwb/uwb_config.h`: gemeinsamer Pin- und Adressblock

### Pin-Block

Die Pinzuordnung steht zentral in `esp32c6_uwb/uwb_config.h` und kann dort direkt angepasst werden.

### Bibliothek (aus dem verlinkten Repo)

Verwende die angepasste DW1000-Bibliothek aus:

`https://github.com/jremington/UWB-Indoor-Localization_Arduino`

Insbesondere den Ordner `DW1000_library` in deine Arduino-Libraries uebernehmen,
damit `DW1000Ranging` verfuegbar ist.

### Flash-Ablauf

1. `anchor.ino` auf das Anchor-Board flashen.
2. `tag.ino` auf das zweite Board flashen.
3. Auf beiden Serial-Monitor (115200) oeffnen.
4. Distanzwerte werden in Metern ausgegeben.

### Hinweise

- Die Device-Adressen (`ANCHOR_ADDR`, `TAG_ADDR`) muessen eindeutig sein.
- Wenn dein Schaltbild leicht andere Pins nutzt, nur den Block in `uwb_config.h` aendern.
- Fuer hohe Genauigkeit sollten Antenna-Delays spaeter kalibriert werden (siehe Repo-Doku).

## PlatformIO-Version (einfaches Trennen von Anchor/Tag)

Im Ordner `esp32c6_uwb` ist jetzt ein PlatformIO-Projekt mit zwei Umgebungen:

- `anchor` (Board als Anchor)
- `tag` (Board als Empfaenger/Tag)

Dateien:

- `esp32c6_uwb/platformio.ini`
- `esp32c6_uwb/include/uwb_config.h`
- `esp32c6_uwb/src/anchor_main.cpp`
- `esp32c6_uwb/src/tag_main.cpp`

### Bibliothek einbinden

Kopiere aus dem verlinkten Repo den Ordner `DW1000_library` nach:

`esp32c6_uwb/lib/DW1000_library`

### Bauen und Flashen

Im Terminal in den Ordner `esp32c6_uwb` wechseln und ausfuehren:

- Anchor build: `pio run -e anchor`
- Anchor flash: `pio run -e anchor -t upload`
- Tag build: `pio run -e tag`
- Tag flash: `pio run -e tag -t upload`
- Monitor (z.B. Tag): `pio device monitor -b 115200`

