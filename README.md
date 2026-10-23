# RoadSense GPS

**A real-time GPS map viewer with live speed and direction indicators.**

RoadSense GPS reads NMEA GPS data from a serial receiver and displays the
current position on live aerial imagery from Spain's National Geographic
Institute (IGN PNOA). It also supports simulated routes, speed-limit zones,
and an audible overspeed alarm.

## Features

- Reads GPS coordinates from `GPGGA` and `GNGGA` NMEA sentences.
- Displays the current position on IGN PNOA aerial imagery.
- Shows movement speed, heading, and compass direction.
- Loads speed-limit zones from `Mapa_INSIA2.txt` and indicates the current
  limit and speed status.
- Plays an audible alarm when the configured overspeed threshold is exceeded.
- Includes standalone and map-route demo modes for testing without a GPS
  receiver.

## Requirements

- Python 3.9 or later
- A GPS receiver that outputs NMEA data over a serial connection (for live
  tracking)
- An internet connection to download map imagery
- Tkinter, which is included with most Python installations

## Installation

Create and activate a virtual environment, then install the dependencies:

```bash
python -m venv .venv
```

On Windows:

```powershell
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

On macOS or Linux:

```bash
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Usage

Start live tracking (the default serial port is `COM3` at `9600` baud):

```bash
python gps_map_viewer.py
```

Specify a serial port and baud rate:

```bash
python gps_map_viewer.py --port COM4 --baud 9600
```

On Linux, a port might look like `/dev/ttyUSB0`:

```bash
python gps_map_viewer.py --port /dev/ttyUSB0 --baud 9600
```

Run the built-in simulated-position demo without a GPS receiver:

```bash
python gps_map_viewer.py --demo
```

Replay the route and speed-limit zones in `Mapa_INSIA2.txt`:

```bash
python gps_map_viewer.py --demo-map
```

Set the base speed for the route demo or provide a different zone file:

```bash
python gps_map_viewer.py --demo-map --demo-speed 20
python gps_map_viewer.py --demo-map --map path/to/zones.txt
```

## Speed-zone data

`Mapa_INSIA2.txt` contains whitespace-separated UTM northing, easting, and
speed-limit values, one point per line. The application uses these points to
determine the nearest speed limit to the current GPS position.
