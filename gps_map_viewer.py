"""
GPS Map Viewer — Lee tramas GGA de un puerto serie y muestra
el mapa de IGN centrado en la posición actual, con velocidad
y dirección de movimiento.

Dependencias:
    pip install pyserial pynmea2 pillow requests

Uso:
    python gps_map_viewer.py
    python gps_map_viewer.py --port COM3 --baud 9600
    python gps_map_viewer.py --demo          # modo demo sin GPS real
"""

import argparse
import io
import math
import os
import queue
import struct
import subprocess
import tempfile
import threading
import time
import wave
import platform
import tkinter as tk
from tkinter import ttk, font as tkfont, messagebox

import pynmea2
import requests
import serial
from PIL import Image, ImageDraw, ImageTk

# ──────────────────────────────────────────────────────────────
# LÍMITES DE VELOCIDAD — cargados desde Mapa_INSIA2.txt
# ──────────────────────────────────────────────────────────────

def _utm_to_latlon(easting: float, northing: float):
    """Convierte UTM ETRS89 zona 30N (EPSG:25830) a grados decimales (lat, lon)."""
    import math
    a      = 6_378_137.0
    f      = 1 / 298.257_222_101
    e2     = 2 * f - f ** 2
    e_p2   = e2 / (1 - e2)
    k0     = 0.9996
    lon0   = math.radians(-3.0)   # meridiano central zona 30
    x      = easting  - 500_000.0
    y      = northing
    M      = y / k0
    mu     = M / (a * (1 - e2/4 - 3*e2**2/64 - 5*e2**3/256))
    e1     = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
    phi1   = (mu
              + (3*e1/2 - 27*e1**3/32) * math.sin(2*mu)
              + (21*e1**2/16 - 55*e1**4/32) * math.sin(4*mu)
              + (151*e1**3/96) * math.sin(6*mu)
              + (1097*e1**4/512) * math.sin(8*mu))
    N1     = a / math.sqrt(1 - e2 * math.sin(phi1)**2)
    T1     = math.tan(phi1) ** 2
    C1     = e_p2 * math.cos(phi1) ** 2
    R1     = a * (1 - e2) / (1 - e2 * math.sin(phi1)**2) ** 1.5
    D      = x / (N1 * k0)
    lat    = phi1 - (N1 * math.tan(phi1) / R1) * (
                 D**2/2
                 - (5 + 3*T1 + 10*C1 - 4*C1**2 - 9*e_p2) * D**4/24
                 + (61 + 90*T1 + 298*C1 + 45*T1**2 - 252*e_p2 - 3*C1**2) * D**6/720)
    lon    = lon0 + (D
                     - (1 + 2*T1 + C1) * D**3/6
                     + (5 - 2*C1 + 28*T1 - 3*C1**2 + 8*e_p2 + 24*T1**2) * D**5/120
                     ) / math.cos(phi1)
    return math.degrees(lat), math.degrees(lon)


# Pares de velocidades solapadas y qué límite corresponde a cada dirección.
# Formato: (spd_sw, spd_ne)  — SW: rumbo 135-315°, NE: el resto
_OVERLAP_PAIRS = [
    (20, 18),   # 20 km/h hacia SW, 18 km/h hacia NE
    (10,  5),   # 10 km/h hacia SW,  5 km/h hacia NE
]


def _load_speed_zones(filepath: str):
    """
    Lee el fichero de mapa y devuelve una lista de dicts:
      { 'lat', 'lon', 'speed', 'overlap_pair' }
    'overlap_pair' es None si el punto no está en zona solapada, o la tupla
    (spd_sw, spd_ne) del par al que pertenece.
    """
    import math
    raw = []
    try:
        with open(filepath) as fh:
            for line in fh:
                parts = line.strip().split()
                if len(parts) == 3:
                    north, east, spd = float(parts[0]), float(parts[1]), int(parts[2])
                    lat, lon = _utm_to_latlon(east, north)
                    raw.append({'lat': lat, 'lon': lon, 'speed': spd,
                                'north': north, 'east': east,
                                'overlap_pair': None})
    except FileNotFoundError:
        return []

    # Detectar zonas solapadas para cada par definido en _OVERLAP_PAIRS
    for spd_sw, spd_ne in _OVERLAP_PAIRS:
        pts_a = [p for p in raw if p['speed'] == spd_sw]
        pts_b = [p for p in raw if p['speed'] == spd_ne]
        if not pts_a or not pts_b:
            continue
        ov_n_min = max(min(p['north'] for p in pts_a), min(p['north'] for p in pts_b))
        ov_n_max = min(max(p['north'] for p in pts_a), max(p['north'] for p in pts_b))
        ov_e_min = max(min(p['east']  for p in pts_a), min(p['east']  for p in pts_b))
        ov_e_max = min(max(p['east']  for p in pts_a), max(p['east']  for p in pts_b))
        if ov_n_max < ov_n_min or ov_e_max < ov_e_min:
            continue   # sin intersección real
        for p in raw:
            if (p['speed'] in (spd_sw, spd_ne)
                    and ov_n_min <= p['north'] <= ov_n_max
                    and ov_e_min <= p['east']  <= ov_e_max):
                p['overlap_pair'] = (spd_sw, spd_ne)

    # Eliminar campos auxiliares UTM
    for p in raw:
        del p['north'], p['east']
    return raw


def _find_speed_limit(lat: float, lon: float, heading_deg,
                       zones: list, search_radius_m: float = 25.0):
    """
    Devuelve el límite de velocidad del punto más cercano dentro de
    search_radius_m, o None si no hay ninguno.

    En zonas solapadas aplica la regla de sentido de circulación:
      - Hacia el SW (135°–315°) → velocidad del par con índice 0 (spd_sw)
      - Hacia el NE  (resto)    → velocidad del par con índice 1 (spd_ne)

    Pares solapados definidos en _OVERLAP_PAIRS:
      (20, 18): 20 km/h SW / 18 km/h NE
      (10,  5): 10 km/h SW /  5 km/h NE
    """
    import math
    best_d, best_zone = float('inf'), None
    for z in zones:
        dlat = (z['lat'] - lat) * 111_000
        dlon = (z['lon'] - lon) * 111_000 * math.cos(math.radians(lat))
        d = math.hypot(dlat, dlon)
        if d < best_d:
            best_d, best_zone = d, z
    if best_d > search_radius_m or best_zone is None:
        return None
    pair = best_zone.get('overlap_pair')
    if pair is not None and heading_deg is not None:
        spd_sw, spd_ne = pair
        h = heading_deg % 360
        return spd_sw if 135 <= h <= 315 else spd_ne
    return best_zone['speed']


def _speed_color(speed_kmh: float, limit: int) -> str:
    """Verde <90 %, Amarillo 90–110 %, Rojo >110 % del límite."""
    if limit <= 0:
        return "#e6edf3"
    ratio = speed_kmh / limit
    if ratio < 0.90:
        return "#3fb950"   # verde
    if ratio <= 1.10:
        return "#ffd700"   # amarillo
    return "#f85149"       # rojo



# ──────────────────────────────────────────────────────────────
# SONIDO DE ALARMA
# ──────────────────────────────────────────────────────────────

def _build_alarm_wav() -> str:
    """
    Genera un fichero WAV temporal con un sonido de alarma bifrecuencia
    usando únicamente la biblioteca estándar (wave + struct + math).
    Devuelve la ruta al fichero temporal.
    """
    sample_rate = 44100
    # Patrón: tono alto → tono bajo → pausa  (se repite en bucle)
    pattern: list[tuple[float, float]] = [
        (1047, 0.18),   # Do6  —  sílaba 1
        (1319, 0.18),   # Mi6  —  sílaba 2
        (1047, 0.18),   # Do6  —  sílaba 3
        (   0, 0.12),   # pausa
        (1319, 0.22),   # Mi6  —  sílaba 4 (más larga)
        (   0, 0.20),   # pausa larga entre ciclos
    ]



    frames: list[bytes] = []
    for freq, dur in pattern:
        n = int(sample_rate * dur)
        for i in range(n):
            if freq == 0:
                val = 0
            else:
                # Envolvente trapezoidal para evitar clics
                fade_samples = min(int(sample_rate * 0.01), n // 4)
                if i < fade_samples:
                    env = i / fade_samples
                elif i > n - fade_samples:
                    env = (n - i) / fade_samples
                else:
                    env = 1.0
                val = int(30000 * env * math.sin(2 * math.pi * freq * i / sample_rate))
            frames.append(struct.pack('<h', val))

    tmp = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
    tmp_path = tmp.name
    tmp.close()
    with wave.open(tmp_path, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames))
    return tmp_path


def _alarm_beep_loop(stop_event: threading.Event):
    """
    Reproduce el sonido de alarma en bucle hasta que stop_event se active.
    Genera el WAV una sola vez y lo reproduce con el comando nativo del SO.
    """
    wav_path = None
    try:
        wav_path = _build_alarm_wav()
        sys_name = platform.system()

        while not stop_event.is_set():
            try:
                if sys_name == 'Windows':
                    import winsound
                    winsound.PlaySound(wav_path, winsound.SND_FILENAME)
                elif sys_name == 'Darwin':
                    subprocess.run(['afplay', wav_path],
                                   timeout=5, capture_output=True)
                else:
                    # Linux: probar reproductores en orden de preferencia
                    _linux_play(wav_path, stop_event)
            except Exception:
                time.sleep(0.5)
    finally:
        if wav_path:
            try:
                os.unlink(wav_path)
            except OSError:
                pass


def _linux_play(wav_path: str, stop_event: threading.Event):
    """Intenta reproducir wav_path con el primer reproductor disponible."""
    players = ['paplay', 'aplay', 'pw-play', 'ffplay', 'mpv', 'cvlc']
    extra: dict[str, list[str]] = {
        'ffplay':  ['-nodisp', '-autoexit', '-loglevel', 'quiet'],
        'mpv':     ['--no-terminal', '--really-quiet'],
        'cvlc':    ['--play-and-exit', '--quiet'],
    }
    for player in players:
        if stop_event.is_set():
            return
        cmd = [player] + extra.get(player, []) + [wav_path]
        try:
            result = subprocess.run(cmd, timeout=6, capture_output=True)
            if result.returncode == 0:
                return          # reproducción correcta; volver para el siguiente ciclo
        except FileNotFoundError:
            continue            # reproductor no instalado
        except subprocess.TimeoutExpired:
            return
    # Último recurso: campana ASCII (puede sonar o no según el terminal/DE)
    print('\a', end='', flush=True)
    time.sleep(0.8)


# Ruta por defecto del fichero de mapa (mismo directorio que el script)
_DEFAULT_MAP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "Mapa_INSIA2.txt")

# ──────────────────────────────────────────────────────────────
# CONFIGURACIÓN POR DEFECTO
# ──────────────────────────────────────────────────────────────
DEFAULT_PORT      = "COM3"
DEFAULT_BAUD      = 9600
MAP_WIDTH         = 900
MAP_HEIGHT        = 680
MARGIN_DEG        = 0.004      # semiancho del recuadro en grados (~450 m)
REFRESH_MS        = 300
WMS_URL           = "https://www.ign.es/wms-inspire/pnoa-ma"

# ── Scroll suave: tile sobredimensionado ──────────────────────
# El tile descargado es FETCH_MULTIPLIER veces más grande que el
# área visible.  Mientras llega el siguiente tile, la imagen se
# desplaza sin tirones usando el tile en caché.
FETCH_MULTIPLIER  = 2.0    # factor de sobredimensionado (ancho y alto)
REPETCH_FRACTION  = 0.55   # re-descargar cuando la posición supere el
                            # 55 % del margen extra del tile en caché
PAN_INTERVAL_MS   = 80     # cada cuántos ms se re-renderiza el recorte (~12 fps)

CARDINALS = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
             "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"]

# ──────────────────────────────────────────────────────────────
# CÁLCULO DE VELOCIDAD Y DIRECCIÓN
# ──────────────────────────────────────────────────────────────

def haversine_km(lat1, lon1, lat2, lon2) -> float:
    """Distancia en km entre dos puntos usando la fórmula de Haversine."""
    R = 6371.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    """Rumbo inicial (0-360°, 0 = Norte, sentido horario)."""
    lat1, lat2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2)
    y = (math.cos(lat1) * math.sin(lat2)
         - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def bearing_to_cardinal(deg: float) -> str:
    return CARDINALS[round(deg / 22.5) % 16]


# ──────────────────────────────────────────────────────────────
# PARSEO GGA
# ──────────────────────────────────────────────────────────────

def parse_gga(line: str):
    """Devuelve (lat, lon) en grados decimales o None."""
    try:
        msg = pynmea2.parse(line)
        if not hasattr(msg, "latitude") or msg.latitude == 0.0:
            return None
        return float(msg.latitude), float(msg.longitude)
    except pynmea2.ParseError:
        return None


# ──────────────────────────────────────────────────────────────
# HILOS GPS
# ──────────────────────────────────────────────────────────────

def gps_thread(port, baud, pos_queue, stop_event, status_queue):
    ser = None
    try:
        ser = serial.Serial(port, baud, timeout=1)
        status_queue.put(f"✔ Puerto {port} abierto a {baud} bps")
        while not stop_event.is_set():
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                pos = parse_gga(line)
                if pos:
                    _update_queue(pos_queue, pos)
                    status_queue.put(f"GPS  {pos[0]:.6f}°N  {pos[1]:.6f}°E")
    except serial.SerialException as e:
        status_queue.put(f"⚠ Error serie: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()


def demo_thread(pos_queue, stop_event, status_queue):
    """Simula movimiento circular sobre Madrid (demo original)."""
    lat0, lon0 = 40.4168, -3.7038
    t = 0.0
    status_queue.put("● Modo DEMO activo — sin puerto serie real")
    while not stop_event.is_set():
        lat = lat0 + 0.002 * math.sin(t)
        lon = lon0 + 0.002 * math.cos(t)
        _update_queue(pos_queue, (lat, lon))
        status_queue.put(f"DEMO  {lat:.6f}°N  {lon:.6f}°E")
        t += 0.01
        time.sleep(0.05)


def map_demo_thread(pos_queue, stop_event, status_queue, map_file, speed_kmh=15.0):
    """
    Recorre los puntos de Mapa_INSIA2.txt interpolando posicion a la velocidad
    indicada (km/h).  Al llegar al ultimo punto invierte el sentido (ping-pong).

    La velocidad simulada varia en tres tramos del ciclo completo para ejercitar
    los tres estados de color del indicador:
      - tramo inicial  (~33 %) -> speed_kmh * 0.80  (verde,   <90 %)
      - tramo central  (~33 %) -> speed_kmh * 1.00  (amarillo, ~100 %)
      - tramo final    (~33 %) -> speed_kmh * 1.20  (rojo,    >110 %)
    """
    raw_zones = _load_speed_zones(map_file)
    if not raw_zones:
        status_queue.put("? Demo mapa: fichero no encontrado")
        return

    waypoints = [(z['lat'], z['lon']) for z in raw_zones]
    n = len(waypoints)
    status_queue.put(f"? Demo MAPA activo -- {n} puntos cargados")

    idx       = 0
    direction = 1
    seg_frac  = 0.0
    cycle_pos = 0.0

    UPDATE_HZ = 20
    dt_s      = 1.0 / UPDATE_HZ

    while not stop_event.is_set():
        i_next = max(0, min(n - 1, idx + direction))
        lat_a, lon_a = waypoints[idx]
        lat_b, lon_b = waypoints[i_next]

        dlat      = (lat_b - lat_a) * 111_000
        dlon      = (lon_b - lon_a) * 111_000 * math.cos(math.radians(lat_a))
        seg_len_m = math.hypot(dlat, dlon)

        t3 = cycle_pos % 1.0
        if t3 < 0.333:
            sim_speed = speed_kmh * 0.80
        elif t3 < 0.667:
            sim_speed = speed_kmh * 1.00
        else:
            sim_speed = speed_kmh * 1.20

        advance_m = (sim_speed / 3.6) * dt_s

        if seg_len_m > 0.1:
            seg_frac += advance_m / seg_len_m
        else:
            seg_frac = 1.0

        while seg_frac >= 1.0:
            seg_frac -= 1.0
            idx += direction
            if idx >= n - 1:
                idx       = n - 1
                direction = -1
            elif idx <= 0:
                idx       = 0
                direction = +1
            i_next        = max(0, min(n - 1, idx + direction))
            lat_a, lon_a  = waypoints[idx]
            lat_b, lon_b  = waypoints[i_next]
            dlat          = (lat_b - lat_a) * 111_000
            dlon          = (lon_b - lon_a) * 111_000 * math.cos(math.radians(lat_a))
            seg_len_m     = math.hypot(dlat, dlon)

        lat = lat_a + seg_frac * (lat_b - lat_a)
        lon = lon_a + seg_frac * (lon_b - lon_a)

        # Avance del ciclo: 1 ciclo = ida y vuelta completa
        total_m = seg_len_m * n if seg_len_m > 0 else 1
        cycle_pos += advance_m / (2 * total_m)

        _update_queue(pos_queue, (lat, lon))
        status_queue.put(
            f"DEMO MAPA  {lat:.6f}N  {lon:.6f}E  sim {sim_speed:.1f} km/h"
        )
        time.sleep(dt_s)


def _update_queue(q, value):
    if q.full():
        try:
            q.get_nowait()
        except queue.Empty:
            pass
    q.put_nowait(value)


# ──────────────────────────────────────────────────────────────
# PETICIÓN WMS A IGN
# ──────────────────────────────────────────────────────────────

def fetch_map(lat, lon, margin=MARGIN_DEG, width=MAP_WIDTH, height=MAP_HEIGHT,
              mult=FETCH_MULTIPLIER):
    """Descarga un tile *mult* veces más grande que el área visible."""
    fetch_w    = int(width  * mult)
    fetch_h    = int(height * mult)
    lat_margin = margin * mult
    lon_margin = margin * (width / height) * mult
    lat_min, lat_max = lat - lat_margin, lat + lat_margin
    lon_min, lon_max = lon - lon_margin, lon + lon_margin
    params = {
        "SERVICE": "WMS", "VERSION": "1.3.0", "REQUEST": "GetMap",
        "LAYERS": "OI.OrthoimageCoverage", "CRS": "EPSG:4326",
        "BBOX": f"{lat_min},{lon_min},{lat_max},{lon_max}",
        "WIDTH": str(fetch_w), "HEIGHT": str(fetch_h),
        "FORMAT": "image/png", "STYLES": "",
    }
    try:
        resp = requests.get(WMS_URL, params=params, timeout=15)
        resp.raise_for_status()
        return Image.open(io.BytesIO(resp.content)).convert("RGBA")
    except Exception as e:
        print(f"[WMS] Error: {e}")
        return None


# ──────────────────────────────────────────────────────────────
# DIBUJO DEL MARCADOR Y FLECHA DE DIRECCIÓN
# ──────────────────────────────────────────────────────────────

def draw_marker(img: Image.Image, heading_deg=None) -> Image.Image:
    out = img.copy()
    draw = ImageDraw.Draw(out)
    cx, cy = img.width // 2, img.height // 2
    r = 12

    # Flecha de dirección (amarilla)
    if heading_deg is not None:
        arrow_len = 36
        a = math.radians(heading_deg)
        ax = cx + arrow_len * math.sin(a)
        ay = cy - arrow_len * math.cos(a)
        draw.line([cx, cy, ax, ay], fill=(255, 210, 0, 230), width=3)
        wing = 10
        for side in (+1, -1):
            wx = ax - wing * math.sin(a + side * math.radians(140))
            wy = ay + wing * math.cos(a + side * math.radians(140))
            draw.line([ax, ay, wx, wy], fill=(255, 210, 0, 230), width=3)

    # Sombra
    draw.ellipse([cx-r+2, cy-r+2, cx+r+2, cy+r+2], fill=(0, 0, 0, 80))
    # Círculo blanco
    draw.ellipse([cx-r, cy-r, cx+r, cy+r], fill=(255, 255, 255, 230))
    # Punto rojo
    ri = 7
    draw.ellipse([cx-ri, cy-ri, cx+ri, cy+ri], fill=(220, 50, 50, 255))
    # Cruz de mira
    draw.line([cx-r-6, cy, cx-r-2, cy], fill=(220, 50, 50, 200), width=2)
    draw.line([cx+r+2, cy, cx+r+6, cy], fill=(220, 50, 50, 200), width=2)
    draw.line([cx, cy-r-6, cx, cy-r-2], fill=(220, 50, 50, 200), width=2)
    draw.line([cx, cy+r+2, cx, cy+r+6], fill=(220, 50, 50, 200), width=2)
    return out


# ──────────────────────────────────────────────────────────────
# WIDGET BRÚJULA
# ──────────────────────────────────────────────────────────────

class CompassWidget(tk.Canvas):
    SIZE = 110

    def __init__(self, parent, **kwargs):
        super().__init__(parent, width=self.SIZE, height=self.SIZE,
                         bg="#0d1117", highlightthickness=0, **kwargs)
        self._heading = 0.0
        self._draw(0.0)

    def set_heading(self, deg: float):
        if abs(deg - self._heading) > 0.5:
            self._heading = deg
            self._draw(deg)

    def _draw(self, deg: float):
        self.delete("all")
        cx = cy = self.SIZE // 2
        r = cx - 6

        self.create_oval(cx-r, cy-r, cx+r, cy+r,
                         fill="#161b22", outline="#30363d", width=2)

        for angle, label in [(0, "N"), (90, "E"), (180, "S"), (270, "O")]:
            a = math.radians(angle)
            xo = cx + (r - 5) * math.sin(a)
            yo = cy - (r - 5) * math.cos(a)
            color = "#dc3232" if label == "N" else "#8b949e"
            self.create_text(xo, yo, text=label,
                             fill=color, font=("Courier", 7, "bold"))

        needle_r = r - 18
        a = math.radians(deg)
        nx = cx + needle_r * math.sin(a)
        ny = cy - needle_r * math.cos(a)
        sx = cx - (needle_r * 0.6) * math.sin(a)
        sy = cy + (needle_r * 0.6) * math.cos(a)
        self.create_line(sx, sy, nx, ny, fill="#dc3232", width=3,
                         capstyle=tk.ROUND)
        self.create_oval(cx-4, cy-4, cx+4, cy+4,
                         fill="#e6edf3", outline="#0d1117")
        self.create_text(cx, cy + r - 14,
                         text=f"{deg:.0f}°", fill="#e6edf3",
                         font=("Courier", 8))


# ──────────────────────────────────────────────────────────────
# WIDGET SEÑAL DE VELOCIDAD
# ──────────────────────────────────────────────────────────────

class SpeedSignWidget(tk.Canvas):
    """
    Señal de límite de velocidad estilo tráfico español:
    círculo blanco con borde rojo y número negro en el centro.
    Cuando no hay límite activo muestra un guión gris.
    """
    SIZE = 110

    def __init__(self, parent, **kwargs):
        super().__init__(parent, width=self.SIZE, height=self.SIZE,
                         bg="#161b22", highlightthickness=0, **kwargs)
        self._limit = None
        self._draw(None)

    def set_limit(self, limit):
        if limit != self._limit:
            self._limit = limit
            self._draw(limit)

    def _draw(self, limit):
        self.delete("all")
        cx = cy = self.SIZE // 2
        r_outer = cx - 4
        r_inner = r_outer - 7   # grosor del aro rojo

        # Sombra suave
        self.create_oval(cx - r_outer + 3, cy - r_outer + 3,
                         cx + r_outer + 3, cy + r_outer + 3,
                         fill="#000000", outline="")

        if limit is not None:
            # Aro exterior rojo
            self.create_oval(cx - r_outer, cy - r_outer,
                             cx + r_outer, cy + r_outer,
                             fill="#cc2200", outline="#881100", width=1)
            # Relleno blanco interior
            self.create_oval(cx - r_inner, cy - r_inner,
                             cx + r_inner, cy + r_inner,
                             fill="#f5f5f5", outline="#dddddd", width=1)
            # Número
            digits = len(str(limit))
            font_size = 28 if digits <= 2 else 22
            self.create_text(cx, cy,
                             text=str(limit),
                             fill="#111111",
                             font=("Arial", font_size, "bold"))
            # Etiqueta inferior
            self.create_text(cx, cy + r_outer + 10,
                             text="km/h", fill="#8b949e",
                             font=("Courier", 7))
        else:
            # Sin zona: círculo gris apagado con guión
            self.create_oval(cx - r_outer, cy - r_outer,
                             cx + r_outer, cy + r_outer,
                             fill="#1e2730", outline="#30363d", width=2)
            self.create_text(cx, cy, text="—",
                             fill="#444d56", font=("Arial", 26, "bold"))
            self.create_text(cx, cy + r_outer + 10,
                             text="sin zona", fill="#444d56",
                             font=("Courier", 7))



# ──────────────────────────────────────────────────────────────
# DIÁLOGO DE CONFIGURACIÓN DE ALARMA
# ──────────────────────────────────────────────────────────────

class AlarmSetupDialog:
    """
    Modal con mapa interactivo para configurar la alarma de zona:
      · Arrastra para mover el mapa
      · Rueda del ratón para zoom
      · Clic para colocar / mover el centro de la alarma
    Tras confirmar, self.result = (lat, lon, radius_m) o None si cancela.
    """
    DISP_W      = 660
    DISP_H      = int(660 * MAP_HEIGHT / MAP_WIDTH)
    MARGIN_MIN  = 0.0003   # ~33 m semiancho → máximo zoom
    MARGIN_MAX  = 0.06     # ~6.6 km semiancho → mínimo zoom
    ZOOM_FACTOR = 1.5      # factor por paso de rueda

    def __init__(self, parent, pil_image: 'Image.Image', render_pos):
        self.result     = None
        self._selected  = None   # (lat, lon) del pin de alarma

        # ── Vista actual del mapa en el diálogo ────────────────
        self._view_lat    = render_pos[0]
        self._view_lon    = render_pos[1]
        self._view_margin = MARGIN_DEG   # mismo zoom inicial que el mapa principal

        # ── Tile en caché (arranca con la captura actual) ──────
        self._tile_pil    = pil_image.convert("RGB").resize(
            (self.DISP_W, self.DISP_H), Image.LANCZOS)
        self._tile_lat    = render_pos[0]
        self._tile_lon    = render_pos[1]
        self._tile_margin = MARGIN_DEG
        self._fetch_gen   = 0
        self._loading     = False

        # ── Estado de arrastre ─────────────────────────────────
        self._press_x   = 0
        self._press_y   = 0
        self._press_lat = render_pos[0]
        self._press_lon = render_pos[1]
        self._is_drag   = False

        # ── Ventana ────────────────────────────────────────────
        self.top = tk.Toplevel(parent)
        self.top.title("Configurar alarma de zona")
        self.top.configure(bg="#0d1117")
        self.top.resizable(False, False)
        self.top.transient(parent)
        self.top.grab_set()
        self.top.protocol("WM_DELETE_WINDOW", self.top.destroy)

        # ── Instrucciones ──────────────────────────────────────
        tk.Label(self.top,
                 text="Arrastra para mover  ·  Rueda del ratón para zoom  ·  Clic para colocar la alarma",
                 font=("Courier", 9), bg="#0d1117", fg="#8b949e").pack(pady=(10, 2))

        # ── Canvas ─────────────────────────────────────────────
        self._canvas = tk.Canvas(self.top,
                                 width=self.DISP_W, height=self.DISP_H,
                                 bg="#161b22", highlightthickness=1,
                                 highlightbackground="#30363d",
                                 cursor="crosshair")
        self._canvas.pack(padx=12, pady=(0, 8))
        self._canvas.bind("<ButtonPress-1>",  self._on_press)
        self._canvas.bind("<B1-Motion>",       self._on_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_release)
        self._canvas.bind("<MouseWheel>",      self._on_wheel)   # Windows/macOS
        self._canvas.bind("<Button-4>",        self._on_wheel)   # Linux ↑
        self._canvas.bind("<Button-5>",        self._on_wheel)   # Linux ↓
        self._tk_img = None

        # ── Control de radio ───────────────────────────────────
        rf = tk.Frame(self.top, bg="#0d1117")
        rf.pack(fill="x", padx=16, pady=(0, 8))
        tk.Label(rf, text="Radio de alarma:", font=("Courier", 10),
                 bg="#0d1117", fg="#e6edf3").pack(side="left")
        self._radius_var = tk.IntVar(value=200)
        tk.Scale(rf, from_=50, to=5000, orient="horizontal",
                 variable=self._radius_var, resolution=50, length=380,
                 bg="#0d1117", fg="#e6edf3", highlightthickness=0,
                 troughcolor="#21262d", activebackground="#dc3232",
                 command=lambda _: self._redraw()
                 ).pack(side="left", padx=(10, 0))
        self._lbl_r = tk.Label(rf, text="200 m", width=8,
                               font=("Courier", 10, "bold"),
                               bg="#0d1117", fg="#ffd700")
        self._lbl_r.pack(side="left", padx=(8, 0))

        # ── Botones ────────────────────────────────────────────
        bf = tk.Frame(self.top, bg="#0d1117")
        bf.pack(pady=(0, 14))
        self._btn_ok = tk.Button(bf, text="\U0001f514  Establecer alarma",
                                  font=("Courier", 11, "bold"),
                                  bg="#1f6feb", fg="white",
                                  activebackground="#388bfd",
                                  relief="flat", padx=18, pady=7,
                                  state="disabled", command=self._confirm)
        self._btn_ok.pack(side="left", padx=(0, 14))
        tk.Button(bf, text="\u2715  Cancelar",
                  font=("Courier", 11), bg="#21262d", fg="#e6edf3",
                  activebackground="#30363d", relief="flat",
                  padx=18, pady=7,
                  command=self.top.destroy).pack(side="left")

        # Render inicial (todos los widgets ya existen)
        self._wheel_after = None
        self._redraw(Image.LANCZOS)

    # ── PPD del diálogo ────────────────────────────────────────

    @property
    def _ppd(self) -> float:
        """Píxeles por grado en la vista actual del diálogo."""
        return self.DISP_H / (2.0 * self._view_margin)

    # ── Conversiones ──────────────────────────────────────────

    def _px_to_latlon(self, px: float, py: float):
        """Convierte píxel del canvas → (lat, lon)."""
        dlat = -(py - self.DISP_H / 2.0) / self._ppd
        dlon =  (px - self.DISP_W / 2.0) / self._ppd
        return self._view_lat + dlat, self._view_lon + dlon

    def _latlon_to_px(self, lat: float, lon: float):
        """Convierte (lat, lon) → píxel del canvas."""
        ppd = self._ppd
        px  = self.DISP_W / 2.0 + (lon - self._view_lon) * ppd
        py  = self.DISP_H / 2.0 - (lat - self._view_lat) * ppd
        return px, py

    # ── Eventos del canvas ─────────────────────────────────────

    def _on_press(self, event):
        self._press_x   = event.x
        self._press_y   = event.y
        self._press_lat = self._view_lat
        self._press_lon = self._view_lon
        self._is_drag   = False
        self._canvas.configure(cursor="fleur")

    def _on_drag(self, event):
        dx = event.x - self._press_x
        dy = event.y - self._press_y
        if abs(dx) > 4 or abs(dy) > 4:
            self._is_drag = True
        if not self._is_drag:
            return
        ppd = self._ppd
        self._view_lat = self._press_lat + dy / ppd
        self._view_lon = self._press_lon - dx / ppd
        self._redraw(Image.NEAREST)   # rápido durante el arrastre

    def _on_release(self, event):
        self._canvas.configure(cursor="crosshair")
        if not self._is_drag:
            lat, lon = self._px_to_latlon(event.x, event.y)
            self._selected = (lat, lon)
            self._btn_ok.configure(state="normal")
            self._redraw(Image.LANCZOS)
        else:
            self._redraw(Image.LANCZOS)   # calidad al soltar
            self._fetch_tile()

    def _on_wheel(self, event):
        """Zoom con la rueda del ratón. Descarga tile solo 300 ms después del último paso."""
        zoom_in = (event.num == 4) or (getattr(event, 'delta', 0) > 0)
        if zoom_in:
            self._view_margin = max(self.MARGIN_MIN,
                                    self._view_margin / self.ZOOM_FACTOR)
        else:
            self._view_margin = min(self.MARGIN_MAX,
                                    self._view_margin * self.ZOOM_FACTOR)
        self._redraw(Image.NEAREST)           # preview inmediato
        # Cancelar descarga pendiente y reprogramar con debounce
        if hasattr(self, '_wheel_after') and self._wheel_after is not None:
            try:
                self.top.after_cancel(self._wheel_after)
            except Exception:
                pass
        self._wheel_after = self.top.after(300, self._fetch_tile)

    # ── Renderizado ────────────────────────────────────────────

    def _redraw(self, quality=Image.NEAREST):
        """
        Renderiza la vista actual reutilizando el tile en caché.
        Durante el arrastre usa NEAREST (rápido); al soltar se llama
        con LANCZOS para calidad final.
        """
        ppd_tile = self.DISP_H / (2.0 * self._tile_margin)
        ppd_view = self._ppd
        scale    = ppd_view / ppd_tile

        new_w = max(1, int(self.DISP_W * scale))
        new_h = max(1, int(self.DISP_H * scale))
        scaled = self._tile_pil.resize((new_w, new_h), quality)

        ppd = self._ppd
        tile_cx = self.DISP_W / 2.0 + (self._tile_lon - self._view_lon) * ppd
        tile_cy = self.DISP_H / 2.0 - (self._tile_lat - self._view_lat) * ppd
        tl_x = int(round(tile_cx - new_w / 2.0))
        tl_y = int(round(tile_cy - new_h / 2.0))

        base = Image.new("RGB", (self.DISP_W, self.DISP_H), (22, 27, 34))
        base.paste(scaled, (tl_x, tl_y))
        self._draw_overlay(base)

    def _draw_overlay(self, base: 'Image.Image'):
        """Dibuja pin + círculo sobre la imagen base y la pone en el canvas.
        Toda la composición PIL ocurre aquí; ImageTk se crea al final."""
        r_m = self._radius_var.get()
        self._lbl_r.configure(text=f"{r_m} m")

        img = base.convert("RGBA")

        if self._selected is not None:
            cx, cy = self._latlon_to_px(*self._selected)
            r_px   = (r_m / 111_000.0) * self._ppd

            # Relleno semitransparente
            ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
            od = ImageDraw.Draw(ov)
            od.ellipse([cx - r_px, cy - r_px, cx + r_px, cy + r_px],
                       fill=(255, 120, 0, 55))
            img = Image.alpha_composite(img, ov)

            # Borde + retícula central
            draw = ImageDraw.Draw(img)
            draw.ellipse([cx - r_px, cy - r_px, cx + r_px, cy + r_px],
                         outline=(255, 120, 0, 255), width=3)
            draw.ellipse([cx - 7, cy - 7, cx + 7, cy + 7],
                         fill=(255, 120, 0, 255), outline=(255, 255, 255, 220), width=2)
            draw.line([cx - 20, cy, cx + 20, cy], fill=(255, 255, 255, 220), width=2)
            draw.line([cx, cy - 20, cx, cy + 20], fill=(255, 255, 255, 220), width=2)

        self._tk_img = ImageTk.PhotoImage(img.convert("RGB"))
        self._canvas.create_image(0, 0, anchor="nw", image=self._tk_img)

        # Indicador de carga
        if self._loading:
            self._canvas.create_rectangle(
                self.DISP_W // 2 - 120, self.DISP_H // 2 - 18,
                self.DISP_W // 2 + 120, self.DISP_H // 2 + 18,
                fill="#0d1117", outline="#30363d")
            self._canvas.create_text(
                self.DISP_W // 2, self.DISP_H // 2,
                text="\u27f3  Cargando mapa\u2026", fill="#8b949e",
                font=("Courier", 12))

        # Coordenadas en la esquina inferior izquierda
        if self._selected is not None:
            slat, slon = self._selected
            coord_txt = f"Alarma: {slat:.5f}\u00b0  {slon:.5f}\u00b0"
            self._canvas.create_text(8, self.DISP_H - 8, anchor="sw",
                                     text=coord_txt, fill="#ffd700",
                                     font=("Courier", 8))

    # ── Descarga de tile ───────────────────────────────────────

    def _fetch_tile(self):
        """Solicita un nuevo tile centrado en _view_lat/_view_lon/_view_margin."""
        self._fetch_gen += 1
        gen    = self._fetch_gen
        lat    = self._view_lat
        lon    = self._view_lon
        margin = self._view_margin
        self._loading = True
        self._redraw()

        def _worker():
            img = fetch_map(lat, lon, margin=margin,
                            width=self.DISP_W, height=self.DISP_H, mult=1.0)
            try:
                self.top.after(0, lambda: self._apply_tile(img, gen, lat, lon, margin))
            except tk.TclError:
                pass  # diálogo cerrado antes de recibir la respuesta

        threading.Thread(target=_worker, daemon=True).start()

    def _apply_tile(self, img, gen: int,
                    lat: float, lon: float, margin: float):
        if gen != self._fetch_gen:
            return
        self._loading = False
        if img is not None:
            self._tile_pil    = img.convert("RGB")
            self._tile_lat    = lat
            self._tile_lon    = lon
            self._tile_margin = margin
        self._redraw(Image.LANCZOS)

    # ── Confirmar ──────────────────────────────────────────────

    def _confirm(self):
        if self._selected is None:
            return
        lat, lon = self._selected
        self.result = (lat, lon, self._radius_var.get())
        self.top.destroy()


# ──────────────────────────────────────────────────────────────
# CONSTRUCCIÓN DE FRAME (ejecutable en hilo secundario)
# ──────────────────────────────────────────────────────────────

def _build_frame(tile_image, tile_center, render_pos, alarm, heading_deg):
    """
    Recorta el tile, dibuja la alarma y el marcador GPS.
    Devuelve (pil_crop_sin_marcador, ImageTk.PhotoImage) o None si falla.
    Diseñada para ejecutarse fuera del hilo de Tkinter.
    """
    try:
        lat, lon       = render_pos
        tlat, tlon     = tile_center
        tile_w, tile_h = tile_image.size

        ppd_lat = tile_h / (2.0 * MARGIN_DEG * FETCH_MULTIPLIER)
        ppd_lon = tile_w / (2.0 * MARGIN_DEG * (MAP_WIDTH / MAP_HEIGHT) * FETCH_MULTIPLIER)

        cx = tile_w / 2.0 + (lon - tlon) * ppd_lon
        cy = tile_h / 2.0 - (lat - tlat) * ppd_lat

        half_w, half_h = MAP_WIDTH / 2, MAP_HEIGHT / 2
        left = int(max(0, min(cx - half_w, tile_w - MAP_WIDTH)))
        top  = int(max(0, min(cy - half_h, tile_h - MAP_HEIGHT)))
        crop = tile_image.crop((left, top, left + MAP_WIDTH, top + MAP_HEIGHT))

        # Círculo de alarma
        if alarm is not None:
            alat, alon = alarm['lat'], alarm['lon']
            r_m        = alarm['radius_m']
            ax_view    = tile_w / 2.0 + (alon - tlon) * ppd_lon - left
            ay_view    = tile_h / 2.0 - (alat - tlat) * ppd_lat - top
            r_px       = (r_m / 111_000.0) * ppd_lat
            overlay    = Image.new('RGBA', crop.size, (0, 0, 0, 0))
            od         = ImageDraw.Draw(overlay)
            od.ellipse([ax_view - r_px, ay_view - r_px,
                        ax_view + r_px, ay_view + r_px],
                       fill=(255, 120, 0, 45))
            crop = Image.alpha_composite(crop.convert('RGBA'), overlay)
            ad   = ImageDraw.Draw(crop)
            ad.ellipse([ax_view - r_px, ay_view - r_px,
                        ax_view + r_px, ay_view + r_px],
                       outline=(255, 120, 0, 230), width=3)
            ad.ellipse([ax_view - 6, ay_view - 6,
                        ax_view + 6, ay_view + 6],
                       fill=(255, 120, 0, 255), outline=(255, 255, 255, 200), width=2)

        pil_crop = crop.copy()                          # para el diálogo de alarma
        crop_marked = draw_marker(crop, heading_deg=heading_deg)
        tk_photo = ImageTk.PhotoImage(crop_marked.convert("RGB"))
        return pil_crop, tk_photo
    except Exception as e:
        print(f"[render] Error: {e}")
        return None


# ──────────────────────────────────────────────────────────────
# APLICACIÓN PRINCIPAL
# ──────────────────────────────────────────────────────────────

class GPSMapApp:
    def __init__(self, root: tk.Tk, args):
        self.root = root
        self.args = args
        mode_tag = " [DEMO MAPA]" if getattr(args, "demo_map", False) else (" [DEMO]" if args.demo else "")
        self.root.title(f"GPS Map Viewer · IGN PNOA{mode_tag}")
        self.root.configure(bg="#0d1117")
        self.root.resizable(False, False)

        self.pos_queue    = queue.Queue(maxsize=1)
        self.status_queue = queue.Queue(maxsize=5)
        self.stop_event   = threading.Event()

        self._current_pos      = None   # último punto GPS recibido (destino)
        self._render_pos       = None   # posición interpolada para renderizar
        self._interp_start     = None   # origen del segmento de interpolación
        self._interp_t         = 1.0   # fracción [0,1]; arranca en 1 (sin segmento)
        self._interp_start_time= 0.0
        self._interp_duration  = REFRESH_MS / 1000.0
        self._last_gps_time    = None
        self._prev_pos     = None
        self._prev_time    = None
        self._speed_kmh    = 0.0
        self._heading_deg  = None
        self._tk_image     = None
        self._fetching     = False
        self._current_limit = None

        # Cargar zonas de velocidad
        self._speed_zones = _load_speed_zones(getattr(args, 'map', _DEFAULT_MAP_FILE))

        # Scroll suave: tile en caché sobredimensionado
        self._tile_image      = None   # PIL Image grande (FETCH_MULTIPLIER × visible)
        self._tile_center     = None   # (lat, lon) del centro del tile en caché
        self._last_render_pil = None   # última vista PIL para el diálogo de alarma
        self._rendering       = False  # True mientras un hilo de render está activo

        # Alarma de zona
        self._alarm           = None   # dict {lat, lon, radius_m} o None
        self._alarm_firing    = False  # True mientras el popup está activo
        self._alarm_sound_stop= None   # Event para detener el hilo de sonido

        self._build_ui()
        self._start_gps()
        self._schedule_refresh()
        self.root.after(PAN_INTERVAL_MS, self._pan_tick)  # inicia el bucle de paneo

    # ── UI ───────────────────────────────────────────────────

    def _build_ui(self):
        PAD = 12
        BG  = "#0d1117"
        ACC = "#dc3232"

        header = tk.Frame(self.root, bg=BG)
        header.pack(fill="x", padx=PAD, pady=(PAD, 0))
        tk.Label(header, text="◉ GPS MAP VIEWER",
                 font=tkfont.Font(family="Courier", size=15, weight="bold"),
                 bg=BG, fg=ACC).pack(side="left")
        tk.Button(header, text="🔔  Alarma",
                  font=("Courier", 10), bg="#21262d", fg="#e6edf3",
                  activebackground="#30363d", relief="flat", cursor="hand2",
                  command=self._open_alarm_dialog).pack(side="left", padx=(14, 0))
        self.lbl_coords = tk.Label(header, text="Esperando señal GPS…",
                                   font=("Courier", 10), bg=BG, fg="#8b949e")
        self.lbl_coords.pack(side="right")

        self.canvas = tk.Canvas(self.root, width=MAP_WIDTH, height=MAP_HEIGHT,
                                bg="#161b22", highlightthickness=1,
                                highlightbackground="#30363d")
        self.canvas.pack(padx=PAD, pady=PAD)
        self._show_placeholder()

        # Panel inferior
        bottom = tk.Frame(self.root, bg="#161b22",
                          highlightthickness=1, highlightbackground="#30363d")
        bottom.pack(fill="x", padx=PAD, pady=(0, PAD))

        self.lbl_status = tk.Label(bottom, text="Iniciando…",
                                   font=("Courier", 9), bg="#161b22",
                                   fg="#8b949e", anchor="w")
        self.lbl_status.pack(side="left", padx=8, pady=6)

        # Brújula (extremo derecho)
        self.compass = CompassWidget(bottom)
        self.compass.pack(side="right", padx=8, pady=4)

        # Señal de límite de velocidad (Canvas tkinter, estilo señal ES)
        self.speed_sign = SpeedSignWidget(bottom)
        self.speed_sign.pack(side="right", padx=8, pady=4)

        # Velocidad + rumbo
        sf = tk.Frame(bottom, bg="#161b22")
        sf.pack(side="right", padx=20, pady=6)
        tk.Label(sf, text="VELOCIDAD", font=("Courier", 7),
                 bg="#161b22", fg="#8b949e").pack()
        self.lbl_speed = tk.Label(sf, text="— km/h",
                                  font=("Courier", 20, "bold"),
                                  bg="#161b22", fg="#e6edf3")
        self.lbl_speed.pack()
        self.lbl_heading = tk.Label(sf, text="Rumbo: —",
                                    font=("Courier", 10),
                                    bg="#161b22", fg="#8b949e")
        self.lbl_heading.pack(pady=(2, 0))

        self.lbl_refresh = tk.Label(bottom, text="",
                                    font=("Courier", 9), bg="#161b22",
                                    fg="#3fb950", anchor="e")
        self.lbl_refresh.pack(side="right", padx=8)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Red.Horizontal.TProgressbar",
                         troughcolor="#161b22", background=ACC,
                         darkcolor=ACC, lightcolor=ACC, bordercolor="#30363d")
        self.progress = ttk.Progressbar(self.root, mode="indeterminate",
                                        style="Red.Horizontal.TProgressbar",
                                        length=MAP_WIDTH)
        self.progress.pack(padx=PAD, pady=(0, PAD))

    def _show_placeholder(self):
        self.canvas.delete("all")
        self.canvas.create_rectangle(0, 0, MAP_WIDTH, MAP_HEIGHT, fill="#161b22")
        self.canvas.create_text(MAP_WIDTH // 2, MAP_HEIGHT // 2,
                                text="Esperando posición GPS…",
                                fill="#30363d", font=("Courier", 18))

    # ── GPS ──────────────────────────────────────────────────

    def _start_gps(self):
        if getattr(self.args, 'demo_map', False):
            target = map_demo_thread
            targs  = (self.pos_queue, self.stop_event, self.status_queue,
                      self.args.map, self.args.demo_speed)
        elif self.args.demo:
            target = demo_thread
            targs  = (self.pos_queue, self.stop_event, self.status_queue)
        else:
            target = gps_thread
            targs  = (self.args.port, self.args.baud, self.pos_queue,
                      self.stop_event, self.status_queue)
        threading.Thread(target=target, args=targs, daemon=True).start()

    # ── Velocidad / rumbo ────────────────────────────────────

    def _compute_motion(self, new_pos):
        now = time.monotonic()
        if self._prev_pos is not None and self._prev_time is not None:
            dt_h = (now - self._prev_time) / 3600.0
            if dt_h > 0:
                lat1, lon1 = self._prev_pos
                lat2, lon2 = new_pos
                dist_km = haversine_km(lat1, lon1, lat2, lon2)
                self._speed_kmh = dist_km / dt_h
                if dist_km > 0.002:          # > ~2 m de desplazamiento
                    self._heading_deg = bearing_deg(lat1, lon1, lat2, lon2)
        self._prev_pos  = new_pos
        self._prev_time = now

    # ── Bucle de refresco ────────────────────────────────────

    def _schedule_refresh(self):
        self._poll_status()
        self._try_update_map()
        self.root.after(REFRESH_MS, self._schedule_refresh)

    def _pan_tick(self):
        """Avanza la posicion de render y re-renderiza el recorte del tile."""
        if self._tile_image is not None and self._current_pos is not None:
            self._advance_render_pos()
            self._render_current_view()
            self._check_alarm()
        self.root.after(PAN_INTERVAL_MS, self._pan_tick)

    def _advance_render_pos(self):
        """
        Interpola _render_pos desde _interp_start hacia _current_pos usando
        tiempo real transcurrido.  t = elapsed / duracion_segmento, donde
        duracion_segmento es el intervalo real entre los dos ultimos puntos GPS.
        Esto garantiza que:
          - El marcador llega al destino exactamente cuando llega el siguiente GPS.
          - Funciona igual a cualquier frecuencia de GPS (demo rapida o lenta).
          - No depende de la velocidad calculada (evita el problema de doble
            interpolacion en la demo de mapa que ya emite a 20 Hz).
        """
        if (not hasattr(self, '_interp_start')
                or self._interp_start is None
                or self._current_pos is None):
            return

        if self._interp_t >= 1.0:
            # Ya hemos llegado al destino; no mover hasta el siguiente GPS
            return

        elapsed  = time.monotonic() - self._interp_start_time
        duration = max(getattr(self, '_interp_duration', REFRESH_MS / 1000.0), 0.01)
        t        = min(1.0, elapsed / duration)

        lat_s, lon_s = self._interp_start
        lat_d, lon_d = self._current_pos

        self._interp_t   = t
        self._render_pos = (
            lat_s + t * (lat_d - lat_s),
            lon_s + t * (lon_d - lon_s),
        )

    def _poll_status(self):
        msgs = []
        while not self.status_queue.empty():
            try:
                msgs.append(self.status_queue.get_nowait())
            except queue.Empty:
                break
        if msgs:
            self.lbl_status.configure(text=msgs[-1])

    def _try_update_map(self):
        try:
            pos = self.pos_queue.get_nowait()
        except queue.Empty:
            return

        now = time.monotonic()
        self._compute_motion(pos)

        # ── Interpolación suave ──────────────────────────────
        # Al llegar un nuevo punto GPS arrancamos un segmento de interpolación
        # desde la posición de render actual hasta el nuevo punto GPS.
        # La duración del segmento es el intervalo real entre los dos últimos
        # puntos GPS, medido con time.monotonic().
        prev_gps = self._current_pos
        self._current_pos = pos

        if prev_gps is None or self._render_pos is None:
            # Primera posición: sin animación, saltar directamente
            self._render_pos        = pos
            self._interp_start      = pos
            self._interp_t          = 1.0
            self._interp_start_time = now
            self._interp_duration   = REFRESH_MS / 1000.0
        else:
            # Intervalo real entre este GPS y el anterior
            dt = (now - self._last_gps_time) if self._last_gps_time is not None else REFRESH_MS / 1000.0
            # Si el GPS llega más rápido que el pan_tick (ej. demo 20 Hz)
            # la interpolación no aporta suavidad; saltar directamente.
            if dt <= PAN_INTERVAL_MS / 1000.0:
                self._render_pos        = pos
                self._interp_start      = pos
                self._interp_t          = 1.0
                self._interp_start_time = now
                self._interp_duration   = dt
            else:
                self._interp_start      = self._render_pos
                self._interp_t          = 0.0
                self._interp_start_time = now
                self._interp_duration   = max(dt, 0.01)

        self._last_gps_time = now
        lat, lon = pos

        self.lbl_coords.configure(text=f"Lat {lat:.6f}°  Lon {lon:.6f}°")

        # Límite de velocidad
        limit = _find_speed_limit(lat, lon, self._heading_deg, self._speed_zones)
        self._current_limit = limit
        spd_color = _speed_color(self._speed_kmh, limit) if limit is not None else "#e6edf3"
        self.lbl_speed.configure(text=f"{self._speed_kmh:.1f} km/h", fg=spd_color)
        self.speed_sign.set_limit(limit)

        if self._heading_deg is not None:
            cardinal = bearing_to_cardinal(self._heading_deg)
            self.lbl_heading.configure(
                text=f"Rumbo: {self._heading_deg:.0f}° {cardinal}")
            self.compass.set_heading(self._heading_deg)
        else:
            self.lbl_heading.configure(text="Rumbo: calculando…")

        # Solo descargamos si no hay tile en cache o la posicion se acerca al borde
        if self._fetching:
            return
        if not self._needs_refetch(lat, lon):
            # El tile actual cubre la posicion; el pan_tick se encarga del render
            return
        self._fetching = True
        self.progress.start(12)
        self.lbl_refresh.configure(text="Descargando mapa…")
        threading.Thread(target=self._fetch_and_display,
                         args=(lat, lon), daemon=True).start()

    def _fetch_and_display(self, lat, lon):
        img = fetch_map(lat, lon)
        self.root.after(0, self._apply_image, img, lat, lon)

    def _needs_refetch(self, lat, lon) -> bool:
        if self._tile_center is None or self._tile_image is None:
            return True
        tile_w, tile_h = self._tile_image.size
        tlat, tlon = self._tile_center
        ppd_lat = tile_h / (2.0 * MARGIN_DEG * FETCH_MULTIPLIER)
        ppd_lon = tile_w / (2.0 * MARGIN_DEG * (MAP_WIDTH / MAP_HEIGHT) * FETCH_MULTIPLIER)
        dx = abs(lon - tlon) * ppd_lon
        dy = abs(lat - tlat) * ppd_lat
        thresh_x = (tile_w - MAP_WIDTH)  / 2.0 * REPETCH_FRACTION
        thresh_y = (tile_h - MAP_HEIGHT) / 2.0 * REPETCH_FRACTION
        return dx > thresh_x or dy > thresh_y

    def _apply_image(self, img, lat, lon):
        self.progress.stop()
        self._fetching = False
        if img is None:
            self.lbl_refresh.configure(text="⚠ Error descargando mapa")
            return
        # Guardar tile sobredimensionado en cache
        self._tile_image  = img
        self._tile_center = (lat, lon)
        self._render_current_view()
        self.lbl_refresh.configure(
            text=f"✔ Actualizado {time.strftime('%H:%M:%S')}")

    def _render_current_view(self):
        """
        Lanza el trabajo pesado de PIL en un hilo secundario para no bloquear
        el event-loop de Tkinter. Si ya hay un render en curso lo descarta
        (solo importa el frame más reciente).
        """
        if self._tile_image is None or self._render_pos is None:
            return
        if getattr(self, '_rendering', False):
            return                        # frame anterior aún en curso → saltamos
        self._rendering = True

        # Capturar estado inmutable para el hilo
        tile_snap   = self._tile_image    # PIL Image (inmutable tras _apply_image)
        tile_center = self._tile_center
        render_pos  = self._render_pos
        alarm       = self._alarm
        heading     = self._heading_deg

        def _worker():
            frame = _build_frame(tile_snap, tile_center, render_pos,
                                 alarm, heading)
            self.root.after(0, self._apply_frame, frame, render_pos)

        threading.Thread(target=_worker, daemon=True).start()

    def _apply_frame(self, frame_data, render_pos):
        """Recibe (pil_crop_sin_marcador, tk_photo) desde el hilo y actualiza el canvas."""
        self._rendering = False
        if frame_data is None:
            return
        pil_crop, tk_photo = frame_data
        self._last_render_pil = pil_crop
        self._tk_image        = tk_photo
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor="nw", image=self._tk_image)
        self._draw_scale()
        self._draw_speed_overlay()

    def _draw_scale(self):
        meters_total = (MARGIN_DEG * 2) * (MAP_WIDTH / MAP_HEIGHT) * 111_000
        bar_px = int(200 * MAP_WIDTH / meters_total)
        x0, y0 = 16, MAP_HEIGHT - 24
        self.canvas.create_rectangle(x0, y0, x0+bar_px, y0+6,
                                     fill="white", outline="white")
        self.canvas.create_text(x0, y0-4, text="200 m", anchor="w",
                                fill="white", font=("Courier", 9, "bold"))

    def _draw_speed_overlay(self):
        """
        Overlay en la esquina superior derecha del mapa.
        Usa canvas items con tags fijos y solo los recrea cuando cambian los datos,
        evitando recrear todos los items en cada frame.
        """
        limit     = self._current_limit
        spd_color = _speed_color(self._speed_kmh, limit) if limit is not None else "#e6edf3"

        # ── Caja de fondo ──────────────────────────────────────
        box_w, box_h = 118, 118
        bx = MAP_WIDTH - box_w - 10
        by = 10
        self.canvas.create_rectangle(bx, by, bx + box_w, by + box_h,
                                     fill="#0d1117", outline="#30363d",
                                     stipple="gray50")

        # ── Señal circular ─────────────────────────────────────
        sign_r = 36
        cx     = bx + box_w // 2
        cy     = by + 10 + sign_r

        self.canvas.create_oval(cx - sign_r + 2, cy - sign_r + 2,
                                cx + sign_r + 2, cy + sign_r + 2,
                                fill="#000000", outline="")
        if limit is not None:
            self.canvas.create_oval(cx - sign_r, cy - sign_r,
                                    cx + sign_r, cy + sign_r,
                                    fill="#cc2200", outline="#881100", width=1)
            inner_r = sign_r - 7
            self.canvas.create_oval(cx - inner_r, cy - inner_r,
                                    cx + inner_r, cy + inner_r,
                                    fill="#f5f5f5", outline="")
            digits    = len(str(limit))
            font_size = 20 if digits <= 2 else 16
            self.canvas.create_text(cx, cy, text=str(limit),
                                    fill="#111111",
                                    font=("Arial", font_size, "bold"))
        else:
            self.canvas.create_oval(cx - sign_r, cy - sign_r,
                                    cx + sign_r, cy + sign_r,
                                    fill="#1e2730", outline="#30363d", width=2)
            self.canvas.create_text(cx, cy, text="—",
                                    fill="#444d56", font=("Arial", 18, "bold"))

        # ── Velocidad actual ────────────────────────────────────
        spd_y = cy + sign_r + 14
        self.canvas.create_text(cx, spd_y,
                                text=f"{self._speed_kmh:.1f} km/h",
                                fill=spd_color, font=("Courier", 12, "bold"))

        # ── Rumbo ───────────────────────────────────────────────
        if self._heading_deg is not None:
            cardinal = bearing_to_cardinal(self._heading_deg)
            self.canvas.create_text(cx, spd_y + 18,
                                    text=f"{self._heading_deg:.0f}° {cardinal}",
                                    fill="#ffd700", font=("Courier", 10))


    # ── Alarma de zona ──────────────────────────────────────

    def _open_alarm_dialog(self):
        """Abre el modal de configuración de alarma."""
        if self._last_render_pil is None or self._render_pos is None:
            messagebox.showinfo("Sin mapa",
                                "Espera a que se descargue el primer mapa.",
                                parent=self.root)
            return
        # Si hay alarma activa, preguntar si desea reemplazarla
        if self._alarm is not None:
            if not messagebox.askyesno("Alarma activa",
                                        "Ya hay una alarma activa.\n¿Deseas reemplazarla?",
                                        parent=self.root):
                return
            # Cancelar alarma anterior
            self._alarm = None
            self._alarm_firing = False
            if self._alarm_sound_stop:
                self._alarm_sound_stop.set()

        dlg = AlarmSetupDialog(self.root, self._last_render_pil, self._render_pos)
        self.root.wait_window(dlg.top)
        if dlg.result is not None:
            lat, lon, radius_m = dlg.result
            self._alarm         = {'lat': lat, 'lon': lon, 'radius_m': radius_m}
            self._alarm_firing  = False

    def _check_alarm(self):
        """Comprueba si la posición GPS está dentro del círculo de alarma."""
        if self._alarm is None or self._alarm_firing:
            return
        if self._render_pos is None:
            return
        lat, lon = self._render_pos
        alat     = self._alarm['lat']
        alon     = self._alarm['lon']
        dist_m   = haversine_km(lat, lon, alat, alon) * 1000.0
        if dist_m <= self._alarm['radius_m']:
            self._trigger_alarm()

    def _trigger_alarm(self):
        """Arranca el sonido y muestra el popup de alarma."""
        self._alarm_firing    = True
        stop_ev               = threading.Event()
        self._alarm_sound_stop = stop_ev
        threading.Thread(target=_alarm_beep_loop, args=(stop_ev,),
                         daemon=True).start()
        self._show_alarm_popup()

    def _show_alarm_popup(self):
        """Muestra un Toplevel de aviso mientras suena la alarma."""
        popup = tk.Toplevel(self.root)
        popup.title("¡ALARMA!")
        popup.configure(bg="#0d1117")
        popup.resizable(False, False)
        popup.transient(self.root)
        popup.grab_set()
        # No permitir cerrarlo con la X sin pasar por Aceptar
        popup.protocol("WM_DELETE_WINDOW", lambda: None)

        # Centrar sobre la ventana principal
        self.root.update_idletasks()
        rx = self.root.winfo_x() + self.root.winfo_width()  // 2
        ry = self.root.winfo_y() + self.root.winfo_height() // 2
        pw, ph = 380, 210
        popup.geometry(f"{pw}x{ph}+{rx - pw//2}+{ry - ph//2}")

        # Icono de advertencia animado (parpadeo de color)
        lbl_icon = tk.Label(popup, text="⚠", font=("Courier", 36),
                            bg="#0d1117", fg="#f85149")
        lbl_icon.pack(pady=(22, 0))

        tk.Label(popup,
                 text="¡Has entrado en el área!",
                 font=("Courier", 15, "bold"),
                 bg="#0d1117", fg="#f85149").pack(pady=(6, 18))

        tk.Button(popup, text="  ✔  Aceptar  ",
                  font=("Courier", 12, "bold"),
                  bg="#dc3232", fg="white",
                  activebackground="#f85149",
                  relief="flat", padx=20, pady=8,
                  command=lambda: self._dismiss_alarm(popup)).pack()

        # Animación de parpadeo del icono
        self._blink_popup(lbl_icon, True, popup)

    def _blink_popup(self, label, state: bool, popup):
        """Alterna el color del icono ⚠ mientras el popup está abierto."""
        try:
            label.configure(fg="#f85149" if state else "#661100")
            popup.after(400, self._blink_popup, label, not state, popup)
        except tk.TclError:
            pass  # popup ya destruido

    def _dismiss_alarm(self, popup):
        """Detiene el sonido, elimina la alarma y cierra el popup."""
        if self._alarm_sound_stop:
            self._alarm_sound_stop.set()
            self._alarm_sound_stop = None
        self._alarm        = None
        self._alarm_firing = False
        try:
            popup.destroy()
        except tk.TclError:
            pass

    # ── Cierre ───────────────────────────────────────────────

    def on_close(self):
        self.stop_event.set()
        self.root.destroy()


# ──────────────────────────────────────────────────────────────
# PUNTO DE ENTRADA
# ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="GPS Map Viewer con IGN PNOA")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help="Puerto serie (p.ej. COM3 o /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help="Baudrate del GPS (por defecto 9600)")
    parser.add_argument("--demo", action="store_true",
                        help="Modo demo sin GPS real (simula posición en Madrid)")
    parser.add_argument("--demo-map", dest="demo_map", action="store_true",
                        help="Demo que recorre los puntos de Mapa_INSIA2.txt "
                             "con velocidad variable para probar los limites")
    parser.add_argument("--demo-speed", dest="demo_speed", type=float, default=15.0,
                        help="Velocidad base en km/h para el modo --demo-map (default: 15)")
    parser.add_argument("--map", default=_DEFAULT_MAP_FILE,
                        help="Ruta al fichero de zonas de velocidad (Mapa_INSIA2.txt)")
    args = parser.parse_args()

    root = tk.Tk()
    app  = GPSMapApp(root, args)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()