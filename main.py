import time
from io import BytesIO

import requests
import serial
import pynmea2
import threading
import queue
from pathlib import Path
from urllib.parse import quote

import matplotlib.pyplot as plt
from PIL import Image
from matplotlib.widgets import CheckButtons
from pyproj import Proj
from datetime import datetime


PORT = "/dev/ttyACM0"
BAUDRATE = 4800
SUPIZQ_X = 446231.13
SUPIZQ_Y = 4471354.38
INFDER_X = 446982.81
INFDER_Y = 4470671.80


def loadSpeedMap(filePath):
    """Carga el archivo de velocidades y retorna una lista de puntos (Y, X, velocidad)"""
    speedPoints = []
    try:
        with open(filePath, 'r') as f:
            for line in f:
                line = line.strip()
                if line:
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            y = float(parts[0])
                            x = float(parts[1])
                            speed = float(parts[2])
                            speedPoints.append((y, x, speed))
                        except ValueError:
                            continue
    except FileNotFoundError:
        print(f"Archivo de velocidades no encontrado: {filePath}")
    return speedPoints


def findNearestPoint(currentY, currentX, speedPoints):
    """Encuentra el punto más cercano en speedPoints a la posición actual"""
    if not speedPoints:
        return None

    minDistance = float('inf')
    nearestPoint = None

    for y, x, speed in speedPoints:
        distance = ((currentX - x) ** 2 + (currentY - y) ** 2) ** 0.5
        if distance < minDistance:
            minDistance = distance
            nearestPoint = (y, x, speed)

    return nearestPoint


def getColorBySpeed(speed):
    """Retorna el color basado en la velocidad del punto del fichero"""
    if speed == 5:
        return "#DEA23E"
    elif speed == 15:
        return "cyan"
    elif speed == 10:
        return "#FA3E9A"
    elif speed == 18:
        return "#6B1975"
    elif speed == 20:
        return "yellow"
    else:
        return "red"


def drawMapPoints(ax, speedPoints, pixToMet):
    """Dibuja todos los puntos del mapa con sus velocidades"""
    scatterPoints = []
    colors = []

    for y, x, speed in speedPoints:
        xPix, yPix = transformUTMtoPix(x, y, pixToMet)
        # Solo dibujar puntos que estén dentro del área visible
        if 0 <= xPix and 0 <= yPix:
            scatterPoints.append([xPix, yPix])
            colors.append(getColorBySpeed(speed))

    if scatterPoints:
        scatter = ax.scatter(
            [p[0] for p in scatterPoints],
            [p[1] for p in scatterPoints],
            c=colors,
            s=20,
            alpha=0.6,
            edgecolors='black',
            linewidth=0.3,
            zorder=1
        )
        return scatter
    return None


def getPixMet(widthImg, heightImg):
    deltaX = INFDER_X - SUPIZQ_X
    deltaY = SUPIZQ_Y - INFDER_Y

    pixPerMeterX = widthImg / deltaX
    pixPerMeterY = heightImg / deltaY

    print(f"Pixeles por metro: X={pixPerMeterX:.2f}, Y={pixPerMeterY}")

    return (pixPerMeterX + pixPerMeterY) /2

def transformUTMtoPix(posX, posY, pixToMet):
    return (posX-SUPIZQ_X) * pixToMet, (SUPIZQ_Y-posY) * pixToMet

def ggaToUtm(tramaGga):
    lat,lon,zone,hemisphere = ggaToLatLong(tramaGga)

    proj_utm = Proj(proj='utm', zone=zone, ellps='WGS84', hemisphere=hemisphere)

    utm_easting, utm_northing = proj_utm(lon, lat)

    return utm_easting, utm_northing, zone, hemisphere

def ggaToLatLong(tramaGga):
    msg = pynmea2.parse(tramaGga)

    lat = msg.latitude
    lon = msg.longitude

    zone = int((lon + 180) / 6) + 1
    hemisphere = 'north' if lat >= 0 else 'south'

    return lat,lon,zone,hemisphere


def getDistanceKm(pos1, pos2):
    distanceM = ((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2) ** 0.5
    return distanceM / 1000


def getTimeDiference(previousTime, actualTime):
    return (actualTime - previousTime).total_seconds() / 3600 if previousTime else 0


def getSpeed(distanceKm, time):
    return distanceKm / time


def getGpsData(pixToMet, positionQueue, speedQueue, stopEvent):
    ser = None
    previousPos = None
    previousTime = None
    speed = 0


    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print("Leyendo datos del GPS...\n")

        while not stopEvent.is_set():
            line = ser.readline().decode("ascii", errors="ignore").strip()

            if line.startswith("$GPGGA"):
                try:
                    pos = ggaToLatLong(line)
                    posUTM = ggaToUtm(line)
                except pynmea2.ParseError:
                    continue

                if previousPos is not None:
                    distanceKm = getDistanceKm(previousPos, pos)
                    time = getTimeDiference(previousTime, datetime.now())
                    speed = getSpeed(distanceKm, time)


                previousPos = pos
                previousTime = datetime.now()

                print(f"Posición UTM: ({posUTM[0]:.2f}, {posUTM[1]:.2f}), Velocidad: {speed:.2f} km/h")


                xPix, yPix = transformUTMtoPix(posUTM[0], posUTM[1], pixToMet)

                # Solo mantiene el dato mas reciente para que la UI no acumule retraso.
                if positionQueue.full():
                    try:
                        positionQueue.get_nowait()
                    except queue.Empty:
                        pass
                positionQueue.put_nowait((xPix, yPix, pos[0], pos[1]))

                if speedQueue.full():
                    try:
                        speedQueue.get_nowait()
                    except queue.Empty:
                        pass
                speedQueue.put_nowait(speed)


    except serial.SerialException as e:
        print(f"Error abriendo el puerto: {e}")
    finally:
        if ser is not None and ser.is_open:
            ser.close()

def latLonToBbox(lat, lon, delta=0.005):
    min_lat = lat - delta
    max_lat = lat + delta
    min_lon = lon - delta
    max_lon = lon + delta

    return f"{min_lon},{min_lat},{max_lon},{max_lat}"

def showMap(positionQueue, speedQueue, stopEvent, widthImg, heightImg):
    print(latLonToBbox(40.389, -3.628))
    last_update = 0
    update_interval = 5.0

    request = requests.get(
        "https://www.ign.es/wms-inspire/pnoa-ma",
        params={
            "SERVICE": "WMS",
            "VERSION": "1.3.0",
            "REQUEST": "GetMap",
            "LAYERS": "OI.OrthoimageCoverage",
            "CRS": "EPSG:4326",
            "BBOX": "40.386,-3.633,40.393,-3.623",  # lat_min,lon_min,lat_max,lon_max
            "WIDTH": "1000",
            "HEIGHT": "800",
            "FORMAT": "image/png",
        },
        timeout=10,
    )

    print("HTTP:", request.status_code, "CT:", request.headers.get("Content-Type", ""))

    if request.status_code == 200:
        content_type = request.headers.get("Content-Type", "").lower()

        with open("debug_wms_response.bin", "wb") as f:
            f.write(request.content)

        if "image/png" in content_type or "image" in content_type:
            with open("debug_wms_image.png", "wb") as f:
                f.write(request.content)

            image = Image.open(BytesIO(request.content))
            image.load()  # fuerza decode real
            print("OK imagen PNG")
        else:
            with open("debug_wms_error.xml", "w", encoding="utf-8") as f:
                f.write(request.text)
            print("WMS devolvió texto/XML, no imagen")
    else:
        print("HTTP error:", request.status_code)

    fig, ax = plt.subplots(figsize=(16, 6))
    img_plot = ax.imshow(image)
    ax.set_title("Posicion GPS en tiempo real")
    ax.set_xlim(0, widthImg)
    ax.set_ylim(heightImg, 0)
    ax.grid(alpha=0.2)

    point = ax.scatter([], [], c="red", s=80)
    label = ax.text(
        10,
        25,
        "Esperando datos GPS...",
        color="white",
        bbox=dict(facecolor="black", alpha=0.6, pad=4),
    )

    speedLabel = ax.text(
        widthImg + 100,
        40,
        "Velocidad: 0.00 km/h",
        color="black",
        fontsize=14,
        fontweight="bold",
        bbox=dict(facecolor="green", alpha=0.8, pad=8),
    )

    referenceSpeedLabel = ax.text(
        widthImg + 100,
        100,
        "Velocidad permitida: --",
        color="black",
        fontsize=14,
        fontweight="bold",
        bbox=dict(facecolor="gray", alpha=0.8, pad=8),
    )

    # Cargar mapa de velocidades
    speedMapPath = Path(__file__).with_name("Mapa_INSIA2.txt")
    speedPoints = loadSpeedMap(str(speedMapPath))
    print(f"Puntos de velocidad cargados: {len(speedPoints)}")

    # Dibujar puntos del mapa
    mapPointsScatter = drawMapPoints(ax, speedPoints, pixToMet)
    showMapPoints = [True]  # Se usa lista para modificarla en función anidada

    latestPoint = [None, None, None, None]
    latestSpeed = 0
    referenceSpeed = None

    def getSpeedColor(speed, refSpeed):
        """Calcula el color según la velocidad comparada con la referencia"""
        if refSpeed is None:
            return "gray"

        minSpeed = refSpeed * 0.9
        maxSpeed = refSpeed * 1.1

        if speed < minSpeed:
            return "green"
        elif speed > maxSpeed:
            return "red"
        else:
            return "yellow"

    def onClose(_):
        stopEvent.set()

    # Crear checkbox para mostrar/ocultar puntos del mapa
    ax_checkbox = fig.add_axes((0.01, 0.05, 0.2, 0.08))
    checkbox = CheckButtons(ax_checkbox, ['Mostrar puntos mapa'])
    checkbox.set_active(0, showMapPoints[0])

    def toggle_map_points(label):
        showMapPoints[0] = not showMapPoints[0]
        if mapPointsScatter:
            mapPointsScatter.set_visible(showMapPoints[0])
        plt.draw()

    checkbox.on_clicked(toggle_map_points)

    fig.canvas.mpl_connect("close_event", onClose)
    plt.show(block=False)

    while not stopEvent.is_set() and plt.fignum_exists(fig.number):
        # ...existing code...
        while True:
            try:
                latestPoint[:] = positionQueue.get_nowait()
                latestSpeed = speedQueue.get_nowait()
            except queue.Empty:
                break

        if latestPoint[0] is not None:
            xPix, yPix, lat, lon = latestPoint
            point.set_offsets([[xPix, yPix]])
            label.set_text(f"Lat Lon: ({lat:.2f}, {lon:.2f})  Pix: ({xPix:.1f}, {yPix:.1f})")

            # Encontrar el punto más cercano y obtener velocidad de referencia
            nearestPoint = findNearestPoint(lat, lon, speedPoints)
            if nearestPoint:
                referenceSpeed = nearestPoint[2]

            if time.time() - last_update > update_interval:
                last_update = time.time()

                try:
                    bbox = latLonToBbox(lat, lon)

                    request = requests.get(
                        "https://www.ign.es/wms-inspire/pnoa-ma",
                        params={
                            "SERVICE": "WMS",
                            "VERSION": "1.3.0",
                            "REQUEST": "GetMap",
                            "LAYERS": "OI.OrthoimageCoverage",
                            "CRS": "EPSG:4326",
                            "BBOX": bbox,
                            "WIDTH": "1000",
                            "HEIGHT": "800",
                            "FORMAT": "image/png",
                        },
                        timeout=10,
                    )

                    content_type = request.headers.get("Content-Type", "").lower()
                    print("WMS update -> HTTP:", request.status_code, "CT:", content_type, "BBOX:", bbox)

                    if request.status_code == 200:
                        # Guarda siempre último binario recibido (útil debug)
                        with open("debug_wms_update_response.bin", "wb") as f:
                            f.write(request.content)

                        if "image" in content_type:
                            try:
                                new_image = Image.open(BytesIO(request.content))
                                new_image.load()  # fuerza decode para detectar corrupciones
                                img_plot.set_data(new_image)

                                # opcional: guardar solo última imagen válida
                                with open("debug_wms_update_image.png", "wb") as f:
                                    f.write(request.content)

                            except Exception as img_err:
                                print("Imagen inválida/corrupta en update:", img_err)
                                with open("debug_wms_update_bad_image.bin", "wb") as f:
                                    f.write(request.content)
                        else:
                            print("Respuesta no es imagen (update)")
                            with open("debug_wms_update_error.xml", "w", encoding="utf-8") as f:
                                f.write(request.text[:10000])
                    else:
                        print("Error WMS update HTTP:", request.status_code)
                        with open("debug_wms_update_http_error.txt", "w", encoding="utf-8") as f:
                            f.write(request.text[:10000])

                except Exception as e:
                 print("Error descargando imagen:", e)

        speedColor = getSpeedColor(latestSpeed, referenceSpeed)
        speedLabel.set_text(f"Velocidad: {latestSpeed:.2f} km/h")
        speedLabel.get_bbox_patch().set_facecolor(speedColor)

        # Actualizar etiqueta de velocidad permitida
        if referenceSpeed is not None:
            referenceSpeedLabel.set_text(f"Velocidad permitida: {referenceSpeed:.0f} km/h")
            referenceColor = getColorBySpeed(referenceSpeed)
            referenceSpeedLabel.get_bbox_patch().set_facecolor(referenceColor)
        else:
            referenceSpeedLabel.set_text("Velocidad permitida: --")
            referenceSpeedLabel.get_bbox_patch().set_facecolor("gray")

        plt.pause(0.12)

    if plt.fignum_exists(fig.number):
        plt.close(fig)

def getImageSize(imagePath):
    image = plt.imread(imagePath)
    height, width = image.shape[:2]
    return width, height


if __name__ == "__main__":
    imagePath = Path(__file__).with_name("etsisi.png")
    widthImg, heightImg = getImageSize(imagePath)

    pixToMet = getPixMet(widthImg, heightImg)
    positionQueue = queue.Queue(maxsize=1)
    speedQueue = queue.Queue(maxsize=1)
    stopEvent = threading.Event()

    gpsThread = threading.Thread(
        target=getGpsData,
        args=(pixToMet, positionQueue, speedQueue, stopEvent),
    )
    mapThread = threading.Thread(
        target=showMap,
        args=(positionQueue, speedQueue, stopEvent, widthImg, heightImg),
    )

    gpsThread.start()
    mapThread.start()

    try:
        mapThread.join()
    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        stopEvent.set()
        mapThread.join(timeout=2)
        gpsThread.join(timeout=2)
