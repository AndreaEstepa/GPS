import serial
import pynmea2
import threading
import queue
from pathlib import Path

import matplotlib.pyplot as plt
from pyproj import Proj


PORT = "/dev/ttyACM0"
BAUDRATE = 4800
SUPIZQ_X = 446231.13
SUPIZQ_Y = 4471354.38
INFDER_X = 446982.81
INFDER_Y = 4470671.80


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
    msg = pynmea2.parse(tramaGga)

    lat = msg.latitude
    lon = msg.longitude

    zone = int((lon + 180) / 6) + 1
    hemisphere = 'north' if lat >= 0 else 'south'

    proj_utm = Proj(proj='utm', zone=zone, ellps='WGS84', hemisphere=hemisphere)

    utm_easting, utm_northing = proj_utm(lon, lat)

    return utm_easting, utm_northing, zone, hemisphere


def getGpsData(pixToMet, positionQueue, stopEvent):
    ser = None

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print("Leyendo datos del GPS...\n")

        while not stopEvent.is_set():
            line = ser.readline().decode("ascii", errors="ignore").strip()

            if line.startswith("$GPGGA"):
                try:
                    pos = ggaToUtm(line)
                except pynmea2.ParseError:
                    continue

                xPix, yPix = transformUTMtoPix(pos[0], pos[1], pixToMet)

                # Solo mantiene el dato mas reciente para que la UI no acumule retraso.
                if positionQueue.full():
                    try:
                        positionQueue.get_nowait()
                    except queue.Empty:
                        pass
                positionQueue.put_nowait((xPix, yPix, pos[0], pos[1]))


    except serial.SerialException as e:
        print(f"Error abriendo el puerto: {e}")
    finally:
        if ser is not None and ser.is_open:
            ser.close()


def showMap(positionQueue, stopEvent, widthImg, heightImg):
    imagePath = Path(__file__).with_name("etsisi.png")
    image = plt.imread(imagePath)

    fig, ax = plt.subplots(figsize=(16, 6))
    ax.imshow(image)
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

    latestPoint = [None, None, None, None]

    def onClose(_):
        stopEvent.set()

    fig.canvas.mpl_connect("close_event", onClose)
    plt.show(block=False)

    while not stopEvent.is_set() and plt.fignum_exists(fig.number):
        while True:
            try:
                latestPoint[:] = positionQueue.get_nowait()
            except queue.Empty:
                break

        if latestPoint[0] is not None:
            xPix, yPix, utmX, utmY = latestPoint
            point.set_offsets([[xPix, yPix]])
            label.set_text(f"UTM: ({utmX:.2f}, {utmY:.2f})  Pix: ({xPix:.1f}, {yPix:.1f})")

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
    stopEvent = threading.Event()

    gpsThread = threading.Thread(
        target=getGpsData,
        args=(pixToMet, positionQueue, stopEvent),
    )
    mapThread = threading.Thread(
        target=showMap,
        args=(positionQueue, stopEvent, widthImg, heightImg),
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
