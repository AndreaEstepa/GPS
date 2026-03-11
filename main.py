import serial
import pynmea2

from pyproj import Proj


PORT = "/dev/ttyACM0"
BAUDRATE = 4800   # prueba 4800 si no funciona
TIME = 1
LATITUDE = 2
LATITUDE_NS = 3
LONGITUDE = 4
LONGITUDE_EW = 5
ALTITUDE = 9
ALTITUDE_UNITS = 10


def ggaToUtm(tramaGga):
    msg = pynmea2.parse(tramaGga)

    lat = msg.latitude
    lon = msg.longitude

    zone = int((lon + 180) / 6) + 1
    hemisphere = 'north' if lat >= 0 else 'south'

    proj_utm = Proj(proj='utm', zone=zone, ellps='WGS84', hemisphere=hemisphere)

    utm_easting, utm_northing = proj_utm(lon, lat)

    return utm_easting, utm_northing, zone, hemisphere



try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print("Leyendo datos del GPS...\n")

    while True:
        line = ser.readline().decode("ascii").strip()

        if line.startswith("$GPGGA"):

            print(ggaToUtm((line)))


except serial.SerialException as e:
    print(f"Error abriendo el puerto: {e}")
except KeyboardInterrupt:
    print("\nSaliendo...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()



