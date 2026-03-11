import serial

PORT = "/dev/ttyACM0"
BAUDRATE = 4800   # prueba 4800 si no funciona

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print("Leyendo datos del GPS...\n")

    while True:
        line = ser.readline().decode("ascii").strip()

        # Mostrar solo tramas importantes
        if line.startswith("$GPGGA") or line.startswith("$GPRMC"):
            print(line)

except serial.SerialException as e:
    print(f"Error abriendo el puerto: {e}")
except KeyboardInterrupt:
    print("\nSaliendo...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()