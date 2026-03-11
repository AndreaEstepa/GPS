# Proyecto de Lectura y Conversión de Datos GPS en Python

Este proyecto es una aplicación en **Python** que lee datos de un dispositivo **GPS** conectado vía **puerto serial** (`/dev/ttyACM0` a `4800` baudios).

## Funcionamiento del archivo `main.py`

El código principal en `main.py` realiza las siguientes tareas:

- Utiliza la biblioteca **pynmea2** para parsear tramas **NMEA**, específicamente las sentencias **`$GPGGA`**, extrayendo **latitud** y **longitud**.
- Convierte las **coordenadas geográficas** a **coordenadas UTM (Universal Transverse Mercator)** utilizando **pyproj**, determinando la **zona** y el **hemisferio** automáticamente.
- Imprime las **coordenadas UTM resultantes** en tiempo real:
  - `easting`
  - `northing`
  - `zona`
  - `hemisferio`
- Maneja **errores de conexión serial** y permite **interrupción manual** (`Ctrl+C`).
- Emplea **threading** para ejecutar la lectura en un **hilo separado**, aunque actualmente solo se utiliza **un hilo**.

## Objetivo del proyecto

El proyecto parece enfocado en el **procesamiento y conversión de datos GPS** para aplicaciones de **posicionamiento**, posiblemente en un entorno **Linux** con hardware GPS compatible.

## Repositorio

- **Repositorio Git:** `git@github.com:AndreaEstepa/GPS.git`  
- **Rama:** `main`
