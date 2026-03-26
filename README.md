# Proyecto GPS con visualizacion en mapa

Aplicacion en Python que lee tramas NMEA por puerto serie, convierte la posicion a UTM y muestra la posicion sobre `etsisi.png` en tiempo real.

## Que hace `main.py`

- Lee sentencias `$GPGGA` desde `/dev/ttyACM0` a `4800` baudios.
- Convierte lat/lon a UTM con `pyproj`.
- Transforma UTM a coordenadas de pixel usando los limites georreferenciados de la imagen.
- Dibuja y actualiza un punto rojo sobre el mapa con `matplotlib`.
- Usa un hilo para adquisicion GPS y el hilo principal para la interfaz grafica.

## Instalacion

```bash
python3 -m pip install -r requirements.txt
```

## Ejecucion

```bash
python3 main.py
```

Si el dispositivo no esta en `/dev/ttyACM0`, cambia `PORT` en `main.py`.
