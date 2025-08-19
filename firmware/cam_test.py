#!/usr/bin/env python3
"""
Servicio de cámara para laboratorio remoto (Raspberry Pi 5):
- Preview remoto vía HTTP (MJPEG) en /preview.
- Captura de imagen en alta resolución gatillada por MQTT (topic lab/cam/cmd, payload JSON).
- Publica estados/ack en lab/cam/status.

Requisitos (Raspberry Pi OS Bookworm, Pi 5):
    sudo apt update
    sudo apt install -y python3-picamera2 python3-libcamera python3-opencv
    pip install flask paho-mqtt

NOTA: Usamos Picamera2 (libcamera). El preview se sirve como MJPEG (simple y compatible con navegadores).
La captura usa "switch_mode_and_capture_file" para obtener una foto en resolución de "still" sin perder el preview.
"""

import os
import sys
import cv2
import json
import time
import queue
import errno
import atexit
import signal
import logging
import threading
from datetime import datetime
from typing import Optional, Tuple

from flask import Flask, Response, jsonify
import paho.mqtt.client as mqtt
from picamera2 import Picamera2

# ==========================
# CONFIGURACIÓN
# ==========================
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC_CMD = os.getenv("MQTT_TOPIC_CMD", "lab/cam/cmd")
MQTT_TOPIC_STATUS = os.getenv("MQTT_TOPIC_STATUS", "lab/cam/status")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID", "pi-cam-preview")

HTTP_HOST = os.getenv("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.getenv("HTTP_PORT", "8080"))

CAPTURE_DIR = os.getenv("CAPTURE_DIR", "/home/pi/captures")

# Tamaños y parámetros (ajusta a tu sensor)
PREVIEW_SIZE: Tuple[int, int] = (1280, 720)   # preview (ancho, alto)
PREVIEW_FPS: int = 12
STILL_SIZE: Tuple[int, int] = (4056, 3040)    # HQ cam: (4056, 3040) / Cam3 wide: ajusta según sensor
PREVIEW_JPEG_QUALITY = 80                     # 0..100

# Overlay de debug (timestamp)
DRAW_OVERLAY = True
FONT = cv2.FONT_HERSHEY_SIMPLEX

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s: %(message)s",
)

# ==========================
# CÁMARA (Picamera2)
# ==========================
try:
    picam2 = Picamera2()
except Exception as e:
    logging.error("No se pudo inicializar Picamera2: %s", e)
    sys.exit(1)

# Configuración de video (preview)
video_config = picam2.create_video_configuration(
    main={"size": PREVIEW_SIZE},
    controls={"FrameRate": PREVIEW_FPS},
)
# Configuración de foto (still)
still_config = picam2.create_still_configuration(
    main={"size": STILL_SIZE},
)

picam2.configure(video_config)
picam2.start()

# Lock para acceso exclusivo a la cámara entre preview y captura
cam_lock = threading.RLock()

# Buffer del preview (último JPEG)
latest_jpeg = None
stop_event = threading.Event()


def preview_worker():
    """Hilo que toma frames de la cámara y compone un JPEG para el stream MJPEG."""
    global latest_jpeg
    period = 1.0 / max(1, PREVIEW_FPS)
    while not stop_event.is_set():
        t0 = time.time()
        try:
            with cam_lock:
                frame = picam2.capture_array("main")  # ndarray BGR
            if DRAW_OVERLAY:
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(frame, ts, (10, frame.shape[0] - 10), FONT, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), PREVIEW_JPEG_QUALITY])
            if ok:
                latest_jpeg = enc.tobytes()
        except Exception as e:
            logging.warning("Error en preview_worker: %s", e)
        # Ajuste de FPS
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)


# ==========================
# CAPTURA DE FOTO
# ==========================

def capture_still(filepath: Optional[str] = None) -> str:
    """Captura una imagen en resolución STILL, devuelve la ruta del archivo."""
    os.makedirs(CAPTURE_DIR, exist_ok=True)
    if filepath is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(CAPTURE_DIR, f"capture_{ts}.jpg")

    logging.info("Capturando still → %s", filepath)
    with cam_lock:
        # Cambia a modo still, captura, y vuelve al modo anterior mientras el preview sigue después
        picam2.switch_mode_and_capture_file(still_config, filepath)
    logging.info("Captura completada")
    return filepath


# ==========================
# SERVIDOR HTTP (Flask)
# ==========================
app = Flask(__name__)


@app.route("/preview")
def mjpeg_preview():
    """Endpoint de stream MJPEG compatible con navegadores (multipart/x-mixed-replace)."""
    def generator():
        boundary = b"frame"
        while True:
            if latest_jpeg is None:
                time.sleep(0.05)
                continue
            yield b"--" + boundary + b"\r\n"
            yield b"Content-Type: image/jpeg\r\n"
            yield b"Content-Length: " + str(len(latest_jpeg)).encode() + b"\r\n\r\n"
            yield latest_jpeg + b"\r\n"
    return Response(generator(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/healthz")
def healthz():
    return jsonify({"status": "ok"})


@app.route("/capture")
def http_capture():  # útil para pruebas rápidas sin MQTT
    path = capture_still()
    return jsonify({"captured": path})


# ==========================
# MQTT
# ==========================

def publish_status(client: mqtt.Client, payload: dict):
    try:
        client.publish(MQTT_TOPIC_STATUS, json.dumps(payload), qos=1)
    except Exception as e:
        logging.warning("No se pudo publicar estado MQTT: %s", e)


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info("MQTT conectado a %s:%d", MQTT_BROKER, MQTT_PORT)
        client.subscribe(MQTT_TOPIC_CMD, qos=1)
        publish_status(client, {"event": "online", "preview": f"http://{HTTP_HOST}:{HTTP_PORT}/preview"})
    else:
        logging.error("Fallo al conectar MQTT (rc=%s)", rc)


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
    except Exception as e:
        logging.warning("Payload no JSON: %s", e)
        return

    cmd = payload.get("cmd")
    if cmd == "capture":
        filename = payload.get("name")
        if filename:
            if not filename.lower().endswith(".jpg"):
                filename += ".jpg"
            path = os.path.join(CAPTURE_DIR, filename)
        else:
            path = None
        try:
            path = capture_still(path)
            publish_status(client, {"event": "captured", "path": path})
        except Exception as e:
            logging.exception("Error en captura")
            publish_status(client, {"event": "error", "detail": str(e)})

    elif cmd == "set":
        # Ajuste de controles libcamera, ej: {"cmd":"set", "controls":{"ExposureTime": 8000}}
        controls = payload.get("controls", {})
        try:
            with cam_lock:
                picam2.set_controls(controls)
            publish_status(client, {"event": "controls_set", "controls": controls})
        except Exception as e:
            logging.exception("Error set_controls")
            publish_status(client, {"event": "error", "detail": str(e)})

    else:
        logging.info("Comando desconocido: %s", cmd)


# ==========================
# ARRANQUE / PARADA
# ==========================

def start_threads_and_mqtt():
    # Hilo de preview
    t = threading.Thread(target=preview_worker, name="preview-worker", daemon=True)
    t.start()

    # MQTT
    client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
    client.on_connect = on_connect
    client.on_message = on_message

    # Si usas credenciales:
    # client.username_pw_set(os.getenv("MQTT_USER"), os.getenv("MQTT_PASS"))

    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()
    return t, client


def stop_everything(client: Optional[mqtt.Client] = None):
    logging.info("Deteniendo servicios…")
    stop_event.set()
    try:
        if client:
            client.loop_stop()
            client.disconnect()
    except Exception:
        pass
    try:
        with cam_lock:
            picam2.stop()
    except Exception:
        pass


def main():
    preview_thread, client = start_threads_and_mqtt()

    def _graceful_exit(*_):
        stop_everything(client)
        os._exit(0)

    signal.signal(signal.SIGINT, _graceful_exit)
    signal.signal(signal.SIGTERM, _graceful_exit)

    logging.info("Servicio HTTP escuchando en http://%s:%d", HTTP_HOST, HTTP_PORT)
    app.run(host=HTTP_HOST, port=HTTP_PORT, debug=False, threaded=True)


if __name__ == "__main__":
    main()

