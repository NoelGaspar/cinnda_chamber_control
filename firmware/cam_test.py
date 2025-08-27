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
import random
from datetime import datetime
from typing import Optional, Tuple

from flask import Flask, Response, jsonify, send_from_directory
import paho.mqtt.client as mqtt
from picamera2 import Picamera2

try:
    from gpiozero import DigitalOutputDevice , PWMOutputDevice
except ImportError:
    print("\n[ERROR] Falta gpiozero. Instala con: pip install gpiozero rpi-lgpio\n")
    sys.exit(1)


# ==========================
# CONFIGURACIÓN
# ==========================
PINS_STEP = 17     # BCM
PINS_DIR = 27      # BCM
PINS_ENABLE = 22   # BCM (ENABLE del DRV8825 es activo en LOW)


STEPS_PER_REV = 200        # Motor típico 1.8° -> 200 pasos/rev a paso completo
MICROSTEP_DIV = 16         # Ajusta según MS1..MS3 en el driver (1,2,4,8,16,32)
STEP_DELAY_S = 0.007      # Retardo entre flancos de STEP (define velocidad)

# 8 posiciones igualmente espaciadas (0..7)
NUM_POSITIONS = 8

# MQTT
MQTT_PORT           = 1883
MQTT_TOPIC_CMD      = "lab/cam/cmd"
MQTT_TOPIC_STATUS   = "lab/cam/status"
MQTT_CLIENT_ID      = "lab_remoto"
mqtt_broker         = "35.223.234.244"
mqtt_psw            = "!iow_woi!"
mqtt_user           = "iowlabs"

HTTP_HOST = os.getenv("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.getenv("HTTP_PORT", "8080"))

CAPTURE_DIR = os.getenv("CAPTURE_DIR", "/home/pi/captures")
PUBLIC_BASE_URL = os.getenv("PUBLIC_BASE_URL", f"http://127.0.0.1:{HTTP_PORT}")


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


# =================================
# CLASE DE CONTROL DE TEMPERATURA
# =================================
class tempController:
    def __init__(self, pin_pwm: int):
        self.pwm = PWM(pin_pwm, active_high=True, initial_value=0, frequency=1000)
        self.temp_sensor = None 
        self.temp = 0.0
        self.lock = threading.Lock()
        self.current_duty = 0.0  # 0.0 .. 1.0

    def readTemp(self):
        print("read temp")



    def get_duty_cycle(self) -> float:
        with self.lock:
            return self.current_duty

    def off(self):
        with self.lock:
            self.pwm.value = 0.0
            self.current_duty = 0.0




# ==========================
# CLASE DE CONTROL
# ==========================
class StepperDRV8825:
    def __init__(self, pin_step: int, pin_dir: int, pin_enable: int,
                 steps_per_rev: int = 200, microstep_div: int = 1,
                 step_delay_s: float = 0.001):
        self.step_pin = DigitalOutputDevice(pin_step, active_high=True, initial_value=False)
        self.dir_pin = DigitalOutputDevice(pin_dir, active_high=True, initial_value=False)
        # ENABLE activo en low → usamos active_high=True y ponemos LOW para habilitar
        self.en_pin = DigitalOutputDevice(pin_enable, active_high=True, initial_value=True)  # arranca deshabilitado (HIGH)

        self.steps_full_rev = steps_per_rev
        self.microstep_div = microstep_div
        self.steps_per_rev = steps_per_rev * microstep_div
        self.step_delay = step_delay_s

        # Estado
        self.current_step = 0  # posición absoluta [0 .. steps_per_rev-1]
        self.lock = threading.Lock()

    # ---- Bajo nivel ----
    def enable(self):
        # ENABLE activo en LOW
        self.en_pin.off()  # LOW → habilita

    def disable(self):
        self.en_pin.on()   # HIGH → deshabilita

    def set_dir(self, clockwise: bool):
        # Define el sentido de giro; puede invertirse según cableado
        if clockwise:
            self.dir_pin.on()
        else:
            self.dir_pin.off()

    def _pulse(self):
        # flanco de subida cuenta como paso en DRV8825
        self.step_pin.on()
        # Half-period corto; el total entre pulsos lo controla el caller
        time.sleep(self.step_delay)
        self.step_pin.off()
        time.sleep(self.step_delay)

    # ---- Alto nivel ----
    def step(self, steps: int):
        if steps == 0:
            return
        self.enable()
        clockwise = steps > 0
        self.set_dir(clockwise)
        n = abs(steps)
        for _ in range(n):
            self._pulse()
            with self.lock:
                if clockwise:
                    self.current_step = (self.current_step + 1) % self.steps_per_rev
                else:
                    self.current_step = (self.current_step - 1) % self.steps_per_rev
        # opcional: self.disable()

    def goto_slot(self, slot_index: int, shortest_path: bool = True):
        """Mueve al slot [0..NUM_POSITIONS-1]."""
        slot_index %= NUM_POSITIONS
        target_step = round(slot_index * (self.steps_per_rev / NUM_POSITIONS)) % self.steps_per_rev
        with self.lock:
            curr = self.current_step
        delta = (target_step - curr) % self.steps_per_rev
        if shortest_path and delta > self.steps_per_rev // 2:
            delta = delta - self.steps_per_rev  # camino inverso más corto
        self.step(delta)


# ==========================
# Motor (Stepper DRV8825)
# ==========================
motor = StepperDRV8825(PINS_STEP, PINS_DIR, PINS_ENABLE,
                           steps_per_rev=STEPS_PER_REV,
                           microstep_div=MICROSTEP_DIV,
                           step_delay_s=STEP_DELAY_S)

motor.enable()

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

@app.route("/files/<path:fname>")
def files(fname):
    return send_from_directory(CAPTURE_DIR, fname, as_attachment=False)

# ==========================
# MQTT
# ==========================

def publish_status(client: mqtt.Client, payload: dict):
    try:
        client.publish(MQTT_TOPIC_STATUS, json.dumps(payload), qos=1)
    except Exception as e:
        logging.warning("No se pudo publicar estado MQTT: %s", e)


def on_connect(client, userdata, flags, rc):
    print("código de conexión MQTT:", rc)
    logging.info("MQTT conectado a %s:%d", mqtt_broker, MQTT_PORT)
    client.subscribe(MQTT_TOPIC_CMD, qos=1)
    publish_status(client, {"event": "online", "preview": f"http://{HTTP_HOST}:{HTTP_PORT}/preview"})




def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        print(f"Mensaje recibido en {msg.topic}: {payload}")
    except Exception as e:
        logging.warning("Payload no JSON: %s", e)
        return

    cmd = payload.get("cmd")
    if cmd == "capture":
        filename = payload.get("name")
        logging.warning("capturando imagen")
        if filename:
            if not filename.lower().endswith(".jpg"):
                filename += ".jpg"
            path = os.path.join(CAPTURE_DIR, filename)
        else:
            path = None
        try:
            path = capture_still(path)
            # dentro de on_message(), caso cmd == "capture" (tras obtener path)
            basename = os.path.basename(path)
            url = f"{PUBLIC_BASE_URL}/files/{basename}"
            publish_status(client, {"event": "captured", "path": path, "filename": basename, "url": url})
            logging.warning("imagen enviada")
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

    elif cmd == "goto":
        # go to position
        positions = payload.get("pos")
        try:
            logging.info("moving to slot %d", positions)
            motor.goto_slot(positions)
            publish_status(client, {"event": "goto", "pos": positions})
        except Exception as e:
            logging.exception("Error set_goto cmd")
            publish_status(client, {"event": "error", "detail": str(e)})
    
    elif cmd == "move":
        # go to position
        steps = payload.get("steps")
        try:
            logging.info("moving %d steps", steps)
            motor.step(steps)
            publish_status(client, {"event": "move", "steps": steps})
        except Exception as e:
            logging.exception("Error set_controls")
            publish_status(client, {"event": "error", "detail": str(e)})
    
    elif cmd == "get_temp":
        try:
            logging.info("sending temp")
            temp = random.uniform(20.0, 30.0)  # replace with actual temperature reading
            publish_status(client, {"event": "temp", "temp": temp})
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
     # Si usas credenciales:
    client.username_pw_set(username = mqtt_user, password = mqtt_psw)
    client.on_connect = on_connect
    client.on_message = on_message

   

    client.connect(mqtt_broker, MQTT_PORT, keepalive=60)
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
    app.run(host = HTTP_HOST, port=HTTP_PORT, debug=False, threaded=True)


if __name__ == "__main__":
    main()

