import os
import sys
import json
import time
from datetime import datetime
import requests
from collections import deque
import paho.mqtt.client as mqtt

# --- Configuración del simulador ---
BROKER = "35.223.234.244"  # Cambia a la IP o hostname de tu Raspberry
PORT = 1883
MQTT_TOPIC_CMD      = "lab/cam/cmd" # a este se envian los comandos
MQTT_TOPIC_STATUS   =  "lab/cam/status" # en este escucho las respuestas
USER = "iowlabs"       # opcional
PASSWORD = "!iow_woi!"   # opcional


SAVE_DIR = os.path.expanduser("./capturas_remotas")

def on_connect(client, userdata, flags, rc):
    print("Conectado al broker con código:", rc)
    client.subscribe(MQTT_TOPIC_STATUS)

def on_message(client, userdata, msg):
        try:
            msg_payload = json.loads(msg.payload.decode('utf-8'))
            print(f"Mensaje recibido: {msg_payload} ")
        except Exception as e:
            print("Error al procesar mensaje:", e)
        try:
            if msg_payload.get("event") == "error":
                print("Error reportado:", msg_payload.get("detail"))
            elif msg_payload.get("event") == "captured":
                print("guardando imagen")
                os.makedirs(SAVE_DIR, exist_ok=True)
                url = msg_payload.get("url")
                name = msg_payload.get("filename") or f"capture_{int(time.time())}.jpg"
                if url:
                    dst = os.path.join(SAVE_DIR, name)
                    print("descargando imagen desde", url)
                    r = requests.get(url, stream=True, timeout=30)
                    r.raise_for_status()
                    with open(dst, "wb") as f:
                        for chunk in r.iter_content(1 << 16):
                            if chunk:
                                f.write(chunk)
                    # aquí puedes notificar en tu UI que ya llegó la imagen
                    print(f"Descargado: {dst}")
        except Exception as e:
            print("Error al guardar imagen:", e)

# --- MQTT setup ---
client = mqtt.Client()
client.username_pw_set(USER, PASSWORD)  # omitir si no usas auth
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()

# --- Simulación ---
print("Iniciando simulación de movimiento... (Ctrl+C para detener)")
time.sleep(30)
try:
    pos = 0
    while True:
        print(f"Moviendo a posición {pos}")
        cmd = json.dumps({"cmd":"goto", "pos": pos})
        client.publish(MQTT_TOPIC_CMD, cmd)
        time.sleep(10)  # espera a que llegue
        fname=f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        cmd = json.dumps({"cmd":"capture", "name": fname})
        client.publish(MQTT_TOPIC_CMD, cmd)
        print(f"adquiriendo una imagen")

        time.sleep(10)
        pos += 1
        if pos > 6:
            pos = 0


except KeyboardInterrupt:
    print("\nSimulación detenida.")
    client.loop_stop()
    client.disconnect()
