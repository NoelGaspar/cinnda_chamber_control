import time
import math
import json
import paho.mqtt.client as mqtt

# --- Configuración del simulador ---
BROKER = "35.223.234.244"  # Cambia a la IP o hostname de tu Raspberry
PORT = 1883
TOPIC = "/temp/value"  # Cambia al topic que uses para setpoints
USER = "iowlabs"       # opcional
PASSWORD = "!iow_woi!"   # opcional

FREQ_HZ = 20            # valores por segundo
AMPLITUD = 1.5           # amplitud del seno (ej: 5 grados)
OFFSET = 25            # temperatura base
PERIODO = 60           # duración en segundos de un ciclo completo

sp = {"offset":25}

def on_connect(client, userdata, flags, rc):
    print("Conectado al broker con código:", rc)
    client.subscribe("/temp/setpoint")

def on_message(client, userdata, msg):
        try:
            msg_payload = msg.payload.decode()
            print(f"Mensaje recibido: {msg_payload} °C")
            data = json.loads(msg_payload)
            if data["cmd"] == "setpoint":
                sp["offset"] = data["arg"]
        except Exception as e:
            print("Error al procesar mensaje:", e)

# --- MQTT setup ---
client = mqtt.Client()
client.username_pw_set(USER, PASSWORD)  # omitir si no usas auth
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()


# --- Simulación ---
print("Iniciando simulación de temperatura... (Ctrl+C para detener)")
try:
    t = 0
    while True:
        # Señal seno entre OFFSET ± AMPLITUD
        temp = sp["offset"] + AMPLITUD * math.sin(2 * math.pi * t / PERIODO)
        client.publish(TOPIC, f"{temp:.2f}")
        print(f"Publicando: {temp:.2f} °C")
        time.sleep(1 / FREQ_HZ)
        t += 1
except KeyboardInterrupt:
    print("\nSimulación detenida.")
    client.loop_stop()
    client.disconnect()
