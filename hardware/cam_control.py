# -*- coding: utf-8 -*-

import cv2
import os
import time
import threading
from datetime import datetime

# ==== Configuración ====
ZOOM_FACTORS = [1, 5, 10]             # Zoom x1, x5, x10
CAPTURE_INTERVAL_MINUTES = 1        # Intervalo en minutos
SAVE_FOLDER = "capturas_micro"       # Carpeta base para guardar imágenes

# Crear carpeta si no existe
os.makedirs(SAVE_FOLDER, exist_ok=True)

data_path = "./" + SAVE_FOLDER



class camController:
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        if not self.cam.isOpened():
            raise Exception("No se pudo abrir la cámara.")
        self.leds_state = 0
        self.running = False
        self.capture_thread = None
    def led(self, _state):
        if _state == 1:
            print("Encendiendo LED...")
            self.leds_state = 1
        else:
            print("Apagando LED...")
            self.leds_state = 0 


    def capture(self):
        ret, frame = self.cam.read()
        if not ret:
            raise Exception("Error al capturar imagen.")
        
        # Obtener fecha y hora actual
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        cv2.imshow("Vista del Microscopio USB", frame)
        time.sleep(10)
        # Ruta de guardado
        filename = f"{SAVE_FOLDER}/img_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f" Imagen guardada: {filename}")
        return frame

    def capture_zoom(self, zoom):
        ret, frame = self.cam.read()
        if not ret:
            raise Exception("Error al capturar imagen.")
        
        # Dimensiones de la imagen
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        if zoom == 1:
            zoomed = frame.copy()
        else:
            rx, ry = int(w // (2 * zoom)), int(h // (2 * zoom))
            cropped = frame[cy - ry:cy + ry, cx - rx:cx + rx]
            zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
        
        cv2.imshow("Vista del Microscopio USB", zoomed)
        time.sleep(10)

        # Obtener fecha y hora actual
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Ruta de guardado
        filename = f"{SAVE_FOLDER}/img_{timestamp}_zoom{zoom}x.jpg"
        cv2.imwrite(filename, zoomed)
        print(f" Imagen guardada: {filename}")
        return zoomed
    
    def video(self):
        self.running = True
        while True:
            ret, frame = self.cam.read()
            if not ret:
                print("Error al capturar imagen.")
                break

            cv2.imshow("Vista del Microscopio USB", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.running = False
                break
            if key == ord('c'):
                # Genera nombre de archivo con la fecha y hora actual
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                filename = f"captura_{timestamp}.png"
                cv2.imwrite(filename, frame)
                print(f"Imagen guardada como: {filename}")
        self.release()

    def release(self):
        self.cam.release()
        cv2.destroyAllWindows()
    def start(self):
        if self.capture_thread is None or not self.capture_thread.is_alive():
            self.capture_thread = threading.Thread(target=self.video)
            self.capture_thread.start()
            self.running = True
        
"""
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error al capturar imagen.")
            break

        # Obtener fecha y hora actual
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Dimensiones de la imagen
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        for zoom in ZOOM_FACTORS:
            if zoom == 1:
                zoomed = frame.copy()
            else:
                rx, ry = int(w // (2 * zoom)), int(h // (2 * zoom))
                cropped = frame[cy - ry:cy + ry, cx - rx:cx + rx]
                zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

            # Ruta de guardado
            filename = f"{SAVE_FOLDER}/img_{timestamp}_zoom{zoom}x.jpg"
            cv2.imwrite(filename, zoomed)
            print(f" Imagen guardada: {filename}")

        # Esperar hasta la siguiente captura
        print(f"🕒 Esperando {CAPTURE_INTERVAL_MINUTES} minutos...")
        time.sleep(CAPTURE_INTERVAL_MINUTES * 60)

except KeyboardInterrupt:
    print("\n Captura interrumpida por el usuario.")

finally:
    cap.release()
    print(" Cámara liberada.")
"""

if __name__ == "__main__":
    cam = camController()
    cam.led(1)
    time.sleep(2)  # Esperar 2 segundos antes de capturar
    test_frame = cam.capture()
    #time.sleep(2)
    cam.capture_zoom(5)
    #time.sleep(2)
    cam.capture_zoom(10)
    cam.led(0)
    time.sleep(30)  # Esperar 2 segundos antes de capturar
    cam.release()