from hardware.cam_control import camController
from hardware.temp_control import tempController
import time


cam = camController()
cam.led(1)
cam.start()
cam.led(0)
time.sleep(200)  # Esperar 2 segundos antes de capturar

cam.capture_thread.join(0.1)
cam.release()