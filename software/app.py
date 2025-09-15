import sys
import json
import time
import datetime
from datetime import datetime
import requests
import csv 
from pathlib import Path
from collections import deque

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap

import paho.mqtt.client as mqtt
import pyqtgraph as pg

from gui import Ui_MainWindow

id = "host_remoto"
mqtt_broker = "35.223.234.244"
mqtt_psw = "!iow_woi!"
mqtt_user = "iowlabs"
MQTT_TOPIC_CMD      = "lab/cam/cmd"
MQTT_TOPIC_STATUS   = "lab/cam/status"


# ==========================
# HILO: Lector de MJPEG
# ==========================
class MjpegStreamThread(QtCore.QThread):
    frameReady = QtCore.pyqtSignal(QtGui.QImage)
    error = QtCore.pyqtSignal(str)
    info = QtCore.pyqtSignal(str)

    def __init__(self, url: str, parent=None, timeout=10):
        super().__init__(parent)
        self.url = url
        self.timeout = timeout

    def run(self):
        try:
            # stream=True nos entrega el cuerpo sin cargar completo
            with requests.get(self.url, stream=True, timeout=self.timeout) as r:
                if r.status_code != 200:
                    self.error.emit(f"HTTP {r.status_code} al abrir {self.url}")
                    return
                ctype = r.headers.get('Content-Type', '')
                # Esperamos algo como: multipart/x-mixed-replace; boundary=frame
                boundary = None
                if 'boundary=' in ctype:
                    boundary = ctype.split('boundary=')[-1].strip()
                if not boundary:
                    # Usa el del servidor del ejemplo
                    boundary = 'frame'
                boundary_bytes = ('--' + boundary).encode()

                self.info.emit(f"Conectado. Content-Type: {ctype}")

                raw = r.raw
                raw.decode_content = True

                # Leemos línea a línea detectando cada parte del multipart
                while not self.isInterruptionRequested():
                    line = raw.readline()
                    if not line:
                        self.error.emit("Conexión cerrada")
                        break
                    if line.startswith(boundary_bytes):
                        # Lee headers de la parte
                        headers = {}
                        while True:
                            h = raw.readline()
                            if not h or h == b"\r\n":
                                break
                            try:
                                k, v = h.decode('utf-8', 'ignore').split(':', 1)
                                headers[k.strip().lower()] = v.strip()
                            except ValueError:
                                pass
                        content_length = int(headers.get('content-length', '0'))
                        if content_length > 0:
                            jpg = raw.read(content_length)
                            # Leer CRLF final de la parte
                            _ = raw.readline()
                            image = QtGui.QImage.fromData(jpg)
                            if not image.isNull():
                                self.frameReady.emit(image)
                        else:
                            # Si no viene Content-Length, intentar leer hasta próxima frontera (menos eficiente)
                            # No implementado aquí por simplicidad; la mayoría de servidores MJPEG lo envían.
                            pass
        except Exception as e:
            self.error.emit(str(e))


class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Set the window title
        self.setWindowTitle("Remote Lab Control")

        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(username = mqtt_user, password = mqtt_psw)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("35.223.234.244", 1883, 60)
        self.mqtt_client.loop_start()

        # Datos para el gráfico
        self.max_points = 600  # 10 minutos si actualiza cada segundo
        self.timestamps = deque(maxlen=self.max_points)
        self.temperatures = deque(maxlen=self.max_points)
        self.setpoints = deque(maxlen=self.max_points)
        self.pwr_outputs = deque(maxlen=self.max_points)
        self.current_setpoint = 25.0
        self.rpm = 0
        
        #logging to CSV
        self.log_dir = Path.cwd() / "logs"
        self.log_dir.mkdir(exist_ok=True)
        self.csv_file = None
        self.csv_writer = None
        self.log_t0 = None  # instante de inicio de logging

        self.state = False
        
        #Imagen de cámara
        pixmap = QPixmap("captura_27.png")  # ruta relativa o absoluta
        self.ui.image_viewer.setPixmap(pixmap)
        self.ui.image_viewer.setMinimumHeight(380)

        #streaming thread
        self.streamThread  = None

        # Gráfico
        # Configurar pyqtgraph en el widget
        self.plot = pg.PlotWidget()
        self.ui.widget_2.layout = pg.QtWidgets.QVBoxLayout(self.ui.widget_2)  # Asegura layout
        self.ui.widget_2.layout.addWidget(self.plot)

        self.plot.setBackground('w')
        self.plot.setLabel('left', 'Potencia (%))')
        self.plot.setLabel('bottom', 'Tiempo (s)')    
        self.plot.showGrid(x = True, y = True)

        pi = self.plot.getPlotItem()
        pi.showAxis('right')
        pi.getAxis('right').setLabel('Temperatura (°C)')

        # ViewBox secundaria para el eje derecho
        self.vb_temp = pg.ViewBox()
        pi.scene().addItem(self.vb_temp)
        pi.getAxis('right').linkToView(self.vb_temp)
        self.vb_temp.setXLink(pi.vb)

        def _updateViews():
            self.vb_temp.setGeometry(pi.vb.sceneBoundingRect())
            self.vb_temp.linkedViewChanged(pi.vb, self.vb_temp.XAxis)

        _updateViews()
        pi.vb.sigResized.connect(_updateViews)
        # Curvas: potencia (izquierda), temperatura y setpoint (derecha)
        self.legend = pi.addLegend()

        # Potencia en naranja (izquierdo)
        self.pwr_curve = pi.plot(pen=pg.mkPen((255,140,0), width=2))


        self.temp_curve = pg.PlotDataItem(pen=pg.mkPen('b', width=2))
        self.setpoint_curve = pg.PlotDataItem(pen=pg.mkPen('r', width=1, style=pg.QtCore.Qt.DashLine))
 
        self.vb_temp.addItem(self.temp_curve)
        self.vb_temp.addItem(self.setpoint_curve)

        self.legend.addItem(self.temp_curve, "Temperatura")
        self.legend.addItem(self.setpoint_curve, "Setpoint")
        self.legend.addItem(self.pwr_curve, "Potencia")

        # Rango fijo 0–100% para potencia (opcional)
        pi.setYRange(0, 100)


        self.ui.comboBox_2.addItems(["zona 1", "zona 2", "zona 3", "zona 4", "zona 5", "zona 6", "zona 7", "zona 8"])
        self.ui.comboBox.addItems(["zoom x1", "zoom x2", "zoom x10", "zoom x100"])
        
        # Timer para actualizar el gráfico
        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.update_plot)
        self.graph_timer.start(50)  # refresco gráfico cada 0.5s

        # Conexión de señales
        self.ui.pushButton.clicked.connect(self.set_setpoint)
        self.ui.pushButton_8.clicked.connect(self.toggle_device)
        self.ui.pushButton_10.clicked.connect(self.set_pid)
        self.ui.pushButton_5.clicked.connect(self.set_home)
        self.ui.pushButton_6.clicked.connect(self.move)
        self.ui.pushButton_7.clicked.connect(self.set_rpm)
        self.ui.pushButton_2.clicked.connect(self.setZoom)
        self.ui.pushButton_4.clicked.connect(self.photoTake)

        self.startPreview()
        
    # --- MQTT Handlers ---

    def on_connect(self, client, userdata, flags, rc):
        print("Conectado al broker con código:", rc)
        client.subscribe(MQTT_TOPIC_STATUS)

    def on_message(self, client, userdata, msg):
        try:
            msg_payload = json.loads(msg.payload.decode('utf-8'))
            print(f"Mensaje recibido: {msg_payload} ")
        except Exception as e:
            print("Error al procesar mensaje:", e)
        try:
            if msg_payload.get("event") == "error":
                print("Error reportado:", msg_payload.get("detail"))
            elif msg_payload.get("event") == "temp":
                temp = msg_payload.get("temp")
                setpoint = msg_payload.get("setpoint", self.current_setpoint)
                pwm_out = msg_payload.get("pwr")
                timestamp = time.time()
                if temp is not None:
                    self.temperatures.append(temp)
                    self.timestamps.append(timestamp)
                    self.setpoints.append(setpoint)
                    self.pwr_outputs.append(pwm_out)
                    print(f"Temperatura actual: {temp} °C, Setpoint: {setpoint} °C")
                    self.write_csv_sample(timestamp, temp, setpoint, pwm_out)       
        except Exception as e:
            print("Error al procesar mensaje:", e)

    # --- UI Actions ---

    def set_setpoint(self, value):
        value = self.ui.doubleSpinBox_2.value()
        self.current_setpoint = value
        print(f"setting set point. new value: {value}")
        payload = json.dumps({"cmd":"setpoint", "sp": value})
        self.mqtt_client.publish(MQTT_TOPIC_CMD, payload)

    def set_pid(self, value):
        _kp = self.ui.doubleSpinBox_3.value()
        _ki = self.ui.doubleSpinBox_4.value()
        _kd = self.ui.doubleSpinBox_5.value()
    
        print(f"setting set point. new value: kp: {_kp}, ki: {_ki}, kd: {_kd}")
        payload = json.dumps({"cmd":"setpid", "kp": _kp, "ki": _ki, "kd": _kd})
        self.mqtt_client.publish(MQTT_TOPIC_CMD, payload)

    def toggle_device(self, checked):
        self.state = not self.state
        self.ui.pushButton_8.setText("Apagar" if self.state else "Encender")
        print(f"Enviando comando de encendido: {self.state}")
        payload = json.dumps({"cmd": "enable_temp", "enable": self.state})
        self.mqtt_client.publish(MQTT_TOPIC_CMD, payload)
        if self.state:
            self.start_csv_log()
        else:
            self.stop_csv_log()

    def set_home(self):
        print("sending home command")
        payload = json.dumps({"cmd": "home", "arg": 1})
        self.mqtt_client.publish(MQTT_TOPIC_CMD,payload)

    def set_rpm(self):    
        value = self.ui.spinBox.value()
        self.rpm = value
        print(f"setting RPM. new value: {value}")
        payload = json.dumps({"cmd": "rpm", "arg": value})
        self.mqtt_client.publish("/temp/control", payload)
    
    def move(self):
        pos = self.ui.comboBox_2.currentText()
        print(f"Moving to position {pos}")
        payload = json.dumps({"cmd": "goto", "pos": pos})
        self.mqtt_client.publish(MQTT_TOPIC_CMD, payload)

    def setZoom(self):
        zoom_text = self.ui.comboBox.currentText()
        if zoom_text == "zoom x1":
            self.zoom = 1
        elif zoom_text == "zoom x2":
            self.zoom = 2
        elif zoom_text == "zoom x10":
            self.zoom = 10
        elif zoom_text == "zoom x100":
            self.zoom = 100
        else:
            print("Zoom option not recognized")
        print(f"Setting digital zoom to {self.zoom}")
        payload = json.dumps({"cmd": "zoom", "arg": self.zoom})
        self.mqtt_client.publish("/temp/control", payload)

    def photoTake(self):
        print(f"Adquiring photo")
        fname=f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        payload = json.dumps({"cmd":"capture", "name": fname})
        self.mqtt_client.publish(MQTT_TOPIC_CMD, payload)

    def update_plot(self):
        if not self.timestamps:
            return
        t0 = self.timestamps[0]
        times = [t - t0 for t in self.timestamps]

        self.temp_curve.setData(times, list(self.temperatures))
        self.setpoint_curve.setData(times, list(self.setpoints))

        # Si pwr_outputs viene 0..1 (duty), escalar a 0..100
        #pwr_pct = [(p or 0)*100 for p in self.pwr_outputs]
        self.pwr_curve.setData(times, list(self.pwr_outputs))

    # --- Preview ---
    def startPreview(self):
        if self.streamThread and self.streamThread.isRunning():
            return
        url = "http://172.30.64.195:8080/preview"
        self.streamThread = MjpegStreamThread(url)
        self.streamThread.frameReady.connect(self.onFrame)
        self.streamThread.error.connect(self.log)
        self.streamThread.info.connect(self.log)
        self.streamThread.start()
        self.log(f"Abriendo preview: {url}")

    def stopPreview(self):
        if self.streamThread:
            self.streamThread.requestInterruption()
            self.streamThread.wait(1500)
            self.streamThread = None
    
    @QtCore.pyqtSlot(QtGui.QImage)
    def onFrame(self, img: QtGui.QImage):
        # Ajusta al tamaño del QLabel manteniendo proporción
        pix = QtGui.QPixmap.fromImage(img)
        if not pix.isNull():
            pix = pix.scaled(self.ui.image_viewer.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            self.ui.image_viewer.setPixmap(pix)
    # ---------- Utilidades ----------
    @QtCore.pyqtSlot(str)
    def log(self, text: str):
        ts = datetime.now().strftime('%H:%M:%S')
        print(f"[{ts}] {text}")

    # --- CSV logger ---
    def start_csv_log(self):
        ts = time.strftime("%Y%m%d_%H%M%S")
        fname = self.log_dir / f"log_{ts}.csv"
        self.csv_file = open(fname, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["iso_time", "t_rel_s", "temp_c", "setpoint_c", "power_pct"])
        self.csv_file.flush()
        self.log_t0 = time.time()
        print(f"[CSV] grabando en: {fname}")

    def stop_csv_log(self):
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            finally:
                self.csv_file = None
                self.csv_writer = None
                self.log_t0 = None
        print("[CSV] archivo cerrado")

    def write_csv_sample(self, timestamp, temp, setpoint, pwr):
        """
        Escribe una fila si el logging está activo.
        timestamp: epoch (seg)
        temp: °C
        setpoint: °C
        pwr: duty 0..1 (se registra como %)
        """
        if self.csv_writer and self.log_t0 is not None:
            t_rel = timestamp - self.log_t0
            pwr_pct = (pwr or 0) * 100 if pwr is not None else ""
            self.csv_writer.writerow([
                datetime.datetime.fromtimestamp(timestamp).isoformat(timespec="seconds"),
                f"{t_rel:.3f}",
                f"{float(temp):.3f}" if temp is not None else "",
                f"{float(setpoint):.3f}" if setpoint is not None else "",
                f"{pwr_pct:.2f}" if pwr is not None else "",
            ])
            # flush ocasional para no perder datos si se corta
            if len(self.timestamps) % 20 == 0:
                self.csv_file.flush()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
