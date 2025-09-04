import sys
import json
import time
from collections import deque

from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap




import paho.mqtt.client as mqtt
import pyqtgraph as pg

from gui import Ui_MainWindow

id = "lab_remoto"
mqtt_broker = "35.223.234.244"
mqtt_psw = "!iow_woi!"
mqtt_user = "iowlabs"

class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

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
        
        self.state = False
        
        #Imagen de cámara
        pixmap = QPixmap("captura_27.png")  # ruta relativa o absoluta
        self.ui.image_viewer.setPixmap(pixmap)

        # Gráfico
        # Configurar pyqtgraph en el widget
        self.plot = pg.PlotWidget()
        self.ui.widget_2.layout = pg.QtWidgets.QVBoxLayout(self.ui.widget_2)  # Asegura layout
        self.ui.widget_2.layout.addWidget(self.plot)

        self.plot.setBackground('w')
        self.plot.setLabel('left', 'Temperatura (°C)')
        self.plot.setLabel('bottom', 'Tiempo (s)')
        self.plot.addLegend()
        self.plot.showGrid(x = True, y = True)

        self.temp_curve = self.plot.plot(pen=pg.mkPen('b', width=2), name="Temperatura")
        self.setpoint_curve = self.plot.plot(pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine), name="Setpoint")
        self.pwr_curve = self.plot.plot(pen=pg.mkPen('g', width=2), name="Potencia")
        
        self.ui.comboBox_2.addItems(["zona 1", "zona 2", "zona 3", "zona 4", "zona 5", "zona 6", "zona 7", "zona 8"])
        self.ui.comboBox.addItems(["zoom x1", "zoom x2", "zoom x10", "zoom x100"])
        
        # Timer para actualizar el gráfico
        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.update_plot)
        self.graph_timer.start(50)  # refresco gráfico cada 0.5s

        # Conexión de señales
        self.ui.pushButton.clicked.connect(self.set_setpoint)
        self.ui.pushButton_8.clicked.connect(self.toggle_device)
        self.ui.pushButton_5.clicked.connect(self.set_home)
        self.ui.pushButton_6.clicked.connect(self.move)
        self.ui.pushButton_7.clicked.connect(self.set_rpm)
        self.ui.pushButton_2.clicked.connect(self.setZoom)
        self.ui.pushButton_4.clicked.connect(self.photoTake)
        
    # --- MQTT Handlers ---

    def on_connect(self, client, userdata, flags, rc):
        print("Conectado al broker con código:", rc)
        client.subscribe("/lab/cam/status")

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
        except Exception as e:
            print("Error al procesar mensaje:", e)

    # --- UI Actions ---

    def set_setpoint(self, value):
        value = self.ui.doubleSpinBox_2.value()
        self.current_setpoint = value
        print(f"setting set point. new value: {value}")
        payload = json.dumps({"cmd":"setpoint", "arg": value})
        self.mqtt_client.publish("/temp/setpoint", payload)


    def toggle_device(self, checked):
        self.state = not self.state
        self.ui.pushButton_8.setText("Apagar" if self.state else "Encender")
        print(f"Enviando comando de encendido: {self.state}")
        payload = json.dumps({"cmd": "power", "arg": self.state})
        self.mqtt_client.publish("/temp/control", json.dumps(payload))

    def set_home(self):
        print("sending home command")
        payload = json.dumps({"cmd": "home", "arg": 1})
        self.mqtt_client.publish("/temp/control", json.dumps(payload))

    def set_rpm(self):    
        value = self.ui.spinBox.value()
        self.rpm = value
        print(f"setting RPM. new value: {value}")
        payload = json.dumps({"cmd": "rpm", "arg": value})
        self.mqtt_client.publish("/temp/control", json.dumps(payload))

    def move(self):
        pos = self.ui.comboBox_2.currentText()
        print(f"Moving to position {pos}")
        payload = json.dumps({"cmd": "move", "arg": pos})
        self.mqtt_client.publish("/temp/control", json.dumps(payload))

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
        self.mqtt_client.publish("/temp/control", json.dumps(payload))

    def photoTake(self):
        print(f"Adquiring photo")
        payload = json.dumps({"cmd": "take", "arg": 1})
        self.mqtt_client.publish("/temp/control", json.dumps(payload))

    def update_plot(self):
        if not self.timestamps:
            return
        t0 = self.timestamps[0]
        times = [t - t0 for t in self.timestamps]

        self.temp_curve.setData(times, list(self.temperatures))
        self.pwr_curve.setData(times, list(self.pwr_outputs))
        self.setpoint_curve.setData(times, list(self.setpoints))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
