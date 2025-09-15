#!/usr/bin/env python3
"""
Host GUI (PyQt5) para laboratorio remoto
- Embebe el preview MJPEG servido por la Raspberry Pi (http://<IP-Pi>:8080/preview)
- Botón para disparar captura por MQTT (lab/cam/cmd → {"cmd":"capture","name":"..."})
- Muestra mensajes de estado recibidos en lab/cam/status

Requisitos:
    pip install PyQt5 requests paho-mqtt

Uso:
    python3 host_gui_mjpeg_mqtt.py

Notas:
- El visor no usa QWebEngine: parsea el stream MJPEG y pinta frames en un QLabel.
- No bloquea el hilo de UI: el parser corre en un QThread.
- Si necesitas autenticación HTTP básica para el preview, amplía la parte de requests.get().
"""

import sys
import json
import time
import requests
from datetime import datetime

from PyQt5 import QtCore, QtGui, QtWidgets
import paho.mqtt.client as mqtt


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


# ==========================
# MQTT Helper (paho-mqtt)
# ==========================
class MqttHelper(QtCore.QObject):
    statusMsg = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal()

    def __init__(self, host='localhost', port=1883, topic_cmd='lab/cam/cmd', topic_status='lab/cam/status', user=None, password=None):
        super().__init__()
        self.host = host
        self.port = port
        self.topic_cmd = topic_cmd
        self.topic_status = topic_status
        self.client = mqtt.Client(client_id=f"host-gui-{int(time.time())}")
        if user and password:
            self.client.username_pw_set(user, password)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

    def start(self):
        try:
            self.client.connect(self.host, self.port, keepalive=60)
            self.client.loop_start()
        except Exception as e:
            self.statusMsg.emit(f"MQTT error: {e}")

    def stop(self):
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected.emit()
            client.subscribe(self.topic_status, qos=1)
            self.statusMsg.emit(f"MQTT conectado a {self.host}:{self.port}")
        else:
            self.statusMsg.emit(f"MQTT fallo conexión (rc={rc})")

    def _on_disconnect(self, client, userdata, rc):
        self.disconnected.emit()
        self.statusMsg.emit("MQTT desconectado")

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            self.statusMsg.emit(f"{msg.topic}: {payload}")
        except Exception:
            self.statusMsg.emit(f"{msg.topic}: <binario {len(msg.payload)} bytes>")

    def publish_capture(self, filename: str = None):
        payload = {"cmd": "capture"}
        if filename:
            payload["name"] = filename
        self.client.publish(self.topic_cmd, json.dumps(payload), qos=1)


# ==========================
# Ventana principal
# ==========================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Host GUI • Preview MJPEG + MQTT Capture")
        self.resize(900, 650)

        # Widgets
        self.urlEdit = QtWidgets.QLineEdit("http://127.0.0.1:8080/preview")
        self.brokerEdit = QtWidgets.QLineEdit("localhost:1883")
        self.topicCmdEdit = QtWidgets.QLineEdit("lab/cam/cmd")
        self.topicStatusEdit = QtWidgets.QLineEdit("lab/cam/status")
        self.connectBtn = QtWidgets.QPushButton("Conectar Preview")
        self.disconnectBtn = QtWidgets.QPushButton("Desconectar Preview")
        self.disconnectBtn.setEnabled(False)

        self.captureNameEdit = QtWidgets.QLineEdit(datetime.now().strftime("capt_%Y%m%d_%H%M%S.jpg"))
        self.captureBtn = QtWidgets.QPushButton("Capturar (MQTT)")

        self.statusBox = QtWidgets.QPlainTextEdit()
        self.statusBox.setReadOnly(True)
        self.statusBox.setMaximumBlockCount(500)

        self.videoLabel = QtWidgets.QLabel()
        self.videoLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.videoLabel.setStyleSheet("background:#111; color:#ddd; border:1px solid #333")
        self.videoLabel.setMinimumHeight(380)
        self.videoLabel.setText("Preview no conectado")

        # Layout
        topForm = QtWidgets.QFormLayout()
        topForm.addRow("URL preview:", self.urlEdit)
        topForm.addRow("Broker MQTT:", self.brokerEdit)
        topForm.addRow("Topic cmd:", self.topicCmdEdit)
        topForm.addRow("Topic status:", self.topicStatusEdit)

        btnRow = QtWidgets.QHBoxLayout()
        btnRow.addWidget(self.connectBtn)
        btnRow.addWidget(self.disconnectBtn)
        btnRow.addStretch(1)
        btnRow.addWidget(QtWidgets.QLabel("Nombre archivo:"))
        btnRow.addWidget(self.captureNameEdit)
        btnRow.addWidget(self.captureBtn)

        central = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(central)
        v.addLayout(topForm)
        v.addLayout(btnRow)
        v.addWidget(self.videoLabel, 1)
        v.addWidget(self.statusBox, 1)
        self.setCentralWidget(central)

        # Conexiones
        self.connectBtn.clicked.connect(self.startPreview)
        self.disconnectBtn.clicked.connect(self.stopPreview)
        self.captureBtn.clicked.connect(self.doCapture)

        # Estado
        self.streamThread = None
        self.mqtt = None

    # ---------- Preview ----------
    def startPreview(self):
        if self.streamThread and self.streamThread.isRunning():
            return
        url = self.urlEdit.text().strip()
        self.streamThread = MjpegStreamThread(url)
        self.streamThread.frameReady.connect(self.onFrame)
        self.streamThread.error.connect(self.log)
        self.streamThread.info.connect(self.log)
        self.streamThread.start()
        self.connectBtn.setEnabled(False)
        self.disconnectBtn.setEnabled(True)
        self.log(f"Abriendo preview: {url}")
        self.ensureMqtt()

    def stopPreview(self):
        if self.streamThread:
            self.streamThread.requestInterruption()
            self.streamThread.wait(1500)
            self.streamThread = None
        self.connectBtn.setEnabled(True)
        self.disconnectBtn.setEnabled(False)
        self.videoLabel.setText("Preview detenido")

    @QtCore.pyqtSlot(QtGui.QImage)
    def onFrame(self, img: QtGui.QImage):
        # Ajusta al tamaño del QLabel manteniendo proporción
        pix = QtGui.QPixmap.fromImage(img)
        if not pix.isNull():
            pix = pix.scaled(self.videoLabel.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            self.videoLabel.setPixmap(pix)

    # ---------- MQTT ----------
    def ensureMqtt(self):
        if self.mqtt:
            return
        hostport = self.brokerEdit.text().strip()
        try:
            host, port = hostport.split(':', 1)
            port = int(port)
        except ValueError:
            host, port = hostport, 1883

        topic_cmd = self.topicCmdEdit.text().strip() or 'lab/cam/cmd'
        topic_status = self.topicStatusEdit.text().strip() or 'lab/cam/status'
        self.mqtt = MqttHelper(host, port, topic_cmd=topic_cmd, topic_status=topic_status)
        self.mqtt.statusMsg.connect(self.log)
        self.mqtt.connected.connect(lambda: self.log("Conectado a broker"))
        self.mqtt.disconnected.connect(lambda: self.log("Desconectado del broker"))
        self.mqtt.start()

    def doCapture(self):
        self.ensureMqtt()
        name = self.captureNameEdit.text().strip() or None
        self.mqtt.publish_capture(name)
        self.log(f"→ MQTT capture: {name or '(autonombre)'}")

    # ---------- Utilidades ----------
    @QtCore.pyqtSlot(str)
    def log(self, text: str):
        ts = datetime.now().strftime('%H:%M:%S')
        self.statusBox.appendPlainText(f"[{ts}] {text}")

    def closeEvent(self, e: QtGui.QCloseEvent):
        self.stopPreview()
        if self.mqtt:
            self.mqtt.stop()
        return super().closeEvent(e)


# ==========================
# main
# ==========================
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
