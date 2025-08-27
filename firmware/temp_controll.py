#!/usr/bin/env python3
"""
Control de temperatura para Raspberry Pi 5
- Lectura de temperatura con ADS1115 (Adafruit) + NTC (modelo Beta/Steinhart-Hart simplificado).
- Salida PWM para calefactor con MOSFET (IRF/IRL) usando gpiozero.
- Control PID con simple-pid.
- Hilo propio de control con sample_time fijo.

Dependencias recomendadas (Bookworm):
    sudo apt update
    sudo apt install -y python3-adafruit-blinka python3-adafruit-circuitpython-ads1x15 \
                        python3-gpiozero python3-rpi-lgpio
    # En tu venv (con --system-site-packages), instala simple-pid:
    python -m pip install simple-pid

Asegúrate de habilitar I2C en la Pi.

Esquema típico divisor (Vcc=3.3V):
    Vcc -- R_SERIE --+-- NTC -- GND
                     |
                   (A0)

Donde (A0) es el canal del ADS1115. La fórmula de Rntc asume NTC a GND.
"""

from __future__ import annotations
import threading
import time
import math
import logging
from dataclasses import dataclass, asdict
from typing import Optional, Deque
from collections import deque

# GPIO / PWM
from gpiozero import PWMOutputDevice

# ADS1115 (Adafruit CircuitPython)
import board
import busio
from adafruit_ads1x15.ads1115 import ADS1115, P0, P1, P2, P3
from adafruit_ads1x15.analog_in import AnalogIn

# PID
from simple_pid import PID

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')


# -------------------------
# Temperatura config
# -------------------------

VCC         = 3.3          # Voltaje del divisor
R_series    = 10000.0      # Ohmios del resistor en serie
R0          = 10000.0      # Ohmios a T0
T0_C        = 25.0         # °C de referencia
Beta        = 3950.0       # Constante Beta del NTC

ads1115_address = 0x48       # Dirección I2C del ADS1115
default_channel = 0          # 0..3
ads1115_gain    = 1             # ±4.096V → suficiente para 3.3V
ads_115_rate    = 250      # sps
avg_n           = 5 


pin_bcm         = 18         # GPIO con PWM por software (gpiozero) – ajusta
frequency_hz    = 1000  # 1 kHz suele ir bien para calefactor
active_high     = True

# PID vars
Kp = 30.0
Ki = 5.0
Kd = 10.0
sample_time = 0.5     # s – periodo del lazo
output_min  = 0.0     # duty min
output_max  = 1.0     # duty max
p_on_m      = True    # proporcional sobre medición


# -------------------------
# Lectura de temperatura (ADS1115 + NTC)
# -------------------------
class TempSensorADS1115NTC:
    def __init__(self):
        self._i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        self._ads = ADS1115( self._i2c, address = ads1115_address )
        
        self._ads.gain = ads1115_gain
        self._ads.data_rate = ads_115_rate

        self._ain = AnalogIn(self._ads, ch)
        
        self._avg_n = max(1, avg_n)
        self._avg_buf = deque(maxlen=self._avg_n)

    def _voltage(self) :
        # AnalogIn.voltage ya compensa el gain y referencia interna
        return float(self._ain.voltage)

    def _moving_avg(self, v):
        self._avg_buf.append(v)
        return sum(self._avg_buf) / len(self._avg_buf)

    def read_temperature_c(self):
        """Lee voltaje, estima Rntc, aplica modelo Beta y devuelve °C.
        Asume divisor: Vcc -- R_series --(Vmeas)-- NTC -- GND
        """
        Vmeas = self._voltage()
        Vcc   = VCC

        if Vmeas <= 0.0 or Vmeas >= Vcc:
            raise RuntimeError(f"Voltaje fuera de rango Vmeas={Vmeas:.3f}V")
        
        # Rntc = R_series * Vntc / (Vcc - Vntc)
        Rntc = R_series * (Vmeas / (Vcc - Vmeas))
        # Beta model
        T0_K = T0_C + 273.15
        lnRR0 = math.log(Rntc / R0)
        invT = 1.0/T0_K + (1.0/Beta) * lnRR0
        T_K = 1.0 / invT
        T_C = T_K - 273.15
        # Filtros suaves
        #T_med = self._median_filter(T_C)
        T_avg = self._moving_avg(T_med)
        return T_avg


# -------------------------
# Actuador calefactor (PWM)
# -------------------------
class HeaterPWM:
    def __init__(self, pwm_cfg: PWMConfig):
        self.dev = PWMOutputDevice(pin=pwm_cfg.pin_bcm,
                                   active_high=pwm_cfg.active_high,
                                   initial_value=0.0,
                                   frequency=pwm_cfg.frequency_hz)
        self._duty = 0.0

    def set(self, duty):
        d = max(0.0, min(1.0, float(duty))) # PWM on raspberry go from 0.0  to  1.0
        self.dev.value = d
        self._duty = d

    def duty(self):
        return self._duty

    def off(self):
        self.set(0.0)


# -------------------------
# Controlador PID en hilo
# -------------------------
class TemperatureController(threading.Thread):
    def __init__(self,
                 sensor: TempSensorADS1115NTC,
                 heater: HeaterPWM,
                 pid_cfg: PIDConfig,
                 max_temp_cutoff_c: float = 70.0):
        super().__init__(daemon=True)
        self.sensor = sensor
        self.heater = heater
        self.pid = PID( Kp, Ki, Kd,
                       setpoint = 25.0,  # default
                       sample_time = sample_time,
                       output_limits=(output_min, output_max),
                       proportional_on_measurement = p_on_m)
        self.enabled = True
        self.max_temp_cutoff_c = max_temp_cutoff_c
        self._stop = threading.Event()
        self._last_read_c: Optional[float] = None
        self._last_out: float = 0.0
        self._fault: Optional[str] = None

    # API externa
    def set_setpoint(self, t_c):
        self.pid.setpoint = float(t_c)

    def set_pid(self, kp: float, ki: float, kd: float):
        self.pid.tunings = (float(kp), float(ki), float(kd))

    def enable(self, v: bool):
        self.enabled = bool(v)
        if not v:
            self.heater.off()

    def stop(self):
        self._stop.set()

    def get_status(self):
        return {
            'enabled': self.enabled,
            'setpoint_c': self.pid.setpoint,
            'temp_c': self._last_read_c,
            'duty': self._last_out,
            'fault': self._fault,
        }

    # Hilo de control
    def run(self):
        period = self.pid.sample_time or 0.5
        while not self._stop.is_set():
            t0 = time.time()
            try:
                t_c = self.sensor.read_temperature_c()
                self._last_read_c = t_c
                # Seguridad
                if t_c >= self.max_temp_cutoff_c:
                    self._fault = f"Corte por sobretemperatura: {t_c:.2f}°C >= {self.max_temp_cutoff_c:.2f}°C"
                    self.heater.off()
                    time.sleep(period)
                    continue
                if not self.enabled:
                    self.heater.off()
                    time.sleep(period)
                    continue
                # PID: pasar medición devuelve 'duty'
                out = self.pid(t_c)
                self.heater.set(out)
                self._last_out = out
                self._fault = None
            except Exception as e:
                logging.warning("Fallo lectura/control: %s", e)
                self._fault = str(e)
                self.heater.off()
            # Mantener periodo
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)


# -------------------------
# Ejemplo de uso
# -------------------------
if __name__ == '__main__':
    ads_cfg = ADSConfig(address=0x48, channel=0, gain=1, data_rate=250)
    ntc_cfg = NTCConfig(Vcc=3.3, R_series=10000.0, R0=10000.0, T0_C=25.0, Beta=3950.0)
    pwm_cfg = PWMConfig(pin_bcm=18, frequency_hz=1000, active_high=True)
    pid_cfg = PIDConfig(Kp=20.0, Ki=2.5, Kd=5.0, sample_time=0.5, output_min=0.0, output_max=1.0, p_on_m=True)

    sensor = TempSensorADS1115NTC(ads_cfg, ntc_cfg, median_k=3, avg_n=5)
    heater = HeaterPWM(pwm_cfg)
    ctrl = TemperatureController(sensor, heater, pid_cfg, max_temp_cutoff_c=60.0)

    ctrl.set_setpoint(37.0)
    ctrl.start()

    try:
        for _ in range(60):
            st = ctrl.get_status()
            logging.info(f"T={st['temp_c']:.2f}°C  SP={st['setpoint_c']:.2f}°C  duty={st['duty']:.2f}  fault={st['fault']}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop()
        heater.off()
        logging.info("Control detenido.")
