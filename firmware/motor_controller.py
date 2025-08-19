#!/usr/bin/env python3
"""
Control de motor paso a paso con DRV8825 en Raspberry Pi 5.
- Oscilación periódica: 100 pasos izquierda ↔ 100 pasos derecha en un hilo dedicado.
- Comandos por teclado para moverse a 8 posiciones absolutas (0..7) igualmente espaciadas.
- Posición 0 es la referencia al iniciar.

Requisitos (Pi 5):
    pip install gpiozero rpi-lgpio
Conexiones típicas (BCM):
    STEP -> GPIO17, DIR -> GPIO27, ENABLE -> GPIO22 (activo en LOW en DRV8825)
Ajusta `PINS_*`, `STEPS_PER_REV` y `MICROSTEP_DIV` según tu hardware.
"""

import threading
import time
import sys
from queue import Queue, Empty

try:
    from gpiozero import DigitalOutputDevice
except ImportError:
    print("\n[ERROR] Falta gpiozero. Instala con: pip install gpiozero rpi-lgpio\n")
    sys.exit(1)

# ==========================
# CONFIGURACIÓN DEL HARDWARE
# ==========================
PINS_STEP = 17     # BCM
PINS_DIR = 27      # BCM
PINS_ENABLE = 22   # BCM (ENABLE del DRV8825 es activo en LOW)

STEPS_PER_REV = 200        # Motor típico 1.8° -> 200 pasos/rev a paso completo
MICROSTEP_DIV = 16         # Ajusta según MS1..MS3 en el driver (1,2,4,8,16,32)
STEP_DELAY_S = 0.0007      # Retardo entre flancos de STEP (define velocidad)

# 8 posiciones igualmente espaciadas (0..7)
NUM_POSITIONS = 8

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
        self.dir_pin.value = True if clockwise else False

    def _pulse(self):
        # flanco de subida cuenta como paso en DRV8825
        self.step_pin.on()
        # Half-period corto; el total entre pulsos lo controla el caller
        time.sleep(self.step_delay / 2)
        self.step_pin.off()
        time.sleep(self.step_delay / 2)

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
# HILO DE OSCILACIÓN 100 ↔ 100
# ==========================
class OscillatorThread(threading.Thread):
    def __init__(self, motor: StepperDRV8825, amplitude_steps: int = 100):
        super().__init__(daemon=True)
        self.motor = motor
        self.amplitude = amplitude_steps
        self._stop_ev = threading.Event()
        self._pause_ev = threading.Event()
        self._pause_ev.clear()  # ejecutando por defecto

    def run(self):
        while not self._stop_ev.is_set():
            if self._pause_ev.is_set():
                time.sleep(0.02)
                continue
            self.motor.step(+self.amplitude)
            time.sleep(0.05)
            self.motor.step(-self.amplitude)
            time.sleep(0.05)

    def stop(self):
        self._stop_ev.set()

    def pause(self, v: bool):
        if v:
            self._pause_ev.set()
        else:
            self._pause_ev.clear()

# ==========================
# BUCLE PRINCIPAL / COMANDOS
# ==========================
HELP = """
Comandos:
  0..7  -> Ir a la posición absoluta (8 slots igualmente espaciados)
  p     -> Pausar/Reanudar oscilación 100↔100
  e     -> Habilitar driver (ENABLE=LOW)
  d     -> Deshabilitar driver (ENABLE=HIGH)
  s N   -> Dar N pasos (positivo horario, negativo antihorario)
  q     -> Salir
"""


def main():
    motor = StepperDRV8825(PINS_STEP, PINS_DIR, PINS_ENABLE,
                           steps_per_rev=STEPS_PER_REV,
                           microstep_div=MICROSTEP_DIV,
                           step_delay_s=STEP_DELAY_S)

    osc = OscillatorThread(motor, amplitude_steps=100)
    osc.start()

    motor.enable()
    print("\n=== Control DRV8825 listo ===")
    print(f"Pasos por vuelta efectivos: {motor.steps_per_rev} (base {STEPS_PER_REV} x microstep {MICROSTEP_DIV})")
    print(HELP)

    paused = False

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "q":
                break
            elif cmd == "p":
                paused = not paused
                osc.pause(paused)
                print("Oscilación", "PAUSADA" if paused else "REANUDADA")
            elif cmd == "e":
                motor.enable()
                print("Driver habilitado")
            elif cmd == "d":
                motor.disable()
                print("Driver deshabilitado")
            elif cmd.startswith("s "):
                try:
                    n = int(cmd.split()[1])
                    osc.pause(True)
                    motor.step(n)
                    print(f"Step manual: {n} -> pos={motor.current_step}")
                    if not paused:
                        osc.pause(False)
                except ValueError:
                    print("Uso: s <entero>")
            elif cmd in list("01234567"):
                slot = int(cmd)
                print(f"Ir al slot {slot} (de {NUM_POSITIONS})…")
                osc.pause(True)
                motor.goto_slot(slot)
                print(f"Llegó a slot {slot}. pos_step={motor.current_step}")
                if not paused:
                    osc.pause(False)
            else:
                print(HELP)

    except KeyboardInterrupt:
        print("\nInterrumpido por usuario")
    finally:
        print("Saliendo…")
        osc.stop()
        motor.disable()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
