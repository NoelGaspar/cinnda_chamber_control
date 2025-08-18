


class tempController:
    def __init__(self):
        self.pwm = 0.0
        self.temp = 0.0
        self.temp_sp = 0.0
        self.state = False
        self.pid = None
    
    def set_pwm(self, pwm):
        if pwm < 0.0 or pwm > 100.0:
            raise ValueError("El valor de PWM debe estar entre 0 y 100.")
        self.pwm = pwm
        print(f"PWM ajustado a {self.pwm}%")