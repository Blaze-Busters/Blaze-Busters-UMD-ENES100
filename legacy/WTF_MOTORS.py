from machine import Pin, PWM
import time

# ----- Pin map -----
# Left motor
IN1 = Pin(18, Pin.OUT)
IN2 = Pin(19, Pin.OUT)
ENA = PWM(Pin(22))

# Right motor
IN3 = Pin(21, Pin.OUT)
IN4 = Pin(4,  Pin.OUT)
ENB = PWM(Pin(23))

# Set PWM frequency
for en in (ENA, ENB):
    en.freq(1000)

# Make everything safe
for p in (IN1, IN2, IN3, IN4):
    p.value(0)

# ----- PWM helper -----
def _set_pwm(pwm, frac):
    frac = max(0.0, min(1.0, frac))
    try:
        pwm.duty_u16(int(frac * 65535))
    except AttributeError:
        pwm.duty(int(frac * 1023))

# ----- DC Motor class -----
class DCMotor:
    def __init__(self, in_a: Pin, in_b: Pin, en_pwm: PWM):
        self.in_a = in_a
        self.in_b = in_b
        self.en   = en_pwm
        self.stop()

    def forward(self, speed):
        self.in_a.value(1)
        self.in_b.value(0)
        _set_pwm(self.en, speed / 100.0)

    def stop(self):
        self.in_a.value(0)
        self.in_b.value(0)
        _set_pwm(self.en, 0.0)

# ----- Create motors -----
motor_left  = DCMotor(IN1, IN2, ENA)
motor_right = DCMotor(IN3, IN4, ENB)

# ----- Drive forward -----
speed = 50    # 0-100 %
duration = 200  # seconds

motor_left.forward(speed)
motor_right.forward(speed)

time.sleep(duration)

motor_left.stop()
motor_right.stop()

