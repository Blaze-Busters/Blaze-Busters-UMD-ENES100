from machine import Pin, PWM
import time

# -----Pin map-----
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
    if frac < 0:  frac = 0.0
    if frac > 1:  frac = 1.0
    try:
        pwm.duty_u16(int(frac * 65535))
    except AttributeError:
        pwm.duty(int(frac * 1023))


# ----- DC Motor class -----
class DCMotor:
    def __init__(self, in_a: Pin, in_b: Pin, en_pwm: PWM,
                 brake_stop=False, invert=False, gain=1.0):
        self.in_a = in_a
        self.in_b = in_b
        self.en   = en_pwm
        self.brake = brake_stop
        self.inv   = invert
        self.gain  = gain
        self.stop()

    def _dir(self, forward=True):
        fwd = forward ^ self.inv
        if fwd:
            self.in_a.value(1); self.in_b.value(0)
        else:
            self.in_a.value(0); self.in_b.value(1)

    def prepare_start(self, speed):
        """Set direction and return final speed with gain applied."""
        if speed is None:
            speed = 0

        speed *= self.gain
        speed = max(min(speed, 100), -100)

        if speed == 0:
            return 0

        self._dir(forward=(speed > 0))
        return speed

    def apply_pwm(self, speed):
        """Apply steady-state (non-kick) speed."""
        if speed == 0:
            self.stop()
            return

        _set_pwm(self.en, abs(speed) / 100.0)

    def stop(self):
        if self.brake:
            self.in_a.value(1); self.in_b.value(1)
        else:
            self.in_a.value(0); self.in_b.value(0)
        _set_pwm(self.en, 0.0)


# ----- CREATE MOTORS -----
motor_left  = DCMotor(IN1, IN2, ENA, brake_stop=False, gain=1)
motor_right = DCMotor(IN3, IN4, ENB, brake_stop=False, gain=1)


# ----- motors_spin with INDEPENDENT KICK STRENGTH -----
def motors_spin(duration, speed_left, speed_right,
                kick_time=0.1,
                kick_left_strength=100,   # 0–100
                kick_right_strength=100): # 0–100
    """
    kick_left_strength / kick_right_strength: percent power during kick
    """

    # Prepare (set direction)
    sL = motor_left.prepare_start(speed_left)
    sR = motor_right.prepare_start(speed_right)

    if sL == 0 and sR == 0:
        time.sleep(duration)
        return

    # Convert strength % → duty fraction (0–1)
    kL = max(0, min(kick_left_strength, 100)) / 100.0
    kR = max(0, min(kick_right_strength, 100)) / 100.0

    # Kick both motors using their individual strengths
    if sL != 0: _set_pwm(ENA, kL)
    if sR != 0: _set_pwm(ENB, kR)

    time.sleep(kick_time)

    # Steady PWM
    motor_left.apply_pwm(sL)
    motor_right.apply_pwm(sR)

    time.sleep(duration)

    # Stop at the end
    motor_left.stop()
    motor_right.stop()
    

# ----- Continuous motor control (ON/OFF) -----

def motor_on(speed_left, speed_right,
             kick_time=0.1,
             kick_left_strength=100,
             kick_right_strength=100):

    # Prepare direction
    sL = motor_left.prepare_start(speed_left)
    sR = motor_right.prepare_start(speed_right)

    if sL == 0 and sR == 0:
        return

    # Convert strength % → duty fraction
    kL = max(0, min(kick_left_strength, 100)) / 100.0
    kR = max(0, min(kick_right_strength, 100)) / 100.0

    # Kick phase
    if sL != 0: _set_pwm(ENA, kL)
    if sR != 0: _set_pwm(ENB, kR)
    time.sleep(kick_time)

    # Hold steady speed
    motor_left.apply_pwm(sL)
    motor_right.apply_pwm(sR)


def motor_off():
    motor_left.stop()
    motor_right.stop()


# ----- TEST -----
#FIRST FUNCTION
motors_spin(5, 80, 80,
            kick_time=0.1,
            kick_left_strength=90,
            kick_right_strength=100)
#SECOND FUNCTION
motor_on(80, 80, kick_left_strength=90, kick_right_strength=100)
motor_off()



