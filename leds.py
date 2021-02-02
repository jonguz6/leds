import RPi.GPIO as GPIO
from time import sleep

r = 14 #GPIO14 r
b = 15 #GPIO15 b
g = 18 #GPIO18 g
clk = 25 #GPIO25 clk
dout = 8 #GPIO8 dout
din = 7#GPIO7 din
cs = 1 #GPIO1 cs

class Pin:
    def __init__(self, pin: int, mode: "str"):
        self.pin = pin
        if mode.lower() == "in":
            mode = GPIO.IN
        else:
            mode = GPIO.OUT
        GPIO.setup(self.pin, mode)

class PwmPin(Pin):
    led = None

    def __init__(self, pin: int, mode: "str", freq: int = 50):
        super().__init__(pin, mode)
        self.led = GPIO.PWM(self.pin, freq)
        self.led.start(100)
        self.led.ChangeDutyCycle(0)

    def pwm_cdc(self, value: int):
        self.led.ChangeDutyCycle(value)

    def pwm_range(self, start: int, stop: int, step: int = 1, wait: float = 0.1, two_way: bool = False):
        for value in range(start, stop, step):
            self.pwm_cdc(value)
            sleep(wait)
        if two_way:
            self.pwm_range(stop, start, step=-step)

def rainbow(pin, wait_time: float = 0.1, step: int = 1):
    pin.pwm_range(0, 100, step=step, two_way=True)
    pin.led.ChangeDutyCycle(0)

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    # for i in {r, g, b}:
    #     GPIO.setup(i, GPIO.OUT)
    PinRed = PwmPin(r, "out")
    PinGreen = PwmPin(g, "out")
    PinBlue = PwmPin(b, "out")
    pins = {PinRed, PinGreen, PinBlue}

    while True:
        try:
            rainbow(PinRed)
            rainbow(PinGreen)
            rainbow(PinBlue)
            # PinGreen.pwm_range(0,100, two_way=True)
        except KeyboardInterrupt:
            GPIO.cleanup()
