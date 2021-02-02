import RPi.GPIO as GPIO
from time import sleep
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import busio
import digitalio
import board
from leds import PwmPin

r = 14 #GPIO14 r
b = 15 #GPIO15 b
g = 18 #GPIO18 g
clk = 25 #GPIO25 clk
dout = 8 #GPIO8 dout
din = 7#GPIO7 din
cs_pin = 1 #GPIO1 cs

# create the spi bus
spi = busio.SPI(clock=clk, MISO=dout, MOSI=din)

# create the cs (chip select)
cs = digitalio.DigitalInOut(cs_pin)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min

    # Convert the left range into a 0-1 range (int)
    valuescaled = int(value - left_min) / int(left_span)

    # Convert the 0-1 range into a value in the right range.
    return int(right_min + (valuescaled * right_span))


class Channel:
    _mcp_object = mcp
    channel_map = {
        "0": MCP.P0,
        "1": MCP.P1,
        "2": MCP.P2,
        "3": MCP.P3,
        "4": MCP.P4,
        "5": MCP.P5,
        "6": MCP.P6,
        "7": MCP.P7
    }
    def __init__(self, channel):
        self.channel = AnalogIn(self._mcp_object, self.channel_map[channel])

class AnalogPin:
    _last_value = 0
    _tolerance = 250
    def __init__(self, channel, pin):
        self.channel = channel
        self.pin = pin

    def value_change_check(self, current_value):
        return abs(current_value - self._last_value) > self._tolerance

    def check_value(self):
        current_value = self.channel.value
        if self.value_change_check(current_value):
            return remap_range(current_value, 0, 65535, 0, 100)
        return self._last_value

    def change_led(self):
        self.pin.pwm_cdc()


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    PinRed = PwmPin(r, "out")
    PinGreen = PwmPin(g, "out")
    PinBlue = PwmPin(b, "out")
    pins = {PinRed, PinGreen, PinBlue}