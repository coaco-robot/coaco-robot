#!/usr/bin/env python

import time
import rospy
from rpi_ws281x import *
from rpi_neopixel.msg import NeopixelMessage

LED_COUNT = 5
LED_PIN = 18
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 255
LED_INVERT = False
LED_CHANNEL = 0


class RpiNeoPixel:
    def __init__(self):
        # Init and start neopixel
        self.strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()

        # Create subscriber
        rospy.init_node("rpi_neopixel")
        rospy.Subscriber("rpi_neopixel", NeopixelMessage, self.callback)
        rospy.spin()

    def clear(self, color, wait_ms=0):
        """ Set every pixel to the same color """
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms/1000.0)

    def callback(self, data):
        """ Subscriber callback """
        if data.animation != "":
            # Some animation code
            self.clear(Color(255, 0, 0), 50)
            self.clear(Color(0, 255, 0), 50)
            self.clear(Color(0, 0, 255), 50)
            self.clear(Color(0, 0, 0))
        
        else:
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, self.num2Color(data.pixels[i]))
                self.strip.show()

    def num2Color(self, number):
        """ Convert uint32 color representation to Color class """
        b = number & 0xFF
        g = (number >> 8) & 0xFF
        r = (number >> 16) & 0xFF

        return Color(r, g, b)

if __name__ == "__main__":
    strip = RpiNeoPixel()
