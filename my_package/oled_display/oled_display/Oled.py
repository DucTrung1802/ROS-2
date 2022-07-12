# Copyright (c) 2017 Adafruit Industries
# Author: Tony DiCola & James DeVito
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from ast import expr
from logging import exception
import time

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

# Raspberry Pi pin configuration:
RST = None  # on the PiOLED this pin isnt used
# Note the following are only used with SPI:
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0
I2C_BUS = 4


# # Alternatively load a TTF font.  Make sure the .ttf font file is in the same directory as the python script!
# # Some other nice fonts to try: http://www.dafont.com/bitmap.php
# # font = ImageFont.truetype('Minecraftia.ttf', 8)


class Oled(object):
    def __init__(self):
        self.__oled = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_bus=I2C_BUS)
        self.__width = self.__oled.width
        self.__height = self.__oled.height
        self.__image = Image.new("1", (self.__width, self.__height), "black")
        self.__draw = ImageDraw.Draw(self.__image)

        self.__debug_index = 0

        self.__oled.begin()
        self.clear()

    def __display(self):
        try:
            self.__oled.display()
            time.sleep(0.01)
        except KeyboardInterrupt:
            self.__oled.display()
            time.sleep(0.01)
            exit(0)
        except:
            self.__debug_index += 1
            print(f"skip read oled {self.__debug_index} time")
            time.sleep(0.01)

    def clear(self):
        self.__draw.rectangle((0, 0, self.__width, self.__height), outline=0, fill=0)
        self.__display()

    def shutdown(self):
        self.__oled.clear()
        self.__draw.rectangle((0, 0, self.__width, self.__height), outline=0, fill=0)
        self.__display()

    def __check_length(self, text, font_set):
        text = str(text)
        if len(text) >= self.__width / font_set["width"]:
            print(f'"{text}" length is larger the size of oled!')

    def __align_calculate(self, text, font_set, horizontal_align):
        x = 0.0
        width_in_pixel = len(text) * font_set["width"]
        if horizontal_align in ["left", "center", "right"]:
            if horizontal_align == "left":
                pass
            elif horizontal_align == "center":
                x = (self.__width - width_in_pixel) / 2.0
            elif horizontal_align == "right":
                x = self.__width - width_in_pixel
        else:
            raise exception("Invalid horizontal align!")
        return x

    def add_text(self, text, font_set, horizontal_align="center", vertical_align=0):

        self.__check_length(text, font_set)
        x = self.__align_calculate(text, font_set, horizontal_align)
        y = float(vertical_align)
        self.__draw.text((x, y), text, font=font_set["font"], fill=255)

    def display(self):
        self.__oled.image(self.__image)
        self.__display()

