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

        self.__max_length = 21  # Each character has the width of 6 pixels
        self.__max_line = (
            50
        )  # Each character has the width of 10 pixels. Vertical pixel: 0 - 50
        self.__font = ImageFont.load_default()

        self.__oled.begin()
        self.clear()

    def clear(self):
        self.__oled.clear()
        self.__draw.rectangle((0, 0, self.__width, self.__height), outline=0, fill=0)
        self.__oled.display()

    def __check_length(self, text):
        text = str(text)
        if len(text) >= self.__max_length:
            print(f'"{text}" length is larger the size of oled!')

    def __align_calculate(self, text, horizontal_align):
        x = 0.0
        length_in_pixel = len(text) * 6
        if horizontal_align in ["left", "center", "right"]:
            if horizontal_align == "left":
                pass
            elif horizontal_align == "center":
                x = (self.__width - length_in_pixel) / 2
            elif horizontal_align == "right":
                x = self.__width - length_in_pixel
        else:
            raise exception("Invalid horizontal align!")
        return x

    def add_text(self, text, horizontal_align="center", vertical_align=25):
        """_summary_

        Args:
            text (str): _description_ \n
            horizontal_align (str, optional): _description_. Defaults to "center". \n
            vertical_align (int, optional): _description_. Defaults to 25. Range: 0 - 50
        """
        self.__check_length(text)
        x = self.__align_calculate(text, horizontal_align)
        y = float(vertical_align)
        self.__draw.text((x, y), text, font=self.__font, fill=255)

    def display(self):
        self.__oled.image(self.__image)
        self.__oled.display()
