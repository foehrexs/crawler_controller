#! /usr/bin/python3
#-*- coding: utf-8 -*-

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106, ssd1306
from PIL import ImageFont, ImageDraw, Image

# I2C Verbindung erstellen
serial = i2c(port=1, address=0x3C)

# Display initialisieren
device = sh1106(serial)

# Text für OLED-Display anzeigen

oled_font = ImageFont.truetype('FreeSans.ttf',14)
with canvas(device) as draw:
	draw.rectangle(device.bounding_box, outline = "white", fill = "black")
	draw.text((10, 10), "Hallo! \n Display geht!", font=oled_font, fill = "white")

# Endlosschleife
try:
    while True:
        pass
except KeyboardInterrupt:
    pass
