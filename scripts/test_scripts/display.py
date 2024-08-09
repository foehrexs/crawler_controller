#!/usr/bin/env python3

import Adafruit_SSD1306
import time
from PIL import Image, ImageDraw, ImageFont


#Display initialisieren
disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, i2c_address=0x3C)

def initialize_display():
	disp.begin()
	disp.clear()
	disp.display()
	
#Anzeigefunktion	
def display_message(message):
	width = disp.width
	height = disp.height
	image = Image.new('1', (width, height))
	draw = ImageDraw.Draw(image)
	
#Standart Schriftart verwenden
	font = ImageFont.load_default()
	
#Text auf Display schreiben
	draw.text((0, 0), message, font=font, fill=255)
	
#Aktualisierung des Displays
	disp.image(image)
	disp.display()
	
	
def turn_off_display():
	disp.clear()
	disp.display()
	disp.command(0xAE) # SSD1306_DISPLAYOFF
	
def turn_on_display():
	disp.command(0xAF)	# SSD1306_DISPLAYON
	disp.display()
	
if __name__ == "__main__":
	initialize_display()
	time.sleep(1)		# Verzögerung von 1 Sekunde
	display_message("Servus!")
	time.sleep(1)		# Verzögerung von 10 Sekunde
	
	input("Das Display ist jetzt an. Drücken Sie eine beliebige Taste, um das Display auszuschalten...")
	turn_off_display()
	
	input("Das Display ist jetzt aus. Drücken Sie eine beliebige Taste, um das Display einzuschalten...")
	turn_on_display()
	
	input("Das Display ist jetzt wieder an. Drücken Sie eine beliebige Taste, um das Script zu beenden...")

