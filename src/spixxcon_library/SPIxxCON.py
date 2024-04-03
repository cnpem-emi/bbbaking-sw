if (__name__ == "__main__"):
    exit()

import struct

import Adafruit_BBIO.GPIO as GPIO
import spicon

RSpin = "P9_26"
INTMOSI = "P8_35"
INTMISO = "P8_33"

spi = spicon.open("/dev/spidev1.0")
spicon.set_speed_mode(spi, 3000000, 0)

GPIO.setup(RSpin, GPIO.OUT)
GPIO.setup(INTMISO, GPIO.IN)
GPIO.setup(INTMOSI, GPIO.OUT)
GPIO.output(INTMOSI, GPIO.HIGH)

class funcoes:
	class SPI_dev:
		def transfer(self, buf):
			return spicon.transfer(spi, buf)
			
	def device_select(self, board = 0, device = 0):
		aux = (board & 0x07) << 5
		aux = aux | (self.parity(board) << 4)
		aux = aux | ((device & 0x07) << 1)
		aux = aux | (self.parity(device))
		GPIO.output(RSpin, GPIO.HIGH)
		self.SPI_dev().transfer(chr(aux))
		GPIO.output(RSpin, GPIO.LOW)
		return aux

	def parity(self, data = 0):
		aux = 0
		for i in range(3):
			aux = aux ^ (data >> i)
		return (aux & 0x01)

	def intOut(self):
		GPIO.output(INTMOSI, GPIO.LOW)
		GPIO.output(INTMOSI, GPIO.HIGH)
		return
boards = []
libs = []
for i in range(8):
	try:
		libs.append(__import__(str(i)))
		boards.append(i)
	except:
		pass
for i in range(len(libs)):
	libs[i].details.board = boards[i]
del i
