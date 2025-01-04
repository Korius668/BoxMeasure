from RPLCD.i2c import CharLCD

class LCDManager(CharLCD):
	"""
	Class for handling an LCD display.
	"""
	def __init__(self, address):	#Constructor
		CharLCD.__init__(self,'PCF8574', address)	# 'PCF8574' is the model of processor on I2C adapter

	def display_message(self, line1, line2=""):
		self.clear()
		self.write_string(line1)
		if line2:
			self.cursor_pos = (1, 0)
			self.write_string(line2)

	def wysw(self,n_1, a, b, c,n, x , y, z):
		n_1 = n_1 % 100
		n = n % 100

		if a == 0 or b == 0 or c == 0:
			line_1 = "Powtorz pomiar"
		else:	
			line_1 = f"{n_1:2d}#{a:2d}cm{b:2d}cm{c:2d}cm"
		
		line_2 = f"{n:2d}#{x:2d}cm{y:2d}cm{z:2d}cm"

		# Wyświetlenie sformatowanego tekstu na ekranie
		self.display_message(line_1, line_2)