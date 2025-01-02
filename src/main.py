import RPi.GPIO  as GPIO
from RPLCD.i2c import CharLCD
import sys
import os
import cv2 as cv
import numpy as np
import time
import smbus

B_L = 7
B_R = 8

TRIG = 17
ECHO = 27	

CAMERA_NUM = 0



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

	def wysw(self,a, b, c, x , y, z):
		self.display_message(f"{a}cm{b}cm{c}cm",f"{x}cm{y}cm{z}cm")

class BoxMeasure():
	
	def __init__(self,camera_num = 0, trig = 17, echo = 27, b_l = 7, b_r = 8):
		
		self.trig = trig
		self.echo = echo
		self.b_l = b_l
		self.b_r = b_r

		try:
			# Ustawienie trybu GPIO
			GPIO.setmode(GPIO.BCM)

			GPIO.setup(trig, GPIO.OUT)
			GPIO.setup(echo, GPIO.IN)
			GPIO.setup(b_l, GPIO.IN, pull_up_down=GPIO.PUD_UP)
			GPIO.setup(b_r, GPIO.IN, pull_up_down=GPIO.PUD_UP)
			
			# Konfiguracja adresu i portu I2C dla LCD
			self.lcd = LCDManager(0x27)

			self.cap = cv.VideoCapture(camera_num)
			if not self.cap.isOpened():
				raise Exception("Cannot open camera")
			
			if not self.cap.isOpened():
				print("Błąd: Nie można otworzyć kamery. Próbuję ponownie...")
				time.sleep(1)
				self.cap = cv.VideoCapture(camera_num)
				if not self.cap.isOpened():
					raise Exception("Cannot open camera")
			self.green_or_black()

			self.h_n = 0
			self.w_n = 0
			self.l_n = 0 
			self.h_l = 0
			self.w_l = 0
			self.l_l = 0

		except Exception("Cannot open camera"):
			print(f"Nie można znaleźć kamery: {camera_num}")
			if camera_num!= 0:
				print("Spróbujemy z kamerą nr 0")
				self.cap = cv.VideoCapture(0)
			else:
				print("Wyłączam program")
				GPIO.cleanup()
				sys.exit(1)
			
		except RuntimeError as e:
			print(f"Błąd konfiguracji GPIO: {e}")
			GPIO.cleanup()
			sys.exit(1)  # Bezpieczne zakończenie programu w przypadku błędu

	def green_or_black(self):
		self.lcd.display_message("L - green","R - black")
		L, R = self.wait_for_button_press()
		self.greenScreen = L
	
	def wait_for_button_press(self):
		L, R = False, False
		while not (L or R):
			L, R = self.check_buttons()
		return L, R

	def check_buttons(self):
		"""
		Checks the state for two buttons.
		When the button was pressed and then released, it returns True for the given button.
		"""
		c_1 = self.check_button(self.b_l)
		c_2 = self.check_button(self.b_r)
		time.sleep(0.1)

		return c_1 == True and self.check_button(self.b_l) == False, c_2 == True and self.check_button(self.b_r) == False

	def check_button(self,button):
		"""
		Checks the state of the button provided as a parameter.
		Returns True if the button is pressed.
		"""
		return GPIO.input(button) == GPIO.LOW

	
	def new_or_rep(self):
		L, R = False, False

		while not (L or R ):
			L, R = self.check_buttons()
		if L==True:
			self.h_l, self.w_l, self.l_l = self.h_n, self.w_n, self.l_n
			self.h_n, self.w_n, self.l_n = self.measure()
		elif R==True:
			self.h_n, self.w_n, self.l_n = self.measure()

	def measure(self):
		self.cap.grab()
		ret, frame = self.cap.read()

		if not ret:
			raise Exception("Can't receive frame")

		if self.greenScreen:
			return self.g_p(frame)
		else:
			return self.b_p(frame)
	
	def g_p(self, frame): #green screen processing
		blur = cv.GaussianBlur(frame,(5,5),0)
		hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

		# Definicja zakresu koloru zielonego w HSV
		lower_green = np.array([20, 0, 0])
		upper_green = np.array([100, 255, 255])

		# Stworzenie maski dla obszarów zielonych
		mask = cv.inRange(hsv, lower_green, upper_green)
		# Odwrócenie maski (czarne to zielone obszary, białe to reszta)
		mask_inv = cv.bitwise_not(mask)

		# Wycięcie tła (green screena)
		fg = cv.bitwise_and(frame, frame, mask=mask_inv)
		#cv.imshow('Green Screen Removal', fg)
		return self.contour(mask_inv)

		
		#cv.imshow("Threshold",mask_inv)

		# Zastąpienie tła innym obrazem (opcjonalnie)
		# bg = cv2.bitwise_and(background, background, mask=mask)
		# result = cv2.add(fg, bg)

	def b_p(self, frame): #black screen processing
		gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
		blur = cv.GaussianBlur(gray,(5,5),0)
		ret,thresh=cv.threshold(blur,160,255,cv.THRESH_BINARY)

		return self.contour(thresh)
	
	def contour(self, f):

		contours, _ = cv.findContours(f, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		if contours:
			largest_contour = max(contours, key=cv.contourArea)
			box = np.zeros_like(f)
			cv.drawContours(box, [largest_contour], -1, (255,255,255), thickness=cv.FILLED)
			x,y,w,h = cv.boundingRect(box)
			#final=frame.copy()
			#final=cv.rectangle(final,(x,y),(x+w,y+h),(0,255,0),2)
			#Przy zalozeniu odleglosci kamery od pudelka 40 cm

			fh=round(0.54*h)
			fw=round(0.54*w)

			try:
				l2=	self.distance()
			except:
				l=0
			else:
				l=35-l2
			finally:
				#print(f"Wysokosc = {h}px Szerokosc = {w}px")
				#print(f"Wysokosc = {fh} cm Szerokosc = {fw} cm Dlugosc = {l} cm")
				#lcd.wysw(fh,fw,l,0,0,0)
				return fh, fw, l
				#cv.imshow("Box",box)
				#cv.imshow("Final",final)
		else:
			return 0, 0, 0

	def distance(self):
		pulse_start = None
		pulse_end = None

		self.output(TRIG, False)
		time.sleep(1) 	# Send the pulse
		GPIO.output(TRIG, True)
		time.sleep(0.00001)
		GPIO.output(TRIG, False)	# Wait for the ECHO pin to go high and record the start time

		while GPIO.input(ECHO) == 0:
			pulse_start = time.time()	# Wait for the ECHO pin to go low and record the end time
		while GPIO.input(ECHO) == 1:
			pulse_end = time.time()	# Calculate the distance based on the time difference

		if pulse_start is None or pulse_end is None:
			raise ValueError("Failed to measure pulse timing")
		pulse_duration = pulse_end - pulse_start
		distance = pulse_duration * 17150
		distance = round(distance, 0)

		return distance
	
	def disp_res(self):
		self.lcd.wysw(self.h_n, self.w_n, self.l_n, self.h_l, self.w_l, self.l_l)
	
	def __del__(self):
		if self.cap:
			self.cap.release()
		GPIO.cleanup()
		print("GPIO , LCD i Kamera zostały wyczyszczone.")

bm = BoxMeasure(camera_num=CAMERA_NUM,trig=TRIG, echo=ECHO, b_l=B_L, b_r=B_R)

try:
	#image = np.zeros((480, 640, 3), dtype=np.uint8)
	#image[:,:,0]=255


	

	#src = cv.imread(cv.samples.findFile("3.jpg"), cv.IMREAD_COLOR)
	#_, fr=cap.read()
	#cv.imwrite("box1.png",fr)
	

	while True:
		bm.new_or_rep()
		bm.disp_res()

		c = cv.waitKey(100)
		if c == 27:
			bm.cleanup()
			bm.lcd.clear()
			break
		elif c == 99: # 99 = ord('c')
			borderType = cv.BORDER_CONSTANT
		elif c == 114: # 114 = ord('r')
			borderType = cv.BORDER_REPLICATE

except KeyboardInterrupt:
	print("Program został zakończony.")
except Exception("Can't receive frame"):
	print("Nie można wczytać klatki. Zakończam program")
except Exception("Cannot open camera"):
	print("Nie można otworzyć kamery")
finally:
	del bm
	cv.destroyAllWindows()
