import RPi.GPIO  as GPIO
import sys
import time
import cv2 as cv
import numpy as np
import csv
import os
from datetime import datetime
import logging
import os

from LCDManager import LCDManager

def setup_logger(log_file: str):
    """
    Konfiguruje logger zapisujący błędy do pliku.
    """
    # Tworzenie folderu na logi, jeśli nie istnieje
    os.makedirs(os.path.dirname(log_file), exist_ok=True)

    # Konfiguracja logowania
    logging.basicConfig(
        filename=log_file,
        filemode='a',  # 'a' oznacza dopisywanie do istniejącego pliku
        format='%(asctime)s - %(levelname)s - %(message)s',
        level=logging.ERROR  # Loguj tylko błędy i wyższe poziomy
    )

def log_error(message: str):
    """
    Funkcja zapisująca błędy do pliku logów.
    """
    logging.error(message)

class BoxMeasure():
	
	def __init__(self,camera_num = 0, trig = 17, echo = 27, b_l = 7, b_r = 8, file_name="dane.csv"):
		"""
		Inicjalizacja obiektu typu BoxMeasure.
	
		:param 
		camera_num: Numer kamery według numeracji systemu.
		trig: 	Numer pinu na GPIO, według numeracji BCM. Odpowiada za wysyłanie sygnału inicjalizującego pomiar czujnika odległości.
		echo: 	Numer pinu na GPIO, według numeracji BCM. Odpowiada za odbieranie sygnału wysyłanego przez czunik odległości. 
		b_l: 	Numer pinu na GPIO, według numeracji BCM. Odpowiada za odbieranie sygnału z lewego przycisku.
		b_r: 	Numer pinu na GPIO, według numeracji BCM. Odpowiada za odbieranie sygnału z prawego przycisku.
		file_name: Definuje nazwę pliku do którego zapisywane będą wyniki.
		"""

		self.__trig = trig
		self.__echo = echo
		self.__b_l = b_l
		self.__b_r = b_r
		self.__file_name = file_name

		try:
			self.lcd = LCDManager(0x27)
			self.lcd.display_message("Loading GPIO","Prosze czekac")

			# Ustawienie trybu GPIO
			GPIO.setmode(GPIO.BCM)

			GPIO.setup(trig, GPIO.OUT)
			GPIO.setup(echo, GPIO.IN)
			GPIO.setup(b_l, GPIO.IN, pull_up_down=GPIO.PUD_UP)
			GPIO.setup(b_r, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		except RuntimeError as e:
			print(f"Błąd konfiguracji GPIO: {e}")
			log_error(f"Błąd konfiguracji GPIO: {e}")
			GPIO.cleanup()
			sys.exit(1)  # Bezpieczne zakończenie programu w przypadku błędu
		
			
			# Konfiguracja adresu i portu I2C dla LCD
			self.lcd.display_message("Loading Camera","Prosze czekac")
			print("Loading Camera. Prosze czekac")
		try:
			self.cap = cv.VideoCapture(camera_num)

			if not self.cap.isOpened():
				print("Błąd: Nie można otworzyć kamery. Próbuję ponownie...")
				self.lcd.display_message("Can't open camera","Probuje ponownie")
				time.sleep(1)
				self.cap = cv.VideoCapture(camera_num)
				if not self.cap.isOpened():
					raise Exception("Cannot open camera")
		except Exception("Cannot open camera"):
			print(f"Nie można znaleźć kamery: {camera_num}")
			log_error(f"Nie można znaleźć kamery: {camera_num}")
			if camera_num!= 0:
				print("Spróbujemy z kamerą nr 0")
				self.cap = cv.VideoCapture(0)
			else:
				print("Wyłączam program")
				GPIO.cleanup()
				sys.exit(1)
		
		self.__green_or_black()
		
		try:
			self.index, self.h_l, self.w_l, self.l_l = load_last_measurement(self.__file_name)
		except Exception("File is empty"):
			log_error("File is empty")
			self.index, self.h_l, self.w_l, self.l_l = -1,0,0,0
		except Exception("File doesn't exist"):
			log_error("File doesn't exist")
			self.index, self.h_l, self.w_l, self.l_l = -1,0,0,0

			self.h_n = 0
			self.w_n = 0
			self.l_n = 0	
		
	def __green_or_black(self):
		self.lcd.display_message("L - green","R - black")
		L, R = self.__wait_for_button_press()
		self.greenScreen = L
	
	def __wait_for_button_press(self):
		L, R = False, False
		while not (L or R):
			L, R = self.__check_buttons()
		return L, R

	def __check_buttons(self):
		"""
		Checks the state for two buttons.
		When the button was pressed and then released, it returns True for the given button.

		return:	L , R - stany binarne dla dwóch przycisków.
		"""
		c_1 = self.__check_button(self.__b_l)
		c_2 = self.__check_button(self.__b_r)
		time.sleep(0.1)

		return c_1 == True and self.__check_button(self.__b_l) == False, c_2 == True and self.__check_button(self.__b_r) == False

	def __check_button(self,button):
		"""
		Checks the state of the button provided as a parameter.
		Returns True if the button is pressed.

		:param button: Numer pinu odpowiadający za przycisk według BCM.
		return:	Stan binarny dla przycisku.
		"""
		return GPIO.input(button) == GPIO.LOW

	def new_or_rep(self):
		"""
		Dokonuje pomiarów w odpowiedzi na naciśnięte przyciski.
		"""
		L, R = False, False

		while not (L or R ):
			L, R = self.__check_buttons()
		if L==True and self.h_n != 0 and self.w_n != 0 and self.l_n != 0:
			self.h_l, self.w_l, self.l_l = self.h_n, self.w_n, self.l_n
			self.index += 1 
			save_measurement(self.__file_name, self.index, (self.h_n, self.w_n, self.l_n))
			self.h_n, self.w_n, self.l_n = self.__measure()
		elif R==True:
			self.h_n, self.w_n, self.l_n = self.__measure()

	def __measure(self):
		"""
		Zbiorcza funkcja do wykonywania pomiarów. 
		Pobiera obraz z kamery i włacza pomiary odpowiednie od wybranego trybu programu.

		return: h, w, l - wymiary paczki w int
		"""
		try:
			
			self.cap.grab()
			ret, frame = self.cap.read()
			if not ret:
				raise Exception("Can't receive frame")
		except Exception("Can't receive frame"):
			log_error("Can't receive frame")
			for i in range(1,10):
				self.lcd.display_message("Problem z kamera",f"Probuje ponownie{i}")
				time.sleep(1)
				self.cap.grab()
				ret, frame = self.cap.read()
				if ret:
					break

		if self.greenScreen:
			return self.__g_p(frame)
		else:
			return self.__b_p(frame)
	
	def __g_p(self, frame): 
		"""
		Green screen processing
		Przetwarzanie obrazu dla zielonego tła.
		:param
		frame:	Klatka obrazu

		return: h, w, l - wymiary paczki w int
		"""
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
		return self.__contour(mask_inv)

		
		#cv.imshow("Threshold",mask_inv)

		# Zastąpienie tła innym obrazem (opcjonalnie)
		# bg = cv2.bitwise_and(background, background, mask=mask)
		# result = cv2.add(fg, bg)

	def __b_p(self, frame):
		"""
		Black screen processing
		Przetwarzanie obrazu dla czarnego tła.
		:param
		frame:	Klatka obrazu

		return: h, w, l - wymiary paczki w int
		"""
		gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
		blur = cv.GaussianBlur(gray,(5,5),0)
		ret,thresh=cv.threshold(blur,160,255,cv.THRESH_BINARY)

		return self.__contour(thresh)
	
	def __contour(self, bin):
		"""
		Odczytywanie wymiarów paczki z obrazu binarnego.

		:param
		bin:	Klatka obrazu w formacie binarnym

		return: h, w, l - wymiary paczki w int
		"""
		contours, _ = cv.findContours(bin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		if contours:
			largest_contour = max(contours, key=cv.contourArea)
			box = np.zeros_like(bin)
			cv.drawContours(box, [largest_contour], -1, (255,255,255), thickness=cv.FILLED)
			x,y,w,h = cv.boundingRect(box)
			#final=frame.copy()
			#final=cv.rectangle(final,(x,y),(x+w,y+h),(0,255,0),2)
			#Przy zalozeniu odleglosci kamery od pudelka 40 cm

			fh=round(0.54*h)
			fw=round(0.54*w)

			if fh>=100 or fw >= 100:
				fh = 0
				fw = 0

			try:
				l2=	self.__distance()
			except ValueError:
				l=0
				print("Nie udało się uzyskać pomiaru, zwracam wartosc 0")
				log_error("Nie udało się uzyskać pomiaru, zwracam wartosc 0")
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

	def __distance(self):
		pulse_start = None
		pulse_end = None

		GPIO.output(self.__trig, False)
		time.sleep(1) 	# Send the pulse
		GPIO.output(self.__trig, True)
		time.sleep(0.00001)
		GPIO.output(self.__trig, False)	# Wait for the ECHO pin to go high and record the start time

		while GPIO.input(self.__echo) == 0:
			pulse_start = time.time()	# Wait for the ECHO pin to go low and record the end time
		while GPIO.input(self.__echo) == 1:
			pulse_end = time.time()	# Calculate the distance based on the time difference

		if pulse_start is None or pulse_end is None:
			raise ValueError("Failed to measure pulse timing")
		
		pulse_duration = pulse_end - pulse_start
		distance = pulse_duration * 17150
		distance = round(distance, 0)
		distance = int(distance)

		return distance
	
	def disp_res(self):
		self.lcd.wysw(self.index +1,self.h_n, self.w_n, self.l_n,self.index, self.h_l, self.w_l, self.l_l)
					
	def __del__(self):
		if self.cap:
			self.cap.release()
		self.lcd.clear()
		GPIO.cleanup()
		print("GPIO , LCD i Kamera zostały wyczyszczone.")

def load_last_measurement(file_name):
	"""
	Wczytuje ostatni wiersz z pliku CSV.
	:param file_name: Nazwa pliku CSV.
	:return: Ostatni numer i wyniki [wysokość, szerokość, długość] lub None, jeśli plik jest pusty.
	"""
	if not os.path.isfile(file_name):
		raise Exception("File doesn't exist") # Domyślny numer porządkowy i dane

	with open(file_name, mode='r', newline='') as file:
		reader = csv.reader(file)
		rows = list(reader)
		if len(rows) > 1:  # Sprawdź, czy są dane oprócz nagłówków
			last_row = rows[-1]
			return int(last_row[0]), int(last_row[1]), int(last_row[2]), int(last_row[3])
		else:
			raise Exception("File is empty")


def save_measurement(file_name, index, measurement):
	"""
	Zapisuje nowy pomiar do pliku CSV.
	:param file_name: Nazwa pliku CSV.
	:param index: Numer porządkowy.
	:param measurement: Wyniki pomiaru [wysokość, szerokość, długość].
	"""
	file_exists = os.path.isfile(file_name)

	with open(file_name, mode='a', newline='') as file:
		writer = csv.writer(file)
		if not file_exists:  # Dodaj nagłówki, jeśli plik jest nowy
			writer.writerow(['Nr', 'Wysokość (cm)', 'Szerokość (cm)', 'Długość (cm)'])
		writer.writerow([index] + list(measurement))