import RPi.GPIO  as GPIO
import sys
import os
import cv2 as cv
import numpy as np
import time

GPIO.setmode(GPIO.BCM)
TRIG = 17 
ECHO = 27

GPIO.setup(TRIG, GPIO.OUT) 
GPIO.setup(ECHO, GPIO.IN)

def distance(): 

	GPIO.output(TRIG, False) 
	time.sleep(2) # Send the pulse 
	GPIO.output(TRIG, True) 
	time.sleep(0.00001)
	GPIO.output(TRIG, False) # Wait for the ECHO pin to go high and record the start time 
	
	while GPIO.input(ECHO) == 0: 
		pulse_start = time.time() # Wait for the ECHO pin to go low and record the end time 
	while GPIO.input(ECHO) == 1: 
		pulse_end = time.time() # Calculate the distance based on the time difference 
	pulse_duration = pulse_end - pulse_start 
	distance = pulse_duration * 171500 
	distance = round(distance, 2) 
	return distance 

def contour(contours, mask_inv):
	largest_contour = max(contours, key=cv.contourArea)
	box = np.zeros_like(mask_inv)
	cv.drawContours(box, [largest_contour], -1, (255,255,255), thickness=cv.FILLED)
	x,y,w,h = cv.boundingRect(box)
	final=frame.copy()
	final=cv.rectangle(final,(x,y),(x+w,y+h),(0,255,0),2)
	#Przy zalozeniu odleglosci kamery od pudelka 40 cm

	fh=round(0.54*h)
	fw=round(0.54*w)
	l=350-distance()
	print(f"Wysokosc = {h}px Szerokosc = {w}px")
	print(f"Wysokosc = {fh} mm Szerokosc = {fw} mm Dlugosc = {l} mm")
	#cv.imshow("Box",box)
	cv.imshow("Final",final)

image = np.zeros((480, 640, 3), dtype=np.uint8)
image[:,:,0]=255


cap = cv.VideoCapture(0)
if not cap.isOpened():
	print("Cannot open camera")
	exit()

#src = cv.imread(cv.samples.findFile("3.jpg"), cv.IMREAD_COLOR)
trvol=100
minArea=10000
#_, fr=cap.read()
#cv.imwrite("box1.png",fr)
greenScreen=False

while cap.grab():
	ret, frame = cap.read()

	if not ret:
		print("Can't receive frame (stream end?). Exiting ...")
		break

    #frameDen= cv.fastNlMeansDenoising(gray,3,21,7)

    #cv.imshow('frame', frame)
    #cv.imshow('frame2', frame2)
    #cv.imshow('Denoise', frameDen)

    #   GREEN SCREEN
	if greenScreen:
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
		cv.imshow('Green Screen Removal', fg)
		contours, _ = cv.findContours(mask_inv, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		#cv.imshow("Threshold",mask_inv)
        
        # Zastąpienie tła innym obrazem (opcjonalnie)
        # bg = cv2.bitwise_and(background, background, mask=mask)
        # result = cv2.add(fg, bg)
    
		if contours:
			contour(contours, mask_inv)
            
	else:
		gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
		blur = cv.GaussianBlur(gray,(5,5),0)
		ret,thresh=cv.threshold(blur,160,255,cv.THRESH_BINARY)
		contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		#cv.imshow("Threshold",thresh)   
		if contours:
			contour(contours, thresh)


	c = cv.waitKey(16)
	if c == 27:
		GPIO.cleanup()
		break
	elif c == 43: # 99 = ord('c')
		trvol+=1
		print("Volume:",trvol)
	elif c == 45: # 99 = ord('c')
		trvol-=1
		print("Volume:",trvol)
	elif c == 99: # 99 = ord('c')
		borderType = cv.BORDER_CONSTANT
	elif c == 114: # 114 = ord('r')
		borderType = cv.BORDER_REPLICATE
