import sys
import os
import cv2 as cv
import numpy as np
import time


image = np.zeros((480, 640, 3), dtype=np.uint8)
image[:,:,0]=255

# to start real-time feed
cap = cv.VideoCapture(3)
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

    cv.imshow('frame', frame)
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
        cv.imshow("Threshold",mask_inv)
        
        # Zastąpienie tła innym obrazem (opcjonalnie)
        # bg = cv2.bitwise_and(background, background, mask=mask)
        # result = cv2.add(fg, bg)
    
        if contours:
            largest_contour = max(contours, key=cv.contourArea)
            box = np.zeros_like(mask_inv)
            cv.drawContours(box, [largest_contour], -1, (255,255,255), thickness=cv.FILLED)
            x,y,w,h = cv.boundingRect(box)
            final=frame.copy()
            final=cv.rectangle(final,(x,y),(x+w,y+h),(0,255,0),2)
            fh=round(0.84*h)
            fw=round(0.84*w)
            print(f"Wysokosc = {h}px Szerokosc = {w}px")
            print(f"Wysokosc = {fh} mm Szerokosc = {fw} mm")
            cv.imshow("Box",box)
            cv.imshow("Final",final)
            

    else:

        gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray,(5,5),0)
        ret,thresh=cv.threshold(blur,160,255,cv.THRESH_BINARY)
        contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.imshow("Threshold",thresh)   
        if contours:
            largest_contour = max(contours, key=cv.contourArea)
            box = np.zeros_like(thresh)
            cv.drawContours(box, [largest_contour], -1, (255,255,255), thickness=cv.FILLED)
            x,y,w,h = cv.boundingRect(box)
            final=frame.copy()
            final=cv.rectangle(final,(x,y),(x+w,y+h),(0,255,0),2)
            fh=round(0.84*h)
            fw=round(0.84*w)
            print(f"Wysokosc = {h}px Szerokosc = {w}px")
            print(f"Wysokosc = {fh} mm Szerokosc = {fw} mm")
            cv.imshow("Box",box)
            cv.imshow("Final",final)
         
    
    time.sleep(0.25)
    c = cv.waitKey(16)
    if c == 27:
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


