import RPi.GPIO  as GPIO
import time

GPIO.setmode(GPIO.BCM)
TRIG = 17 
ECHO = 27
 # Set up the TRIG and ECHO pins 
GPIO.setup(TRIG, GPIO.OUT) 
GPIO.setup(ECHO, GPIO.IN)
#GPIO.cleanup()
def  test():
	while True:
		print(GPIO.input(ECHO))

def distance(): 
# Ensure the TRIG pin is set low for a short period before sending the pulse 
	GPIO.output(TRIG, False) 
	time.sleep(2) # Send the pulse 
	print("Sending pulse")
	GPIO.output(TRIG, True) 
	time.sleep(0.00001)
	GPIO.output(TRIG, False) # Wait for the ECHO pin to go high and record the start time 
	
	while GPIO.input(ECHO) == 0: 
		pulse_start = time.time() # Wait for the ECHO pin to go low and record the end time 
	while GPIO.input(ECHO) == 1: 
		pulse_end = time.time() # Calculate the distance based on the time difference 
	pulse_duration = pulse_end - pulse_start 
	distance = pulse_duration * 17150 
	distance = round(distance, 2) 
	return distance 
try: 

	while True: 
		dist = distance()
		print(f"Distance: {dist} cm") 
		time.sleep(1)
		
 
except KeyboardInterrupt: 
	print(6)
	print("Measurement stopped by User") 
	GPIO.cleanup()

