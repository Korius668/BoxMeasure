import RPi.GPIO as GPIO
import time

B_1 = 7
B_2 = 8

def check_button(button):
	return GPIO.input(button) == GPIO.LOW

def check_buttons():
	c_1 = check_button(B_1)
	c_2 = check_button(B_2)
	time.sleep(0.1)

	return [c_1 == True and check_button(B_1) == False, c_2 == True and check_button(B_2) == False]

GPIO.setmode(GPIO.BCM)
GPIO.setup(B_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
print(f"Konfiguracja GPIO dla {B_1}: {GPIO.gpio_function(B_1)}")
GPIO.setup(B_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print(f"Konfiguracja GPIO dla {B_2}: {GPIO.gpio_function(B_2)}")


try:
	# Główna pętla programu
	while True:
		print(check_buttons())
except KeyboardInterrupt:
	print("Program został zakończony.")
finally:
	# Czyszczenie ustawień GPIO
	GPIO.cleanup()
	print("GPIO zostało wyczyszczone.")
