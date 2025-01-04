import cv2 as cv
import logging
import os

from BoxMeasure import BoxMeasure

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


B_L = 7
B_R = 8

TRIG = 17
ECHO = 27	

CAMERA_NUM = 0

def main():
	try:
		bm = BoxMeasure(camera_num=CAMERA_NUM,trig=TRIG, echo=ECHO, b_l=B_L, b_r=B_R)

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
		log_error("KeyboardInterrupt")
	except Exception("Can't receive frame"):
		print("Nie można wczytać klatki. Zakończam program")
		log_error("Can't receive frame")
	except Exception("Cannot open camera"):
		print("Nie można otworzyć kamery")
		log_error("Cannot open camera")
	finally:
		del bm
		cv.destroyAllWindows()

if __name__ == "__main__":
	setup_logger("logs/error_log.txt")
	main()