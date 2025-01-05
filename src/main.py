import cv2 as cv
import logging
import sys
from BoxMeasure import BoxMeasure
from logger_config import setup_logger

B_L = 7
B_R = 8

TRIG = 17
ECHO = 27	

CAMERA_NUM = 0

def main():
	logging.debug("Start programu")
	try:
		bm = BoxMeasure(camera_num=CAMERA_NUM,trig=TRIG, echo=ECHO, b_l=B_L, b_r=B_R)
	except KeyboardInterrupt:
		logging.info("Wyłączam program")
		sys.exit(1)

	try:
		while True:
			bm.new_or_rep()
			bm.disp_res()
			"""
			c = cv.waitKey(100)
			if c == 27:
				raise KeyboardInterrupt
			elif c == 99: # 99 = ord('c')
				borderType = cv.BORDER_CONSTANT
			elif c == 114: # 114 = ord('r')
				borderType = cv.BORDER_REPLICATE
			"""
	except Exception as e:
		logging.error(f"{e}")
	finally:
		del bm
		cv.destroyAllWindows()
		logging.info("Wyłączam program")

if __name__ == "__main__":
	setup_logger("logs/app_log.txt")
	main()