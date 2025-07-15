from baro import BME280
import numpy as np
from pymavlink import mavutil
import threading
import time

VERSION = '0.0.7'

class BaroFix:
	BARO_INIT_PULL = 200
	BARO_STD_DEV = 1.2
	BARO_READ_DELAY = 0.04
	
	__slots__ = (
		"baro1",
		"baro2",
		"baro1_press",
		"baro2_press",
		"baro1_press_gnd",
		"baro2_press_gnd",
		"baro1_alt",
		"baro2_alt"
	)
	
	def __init__(self):
		self.baro_init()
		self.thread_init()
		
	def meadle_outliers(self, data):
		mean = np.mean(data)
		std = np.std(data)	

		threshold = self.BARO_STD_DEV * std
		filtered = [x for x in data if abs(x - mean) <= threshold]
		return round(np.mean(filtered), 2)
		
	def baro_init(self):
		self.baro1 = BME280(0x76)
		self.baro2 = BME280(0x77)
		
		baro1_array = []
		baro2_array = []

		for i in range(self.BARO_INIT_PULL):
			baro1_array.append(round(self.baro1.read()[1], 2))
			baro2_array.append(round(self.baro2.read()[1], 2))
			
		self.baro1_press_gnd = self.meadle_outliers(baro1_array)
		self.baro2_press_gnd = self.meadle_outliers(baro2_array)
		self.baro1_press = self.baro1_press_gnd
		self.baro2_press = self.baro2_press_gnd

	def thread_init(self):
		threading.Thread(target=self.thread_baro).start()

	def thread_baro(self):
		baro1_array = []
		baro2_array = []
		last_read = time.time()

		while True:
			baro1_array.append(round(self.baro1.read()[1], 2))
			baro2_array.append(round(self.baro2.read()[1], 2))
			
			if time.time() - last_read >= self.BARO_READ_DELAY:
				baro1_freq_size = len(baro1_array)
				baro2_freq_size = len(baro2_array)
				
				baro1_array.append(self.baro1_press)
				baro1_array.append(self.baro1_press)
				self.baro1_press = self.meadle_outliers(baro1_array)
				self.baro1_alt = round(44330 * (1.0 - (self.baro1_press / self.baro1_press_gnd) ** (1/5.255)), 2)
				
				baro2_array.append(self.baro2_press)
				baro2_array.append(self.baro2_press)
				self.baro2_press = self.meadle_outliers(baro2_array)
				self.baro2_alt = round(44330 * (1.0 - (self.baro2_press / self.baro2_press_gnd) ** (1/5.255)), 2)

				#print("baro", baro1_freq_size, baro2_freq_size, self.baro1_press, self.baro2_press, self.baro1_alt, self.baro2_alt)

				baro1_array = []
				baro2_array = []
				last_read = time.time()
				
			time.sleep(0.0002)
		
		
		
fix = BaroFix()
#print(fix.baro1_press_gnd)
#print(fix.baro2_press_gnd)

