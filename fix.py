from baro import BME280
import numpy as np
from pymavlink import mavutil
import threading

VERSION = '0.0.5'

class BaroFix:
	BARO_INIT_PULL = 200
	BARO_STD_DEV = 1.2
	
	__slots__ = (
		"baro1",
		"baro2",
		"baro1_press_gnd",
		"baro2_press_gnd"
	)
	
	def __init__(self):
		self.baro_init()
		
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
		#print("baro1 init:", )
		#print("baro2 init:", meadle_outliers(baro2_array))

	def thread_baro(self):
		pass
		
		
		
fix = BaroFix()
print(fix.baro1_press_gnd)
print(fix.baro2_press_gnd)

