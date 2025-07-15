from baro import BME280
import numpy as np
from pymavlink import mavutil

VERSION = '0.0.3'

baro1 = BME280(0x76)
baro2 = BME280(0x77)

print(baro1.read()[1])
print(baro2.read()[1])

BARO_INIT_PULL = 200
BARO_STD_DEV = 1.2

baro1_array = []
baro2_array = []

for i in range(BARO_INIT_PULL):
	baro1_array.append(round(baro1.read()[1], 2))
	baro2_array.append(round(baro2.read()[1], 2))
	
	
def meadle_outliers(data):
	mean = np.mean(data)
	std = np.std(data)	

	threshold = BARO_STD_DEV * std
	filtered = [x for x in data if abs(x - mean) <= threshold]
	return round(np.mean(filtered), 2)
	
	
print("baro1 init:", meadle_outliers(baro1_array))
print("baro2 init:", meadle_outliers(baro2_array))

