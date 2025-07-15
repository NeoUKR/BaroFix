import smbus
import time
from ctypes import c_short, c_byte, c_ubyte
# from ctypes import c_byte
# from ctypes import c_ubyte
import numpy as np
import statistics


class BME280:
    __slots__ = (
        "addr",
        "bus_id",
        "bus"
    )

    def __init__(self, addr=None, bus_id=None):
        if addr is None:
            self.addr = 0x76
        else:
            self.addr = addr

        if bus_id is None:
            self.bus_id = 1
        else:
            self.bus_id = bus_id

        self.bus = smbus.SMBus(self.bus_id)

    @staticmethod
    def getShort(data, index):
        # return two bytes from data as a signed 16-bit value
        return c_short((data[index + 1] << 8) + data[index]).value

    @staticmethod
    def getUShort(data, index):
        # return two bytes from data as an unsigned 16-bit value
        return (data[index + 1] << 8) + data[index]

    @staticmethod
    def getChar(data, index):
        # return one byte from data as a signed char
        result = data[index]
        if result > 127:
            result -= 256
        return result

    @staticmethod
    def getUChar(data, index):
        # return one byte from data as an unsigned char
        result = data[index] & 0xFF
        return result

    def read_id(self):
        # Chip ID Register Address
        REG_ID = 0xD0
        (chip_id, chip_version) = self.bus.read_i2c_block_data(self.addr, REG_ID, 2)
        return (chip_id, chip_version)

    def read(self):
        # Register Addresses
        REG_DATA = 0xF7
        REG_CONTROL = 0xF4
        REG_CONFIG = 0xF5
        REG_CONTROL_HUM = 0xF2
        REG_HUM_MSB = 0xFD
        REG_HUM_LSB = 0xFE
        # Oversample setting - page 27
        OVERSAMPLE_TEMP = 2
        OVERSAMPLE_PRES = 2
        MODE = 1
        # Oversample setting for humidity register - page 26
        OVERSAMPLE_HUM = 2
        self.bus.write_byte_data(self.addr, REG_CONTROL_HUM, OVERSAMPLE_HUM)
        control = OVERSAMPLE_TEMP << 5 | OVERSAMPLE_PRES << 2 | MODE
        self.bus.write_byte_data(self.addr, REG_CONTROL, control)
        # Read blocks of calibration data from EEPROM
        # See Page 22 data sheet
        cal1 = self.bus.read_i2c_block_data(self.addr, 0x88, 24)
        cal2 = self.bus.read_i2c_block_data(self.addr, 0xA1, 1)
        cal3 = self.bus.read_i2c_block_data(self.addr, 0xE1, 7)
        # Convert byte data to word values
        dig_T1 = self.getUShort(cal1, 0)
        dig_T2 = self.getShort(cal1, 2)
        dig_T3 = self.getShort(cal1, 4)
        dig_P1 = self.getUShort(cal1, 6)
        dig_P2 = self.getShort(cal1, 8)
        dig_P3 = self.getShort(cal1, 10)
        dig_P4 = self.getShort(cal1, 12)
        dig_P5 = self.getShort(cal1, 14)
        dig_P6 = self.getShort(cal1, 16)
        dig_P7 = self.getShort(cal1, 18)
        dig_P8 = self.getShort(cal1, 20)
        dig_P9 = self.getShort(cal1, 22)
        dig_H1 = self.getUChar(cal2, 0)
        dig_H2 = self.getShort(cal3, 0)
        dig_H3 = self.getUChar(cal3, 2)
        dig_H4 = self.getChar(cal3, 3)
        dig_H4 = (dig_H4 << 24) >> 20
        dig_H4 = dig_H4 | (self.getChar(cal3, 4) & 0x0F)
        dig_H5 = self.getChar(cal3, 5)
        dig_H5 = (dig_H5 << 24) >> 20
        dig_H5 = dig_H5 | (self.getUChar(cal3, 4) >> 4 & 0x0F)
        dig_H6 = self.getChar(cal3, 6)
        # Wait in ms (Datasheet Appendix B: Measurement time and current calculation)
        wait_time = 1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM) + 0.575)
        # time.sleep(wait_time/1000)  # Wait the required time
        # Read temperature/pressure/humidity
        data = self.bus.read_i2c_block_data(self.addr, REG_DATA, 8)
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw = (data[6] << 8) | data[7]
        # Refine temperature
        var1 = ((((temp_raw >> 3) - (dig_T1 << 1))) * (dig_T2)) >> 11
        var2 = (((((temp_raw >> 4) - (dig_T1)) * ((temp_raw >> 4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
        t_fine = var1 + var2
        temperature = float(((t_fine * 5) + 128) >> 8);
        # Refine pressure and adjust for temperature
        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * dig_P6 / 32768.0
        var2 = var2 + var1 * dig_P5 * 2.0
        var2 = var2 / 4.0 + dig_P4 * 65536.0
        var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * dig_P1
        if var1 == 0:
            pressure = 0
        else:
            pressure = 1048576.0 - pres_raw
            pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
            var1 = dig_P9 * pressure * pressure / 2147483648.0
            var2 = pressure * dig_P8 / 32768.0
            pressure = pressure + (var1 + var2 + dig_P7) / 16.0
        # Refine humidity
        humidity = t_fine - 76800.0
        humidity = (hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
        humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
        if humidity > 100:
            humidity = 100
        elif humidity < 0:
            humidity = 0
        return temperature / 100.0, pressure / 100.0, humidity


'''
baro1 = BME280(0x77)
baro2 = BME280(0x76)
#print(baro1.read_id())


baro1_start = baro1.read()[1]
baro2_start = baro2.read()[1]
delta = baro1_start - baro2_start
print("initial", baro1_start, baro2_start, "delta", delta)

stek1 = []
stek2 = []

first_stek = True

start_freq = time.time()
num = 0
while True:
  cur_time = time.time()
  if (cur_time-start_freq) >= 0.1:
    start_freq = cur_time
    if first_stek:
      press = statistics.mean(stek1)
      num = len(stek1)
      stek1 = []
    else:
      press = statistics.mean(stek2)
      num = len(stek2)
      stek2 = []

    first_stek = not first_stek
    print(time.time(), press, num, stek1)


  if first_stek:
    stek1.append(baro1.read()[1])
  else:
    stek2.append(baro1.read()[1])

  #baro1_p = baro1.read()[1]
  #baro2_p = baro2.read()[1]+delta
  #baro2_p = baro1_p
  #print("b1", baro1_p, "b2", baro2_p)
  #dif_baro = round(baro1_p-baro2_p, 3)

  #h = 44_330 * (1-(baro1_p/baro2_p)**(1/5.255))
  #print(time.time(), dif_baro, "dif alt:", f'{h:.2f}', "b1", f'{baro1_p:.2f}','b2', f'{baro2_p:.2f}',"baro1", f'{baro1.read()[1]:.2f}', "baro2", f'{baro2.read()[1]:.2f}')
  time.sleep(0.001)
'''

'''
DEVICE = 0x76  #This address may have to change depending on your BME280 sensor.
bus = smbus.SMBus(1) # Version 2 Pi, Pi 2 & Pi 3 uses bus 1. Rev 1 Pi uses bus 0.
def getShort(data, index):
  # return two bytes from data as a signed 16-bit value
  return c_short((data[index+1] << 8) + data[index]).value
def getUShort(data, index):
  # return two bytes from data as an unsigned 16-bit value
  return (data[index+1] << 8) + data[index]
def getChar(data,index):
  # return one byte from data as a signed char
  result = data[index]
  if result > 127:
    result -= 256
  return result
def getUChar(data,index):
  # return one byte from data as an unsigned char
  result =  data[index] & 0xFF
  return result
def readBME280ID(addr=DEVICE):
  # Chip ID Register Address
  REG_ID     = 0xD0
  (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
  return (chip_id, chip_version)
def readBME280All(addr=DEVICE):
  # Register Addresses
  REG_DATA = 0xF7
  REG_CONTROL = 0xF4
  REG_CONFIG  = 0xF5
  REG_CONTROL_HUM = 0xF2
  REG_HUM_MSB = 0xFD
  REG_HUM_LSB = 0xFE
  # Oversample setting - page 27
  OVERSAMPLE_TEMP = 2
  OVERSAMPLE_PRES = 2
  MODE = 1
  # Oversample setting for humidity register - page 26
  OVERSAMPLE_HUM = 2
  bus.write_byte_data(addr, REG_CONTROL_HUM, OVERSAMPLE_HUM)
  control = OVERSAMPLE_TEMP<<5 | OVERSAMPLE_PRES<<2 | MODE
  bus.write_byte_data(addr, REG_CONTROL, control)
  # Read blocks of calibration data from EEPROM
  # See Page 22 data sheet
  cal1 = bus.read_i2c_block_data(addr, 0x88, 24)
  cal2 = bus.read_i2c_block_data(addr, 0xA1, 1)
  cal3 = bus.read_i2c_block_data(addr, 0xE1, 7)
  # Convert byte data to word values
  dig_T1 = getUShort(cal1, 0)
  dig_T2 = getShort(cal1, 2)
  dig_T3 = getShort(cal1, 4)
  dig_P1 = getUShort(cal1, 6)
  dig_P2 = getShort(cal1, 8)
  dig_P3 = getShort(cal1, 10)
  dig_P4 = getShort(cal1, 12)
  dig_P5 = getShort(cal1, 14)
  dig_P6 = getShort(cal1, 16)
  dig_P7 = getShort(cal1, 18)
  dig_P8 = getShort(cal1, 20)
  dig_P9 = getShort(cal1, 22)
  dig_H1 = getUChar(cal2, 0)
  dig_H2 = getShort(cal3, 0)
  dig_H3 = getUChar(cal3, 2)
  dig_H4 = getChar(cal3, 3)
  dig_H4 = (dig_H4 << 24) >> 20
  dig_H4 = dig_H4 | (getChar(cal3, 4) & 0x0F)
  dig_H5 = getChar(cal3, 5)
  dig_H5 = (dig_H5 << 24) >> 20
  dig_H5 = dig_H5 | (getUChar(cal3, 4) >> 4 & 0x0F)
  dig_H6 = getChar(cal3, 6)
  # Wait in ms (Datasheet Appendix B: Measurement time and current calculation)
  wait_time = 1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM)+0.575)
  time.sleep(wait_time/1000)  # Wait the required time
  # Read temperature/pressure/humidity
  data = bus.read_i2c_block_data(addr, REG_DATA, 8)
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
  hum_raw = (data[6] << 8) | data[7]
  #Refine temperature
  var1 = ((((temp_raw>>3)-(dig_T1<<1)))*(dig_T2)) >> 11
  var2 = (((((temp_raw>>4) - (dig_T1)) * ((temp_raw>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
  t_fine = var1+var2
  temperature = float(((t_fine * 5) + 128) >> 8);
  # Refine pressure and adjust for temperature
  var1 = t_fine / 2.0 - 64000.0
  var2 = var1 * var1 * dig_P6 / 32768.0
  var2 = var2 + var1 * dig_P5 * 2.0
  var2 = var2 / 4.0 + dig_P4 * 65536.0
  var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
  var1 = (1.0 + var1 / 32768.0) * dig_P1
  if var1 == 0:
    pressure=0
  else:
    pressure = 1048576.0 - pres_raw
    pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
    var1 = dig_P9 * pressure * pressure / 2147483648.0
    var2 = pressure * dig_P8 / 32768.0
    pressure = pressure + (var1 + var2 + dig_P7) / 16.0
  # Refine humidity
  humidity = t_fine - 76800.0
  humidity = (hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
  humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
  if humidity > 100:
    humidity = 100
  elif humidity < 0:
    humidity = 0
  return temperature/100.0,pressure/100.0,humidity

def main():
	(chip_id, chip_version) = readBME280ID()
	print("Chip ID     :", chip_id)
	print("Version     :", chip_version)

	aver_pressure = []
	for i in range(5):
		start_temperature, start_pressure, start_humidity = readBME280All()
		aver_pressure.append(start_pressure)

	start_pressure = np.mean(aver_pressure)
	#print(len(aver_pressure), numpy.mean(aver_pressure))
	print('start presure:', start_pressure)
	aver_alt = [0, 0, 0, 0, 0]

	while True:
		temperature, pressure, humidity = readBME280All()
		aver_pressure.pop(0)
		aver_pressure.append(pressure)
		cur_pressure = np.mean(aver_pressure)
		alt = 44_330 * (1 - (cur_pressure/start_pressure) ** (1 / 5.5255))
		aver_alt.pop(0)
		aver_alt.append(alt)
		cur_alt = np.mean(aver_alt)

		print(end='\r')
		print(f'{time.time():.5f}', "Alt: " , f'{cur_alt:.2f}:', "Pressure : ", cur_pressure, "hPa", "Temperature : ", temperature, "C", "Humidity : ", humidity, "%", end="")
		#time.sleep(0.001)

if __name__=="__main__":
	main()
'''
