from baro import BME280
import numpy as np
from pymavlink import mavutil
import threading
import time
import logging
import os
from datetime import datetime

VERSION = '0.3.0'
INFO = logging.INFO
DEBUG = logging.DEBUG

class BaroFix:
	BARO_INIT_PULL = 200
	BARO_STD_DEV = 1.2
	BARO_READ_DELAY = 0.04
	
	__slots__ = (
		"master",
		"logger_telem",
		"logger_error",
		"logger_debug",
		"logger_debug_make",
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
		self.logger_debug_make = True
		self.log_init()
		self.baro_init()
		#self.fc_init()
		self.thread_init()
		
	def get_time_us(self):
		return int(time.time()*1e6)
	
	def meadle_outliers(self, data):
		mean = np.mean(data)
		std = np.std(data)	

		threshold = self.BARO_STD_DEV * std
		filtered = [x for x in data if abs(x - mean) <= threshold]
		return round(np.mean(filtered), 2)
		
	def log_init(self):
		datetime_now = datetime.now()
		datetime_str = datetime_now.strftime("%Y%m%d_%H%M%S")		
		folder_log = 'log/' + datetime_str
		
		os.mkdir(folder_log)
		# ===========================================================
		self.logger_telem = logging.getLogger('Telemetry')
		self.logger_telem.setLevel(logging.INFO)
		handler_telem = logging.FileHandler(folder_log + '/' + "tel_" + datetime_str + ".log")
		handler_telem.setFormatter(logging.Formatter('%(message)s'))
		self.logger_telem.addHandler(handler_telem)
		# ===========================================================
		self.logger_error = logging.getLogger('Error')
		self.logger_error.setLevel(logging.INFO)
		handler_error = logging.FileHandler(folder_log + '/' + "error_" + datetime_str + ".log")
		handler_error.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
		self.logger_error.addHandler(handler_error)
		# ===========================================================
		self.logger_debug = logging.getLogger('Debug')
		self.logger_debug.setLevel(logging.DEBUG)
		handler_debug = logging.FileHandler(folder_log + '/' + "debug_" + datetime_str + ".log")
		handler_debug.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
		self.logger_debug.addHandler(handler_debug)
		# ===========================================================
		self.log_telemetry_init()

	def log_error_message(self, message):
		print(f'Error: {message}')
		self.logger_error.info(message)
		
	def log_debug_message(self, message, level=logging.DEBUG):
		if self.logger_debug_make:
			mes = f"{self.get_time_us()} {message}"
			if level == logging.INFO:
				print(f'{datetime.now()} {mes}')
				self.logger_debug.info(mes)
			elif level == logging.DEBUG:
				self.logger_debug.debug(mes)
			else:
				print(f'{datetime.now()} {mes}')
				self.logger_debug.info(mes)

	def log_telemetry_message(self, message):
		self.logger_telem.info(message)

	def log_telemetry_init(self):
		self.log_telemetry_message("FMT, 1, 99, BAR1, Qffff, Pressure, Altitude, Frequency, Frequency values")
		self.log_telemetry_message("FMT, 2, 99, BAR2, Qffff, Pressure, Altitude, Frequency, Frequency values")
		
	def log_telemetry_baro1(self, press, alt, freq, freq_values):
		self.log_telemetry_message(f"BAR1, {self.get_time_us()}, {press}, {alt}, {freq}, {freq_values}")

	def log_telemetry_baro2(self, press, alt, freq, freq_values):
		self.log_telemetry_message(f"BAR2, {self.get_time_us()}, {press}, {alt}, {freq}, {freq_values}")
		
	def baro_init(self):
		self.log_debug_message("Baro initialization start.",level=INFO)
		self.baro1 = BME280(0x76)
		self.baro2 = BME280(0x77)
		self.log_debug_message("Baro initialization finish.",level=INFO)
		
		baro1_array = []
		baro2_array = []

		self.log_debug_message("Ground pressure calculation start.",level=INFO)

		for i in range(self.BARO_INIT_PULL):
			baro1_array.append(round(self.baro1.read()[1], 2))
			baro2_array.append(round(self.baro2.read()[1], 2))
			
		self.baro1_press_gnd = self.meadle_outliers(baro1_array)
		self.baro2_press_gnd = self.meadle_outliers(baro2_array)
		self.baro1_press = self.baro1_press_gnd
		self.baro2_press = self.baro2_press_gnd

		self.log_debug_message("Ground pressure calculation finish.",level=INFO)

	def thread_init(self):
		self.log_debug_message("Thread initialization start.",level=INFO)
		threading.Thread(target=self.thread_baro).start()
		self.fc_init()
		threading.Thread(target=self.thread_fc).start()
		self.log_debug_message("Thread initialization finish.",level=INFO)

	def thread_baro(self):
		baro1_array = []
		baro2_array = []
		last_read = time.time()

		freq_ps_block = 1/self.BARO_READ_DELAY

		while True:
			try:
				baro1_array.append(round(self.baro1.read()[1], 2))
				baro2_array.append(round(self.baro2.read()[1], 2))
			except Exception as e:
				self.log_error_message(f"Error baro reading. {e}")
			
			if time.time() - last_read >= self.BARO_READ_DELAY:
				try:
					baro1_freq_size = len(baro1_array)
					baro2_freq_size = len(baro2_array)
					baro1_freq_ps = freq_ps_block*baro1_freq_size
					baro2_freq_ps = freq_ps_block*baro2_freq_size
					#=========================================
					baro1_array.append(self.baro1_press)
					baro1_array.append(self.baro1_press)
					self.baro1_press = self.meadle_outliers(baro1_array)
					self.baro1_alt = round(44330 * (1.0 - (self.baro1_press / self.baro1_press_gnd) ** (1/5.255)), 2)
					self.log_telemetry_baro1(self.baro1_press, self.baro1_alt, baro1_freq_ps, baro1_freq_size)
					#=========================================
					baro2_array.append(self.baro2_press)
					baro2_array.append(self.baro2_press)
					self.baro2_press = self.meadle_outliers(baro2_array)
					self.baro2_alt = round(44330 * (1.0 - (self.baro2_press / self.baro2_press_gnd) ** (1/5.255)), 2)
					self.log_telemetry_baro2(self.baro2_press, self.baro2_alt, baro2_freq_ps, baro2_freq_size)
					#=========================================
				except Exception as e:
					self.log_error_message(f"Error baro calculation. {e}")
					
				baro1_array = []
				baro2_array = []
				last_read = time.time()
				#=========================================
			time.sleep(0.0002)
		
		
	def thread_fc(self):
		self.log_debug_message("FC comunication started.",level=INFO)
		
		while True:
			print(1)
			#msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.02)
			msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=0.02)
			 
			print(msg)
			time.sleep(0.001)
		pass
		
		
	def fc_init(self):
		self.log_debug_message("FC connection.",level=INFO)
		connect_string = '/dev/ttyS0'
		self.master = mavutil.mavlink_connection(connect_string, baud = 115200, source_system=1, source_component=145)
		self.log_debug_message("FC initialization start.",level=INFO)
		self.master.wait_heartbeat()
		self.log_debug_message("FC initialization finish.",level=INFO)
		
	
		
fix = BaroFix()
#print(fix.baro1_press_gnd)
#print(fix.baro2_press_gnd)

