from baro import BME280
import numpy as np
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import threading
import time
import logging
import os
from datetime import datetime
import RPi.GPIO as GPIO
import math

VERSION = '0.6.1'
INFO = logging.INFO
DEBUG = logging.DEBUG

class BaroFix:
	BARO_INIT_PULL = 200
	BARO_STD_DEV = 1.2
	BARO_READ_DELAY = 0.04
	#RUN_PAUSE_S = 20
	RUN_PAUSE_S = 1
	
	__slots__ = (
		"time_booting_s",
		"master",
		"mode",
		"logger_telem",
		"logger_error",
		"logger_debug",
		"logger_debug_make",
		"baro0_press",
		"baro1",
		"baro1_press",
		"baro1_press_gnd",
		"baro1_alt",
		"baro2",
		"baro2_press",
		"baro2_press_gnd",
		"baro2_alt",
		"gp_vx",
		"gp_vy",
		"gp_vz",
		"gp_vg",
		"gp_alt",
		"gp_ralt",
		"rc_ch1",
		"rc_ch2",
		"rc_ch3",
		"rc_ch4",
		"rc_ch5",
		"rc_ch6",
		"rc_ch7",
		"rc_ch8",
		"rc_ch9",
		"rc_ch10",
		"rc_ch11",
		"rc_ch12",
		"rf_alt",
	)
	
	def led_init(self):
		try:
			GPIO.setmode(GPIO.BCM)

			GPIO.setup(16,GPIO.OUT) #led R
			GPIO.setup(20,GPIO.OUT) #led G
			GPIO.setup(21,GPIO.OUT) #led B

			#GPIO.output(16, GPIO.LOW) #R
			#GPIO.output(20, GPIO.LOW) #G
			#GPIO.output(21, GPIO.LOW) #B

			GPIO.output(16, GPIO.HIGH) #R
			time.sleep(self.RUN_PAUSE_S)
			GPIO.output(16, GPIO.LOW) #R
			GPIO.output(21, GPIO.HIGH) #B
		except Exception as e:
			self.log_error_message(f"Error led initialization.")

	def led_set_active(self):
		try:
			GPIO.output(21, GPIO.LOW) #B
			GPIO.output(20, GPIO.HIGH) #G
		except Exception as e:
			self.log_error_message(f"Error led set active.")
	
	def __init__(self):
		self.logger_debug_make = True
		self.log_init()
		
		self.led_init()
		
		self.baro_init()
		#self.fc_init()
		self.thread_init()
		self.mode = 0
		self.time_booting_s = time.time()
		self.led_set_active()

	def get_time_boot_ms(self):
		return int((time.time()-self.time_booting_s)*1e3)
		
	def get_time_boot_us(self):
		return int((time.time()-self.time_booting_s)*1e6)
		
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
		self.log_telemetry_message("FMT, 1, 99, BAR0, Qf, TimeUS, Pressure")
		self.log_telemetry_message("FMT, 2, 99, BAR1, Qffff, TimeUS, Pressure, Altitude, Frequency, Frequency values")
		self.log_telemetry_message("FMT, 3, 99, BAR2, Qffff, TimeUS, Pressure, Altitude, Frequency, Frequency values")
		self.log_telemetry_message("FMT, 4, 99, GNSS, Qffff, TimeUS")
		self.log_telemetry_message("FMT, 5, 99, GP, Qffff, TimeUS, Velocity x, Velocity y, Velocity z, Velocity ground, Altitude, Altutude relative")
		self.log_telemetry_message("FMT, 6, 99, RC, Qffff, TimeUS, Ch 1, Ch 2, Ch 3, Ch 4, Ch 5, Ch 6, Ch 7, Ch 8, Ch 9, Ch 10, Ch 11, Ch 12")
		self.log_telemetry_message("FMT, 7, 99, RF, Qffff, TimeUS, Altitude, Altitude origin, Altitude delta, Speed")

	def log_telemetry_baro0(self, press):
		self.log_telemetry_message(f"BAR0, {self.get_time_us()}, {press}")
		
	def log_telemetry_baro1(self, press, alt, freq, freq_values):
		self.log_telemetry_message(f"BAR1, {self.get_time_us()}, {press}, {alt}, {freq}, {freq_values}")

	def log_telemetry_baro2(self, press, alt, freq, freq_values):
		self.log_telemetry_message(f"BAR2, {self.get_time_us()}, {press}, {alt}, {freq}, {freq_values}")
		
	def log_telemetry_gp(self, vx, vy, vz, vg, alt, r_alt):
		self.log_telemetry_message(f"GP, {self.get_time_us()}, {vx}, {vy}, {vz}, {vg}, {alt}, {r_alt}")

	def log_telemetry_rc(self, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12):
		self.log_telemetry_message(f"RC, {self.get_time_us()}, {ch1}, {ch2}, {ch3}, {ch4}, {ch5}, {ch6}, {ch7}, {ch8}, {ch9}, {ch10}, {ch11}, {ch12}")

	def log_telemetry_rf(self, alt, alt_or, alt_delta, speed):
		self.log_telemetry_message(f"RF, {self.get_time_us()}, {alt/100}, {alt_or/100}, {alt_delta/100}, {speed}")

		
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
					#print(self.baro1_press, self.baro2_press)
				except Exception as e:
					self.log_error_message(f"Error baro calculation. {e}")
					
				baro1_array = []
				baro2_array = []
				last_read = time.time()
				#=========================================
			time.sleep(0.0002)
		
		
	def thread_fc(self):
		self.log_debug_message("FC comunication started.",level=INFO)
		#Data initialization
		self.gp_vx = 0
		self.gp_vy = 0
		self.gp_vz = 0
		self.gp_vg = 0
		#==========
		#correction_matrix = [0,0,0,0,0,70,130,200,350,550,700,850,1000]
		correction_matrix = [0, 0, 0, 0, 0, 7, 13, 20, 100, 100, 100, 100, 100]
		#==========
		self.rf_alt = int(self.baro2_alt*100)
		
		
		
		types_list = ['ATTITUDE', 'RC_CHANNELS', 'GLOBAL_POSITION_INT', 'SCALED_PRESSURE', 'HIGHRES_IMU']
		
		while True:
			baro_delta = 0
			try:
				msg = self.master.recv_match(type=types_list, blocking=True, timeout=0.02)
				if msg is not None:
					#print(msg._mavlink_version)
					#print(msg)
					msg_type = msg.get_type() 
					if msg_type == 'ATTITUDE':
						if msg.pitch > 0.785:
							pitch = 0.785
						elif msg.pitch < -0.785:
							pitch = -0.785
						
						if msg.roll > 0.785:
							roll = 0.785
						elif msg.roll > -0.785:
							roll = -0.785
						alt_compensate = self.baro2_alt / (math.cos(pitch) * math.cos(roll))
						
						try:
							cur_alt = int(alt_compensate*100)

							self.master.mav.heartbeat_send(0, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
							if cur_alt <= 0:
								self.rf_alt = 1
							else:
								if self.mode == 0:
									self.rf_alt = cur_alt
								elif self.mode == 1:
									last_fix = 0
									cur_value = 0
									
									for cur_fix in correction_matrix:
										if self.gp_vg < cur_value:
											break
											
										last_fix = cur_fix
										cur_value += 1
									
									baro_delta = last_fix+(1-cur_value-self.gp_vg)*(cur_fix-last_fix)
									
									self.rf_alt = int(cur_alt - baro_delta)
								elif self.mode == 2:
									self.rf_alt = self.rf_alt

							self.log_telemetry_rf(self.rf_alt, cur_alt, baro_delta, self.gp_vg)
							#---------------
							
							sign = mavlink2.MAVLink_distance_sensor_message(
								time_boot_ms = self.get_time_boot_ms(),
								min_distance = 1,
								max_distance = 40000,
								current_distance = self.rf_alt,
								type = 3,
								id = 0,
								orientation = 25,#orientation = 100,#
								covariance = 0,
								horizontal_fov = 3,
								vertical_fov = 3,
								quaternion = [1, 0, 0, 0],
								signal_quality = 100
							)
							self.master.mav.send(sign)
							
							'''
							self.master.mav.distance_sensor_send(
								time_boot_ms=self.get_time_boot_ms(),
								min_distance=1,
								max_distance=40000,
								current_distance=self.rf_alt,
								type=3,#type=4,
								id=0,
								orientation=25,
								covariance=0
							)
							'''
							#---------------
							#---------------
						except Exception as e:
							self.log_error_message(f"Error on send FC data: {e}")						
						
					elif msg_type == 'RC_CHANNELS':
						ch1 = msg.chan1_raw
						ch2 = msg.chan2_raw
						ch3 = msg.chan3_raw
						ch4 = msg.chan4_raw
						ch5 = msg.chan5_raw
						ch6 = msg.chan6_raw
						ch7 = msg.chan7_raw
						ch8 = msg.chan8_raw
						ch9 = msg.chan9_raw
						ch10 = msg.chan10_raw
						ch11 = msg.chan11_raw
						ch12 = msg.chan12_raw
						
						if ch9 < 1350:
							self.mode = 0
						elif ch9 >= 1350 and ch9 < 1750:
							self.mode = 1
						elif ch9 >= 1750:
							self.mode = 2
						self.log_telemetry_rc(ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12)
						
						self.rc_ch1 = ch1
						self.rc_ch2 = ch2
						self.rc_ch3 = ch3
						self.rc_ch4 = ch4
						self.rc_ch5 = ch5
						self.rc_ch6 = ch6
						self.rc_ch7 = ch7
						self.rc_ch8 = ch8
						self.rc_ch9 = ch9
						self.rc_ch10 = ch10
						self.rc_ch11 = ch11
						self.rc_ch12 = ch12
						
						#print(ch9, self.mode)
					elif msg_type == "GLOBAL_POSITION_INT":
						self.gp_vx = msg.vx/100
						self.gp_vy = msg.vy/100
						self.gp_vz = msg.vz/100
						self.gp_vg = abs(self.gp_vx) + abs(self.gp_vy)
						self.gp_alt = msg.alt
						self.gp_ralt = msg.relative_alt
						self.log_telemetry_gp(self.gp_vx, self.gp_vy, self.gp_vz, self.gp_vg, self.gp_alt, self.gp_ralt)
					elif msg_type == "SCALED_PRESSURE":
						self.baro0_press = msg.press_abs
						self.log_telemetry_baro0(self.baro0_press)
					#elif msg_type == "HIGHRES_IMU":
					#	print(msg)
						
			except Exception as e:
				self.log_error_message(f"Error on FC data read: {e}")
			#=======================================================================================	

			#====================================================================
			time.sleep(0.001)
		pass
		
		
	def fc_init(self):
		self.log_debug_message("FC connection.",level=INFO)
		connect_string = '/dev/ttyS0'
		#self.master = mavutil.mavlink_connection(connect_string, baud = 115200, source_system=1, source_component=145)
		self.master = mavutil.mavlink_connection(connect_string, baud = 115200, source_system=1, source_component=145)
		self.log_debug_message("FC initialization start.",level=INFO)
		self.master.wait_heartbeat()
		
		master.mav.request_data_stream_send(
			target_system=master.target_system,
			target_component=master.target_component,
			req_stream_id=mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # Stream with ATTITUDE
			req_message_rate=20,   # Frequency in Hz
			start_stop=1           # 1 = start sending, 0 = stop
		)
		
		self.master.mav.request_data_stream_send(
			self.master.target_system,
			self.master.target_component,
			mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
			2,     # 10 Hz
			1       # start (0 = stop)
		)
		
		self.master.mav.request_data_stream_send(
			self.master.target_system,
			self.master.target_component,
			mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
			2,  # Hz
			1   # start streaming
		)		
		
		self.master.mav.request_data_stream_send(
			self.master.target_system,
			self.master.target_component,
			mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # stream_id
			2,  # rate in Hz
			1   # start streaming (0 = stop)
		)		
		
		self.master.mav.request_data_stream_send(
			self.master.target_system,
			self.master.target_component,
			mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  # stream_id = 1
			5,  # Rate in Hz
			1    # Start streaming
		)		
		
		self.master.wait_heartbeat()
		self.log_debug_message("FC initialization finish.",level=INFO)
		
	
		
fix = BaroFix()
#print(fix.baro1_press_gnd)
#print(fix.baro2_press_gnd)

