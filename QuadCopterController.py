import time
import random
import math
import msvcrt
import struct
import tkinter as tk
import importlib
from importlib.machinery import SourceFileLoader


xpc = SourceFileLoader('xpc', '../../Python3/src/xpc.py').load_module()

f_run 		= True
f_control 	= True
f_override	= True

elev		= -1
heading 	= -1

prev_Q	= 1
prev_P	= 1
prev_R	= 1

def negOne(val):
	if val < -1.0:
		return -1.0
	elif val > 1.0:
		return 1.0
	else:
		return val
		
def negOneMax(val, max):
	if val < max * -1:
		return (max * -1)
	elif val > max:
		return max
	else:
		return val

def zeroOne(val):
	if val < 0.0:
		return 0.0
	elif val > 1.0:
		return 1.0
	else:
		return val

def zeroOneMax(val, max):
	if val < 0.0:
		return 0.0
	elif val > max:
		return max
	else:
		return val

'''def normal(vals):
	newvals = list()

	maxval = vals[0]
	minval = vals[0]
	for v in vals:
		if v > maxval:
			maxval = v
		if v < minval:
			minval = v
	print(maxval, minval ,end='\r')
	for v in vals:
		if (maxval > 1):
			newvals.append(v - (maxval - 1))
		elif (minval < 0):
			newvals.append(v - minval)	# minval - negative
		else:
			newvals.append(v)
	
	return newvals
	pass'''

def normal(vals):
	maxval = vals[0]
	minval = vals[0]
	for v in vals:
		if v > maxval:
			maxval = v
		if v < minval:
			minval = v
	
	#print(maxval, minval ,end='\r')
	
	pp = -1 * minval + maxval
	mid = (maxval + minval) / 2		# minval - negative
	
	newvals = list()
	for v in vals:
		if (minval < 0 or maxval > 1):
			if (v > 0):
				newvals.append((v + -1*minval) / pp)
			else:
				newvals.append((v + -1*minval) / pp)
		else:
			newvals.append(v)
	
	return newvals
	pass

def mySendDREF(cl, dref, values):
	mySendDREFs(cl, [dref], [values])

def mySendDREFs(cl, drefs, values):
		if len(drefs) != len(values):
			raise ValueError("drefs and values must have the same number of elements.")

		buffer = struct.pack(b"<4sx", b"DREF")
		for i in range(len(drefs)):
			dref = drefs[i]
			value = values[i]

			# Preconditions
			if len(dref) == 0 or len(dref) > 255:
				raise ValueError("dref must be a non-empty string less than 256 characters.")

			if value is None:
				raise ValueError("value must be a scalar or sequence of floats.")

			# Pack message
			if hasattr(value, "__len__"):
				if len(value) > 255:
					raise ValueError("value must have less than 256 items.")
				fmt = "<B{0:d}sB{1:d}f".format(len(dref), len(value))
				buffer += struct.pack(fmt.encode(), len(dref), dref.encode(), len(value), *value)
			else:
				fmt = "<B{0:d}sBf".format(len(dref))
				buffer += struct.pack(fmt.encode(), len(dref), dref.encode(), 1, value)

		# Send
		cl.sendUDP(buffer)

def loop(cl, l):
	global f_control
	global f_override
	
	global heading
	global elev
	
	global prev_Q
	global prev_P
	global prev_R

	# sim vars
	drefs = [
	'sim/operation/override/override_joystick',\
	'sim/operation/override/override_joystick_pitch',\
	'sim/operation/override/override_joystick_roll',\
	'sim/operation/override/override_joystick_heading',\
	'sim/operation/override/override_throttles',\
	'sim/joystick/yoke_pitch_ratio',\
	'sim/joystick/yoke_roll_ratio',\
	'sim/joystick/yoke_heading_ratio',\
	'sim/flightmodel/controls/elv_trim',\
	'sim/flightmodel/controls/ail_trim',\
	'sim/flightmodel/controls/rud_trim',\
	'sim/flightmodel/engine/ENGN_thro_use',\
	'sim/flightmodel/controls/flaprqst',\
	'sim/flightmodel/controls/sbrkrqst',\
	'sim/aircraft/parts/acf_gear_deploy',\
	'sim/flightmodel/controls/parkbrake',\
	'sim/flightmodel/position/latitude',\
	'sim/flightmodel/position/longitude',\
	'sim/flightmodel/position/elevation',\
	'sim/flightmodel/position/theta',\
	'sim/flightmodel/position/phi',\
	'sim/flightmodel/position/psi',\
	'sim/flightmodel/position/alpha',\
	'sim/flightmodel/position/indicated_airspeed',\
	'sim/flightmodel/position/true_airspeed',\
	'sim/flightmodel/position/groundspeed',\
	'sim/flightmodel/controls/nosewheel_steer',\
	'sim/flightmodel/weight/m_total',\
	'sim/flightmodel/weight/m_fuel_total',\
	'sim/flightmodel/position/magnetic_variation',\
	'sim/flightmodel/position/P',\
	'sim/flightmodel/position/Q',\
	'sim/flightmodel/position/R',\
	'sim/flightmodel/position/vh_ind',\
	'sim/flightmodel/controls/vectrqst',\
	'sim/flightmodel/engine/ENGN_thro'
	]
	
	# "header"
	dref_override_joystick			= 0
	dref_override_joystick_pitch	= 1
	dref_override_joystick_roll		= 2
	dref_override_joystick_yaw		= 3
	dref_override_throttle			= 4
	dref_pitch						= 5
	dref_roll						= 6
	dref_yaw						= 7
	dref_pitch_trim					= 8
	dref_roll_trim					= 9
	dref_yaw_trim					= 10
	dref_throttle					= 11
	dref_flap						= 12
	dref_speedbrake					= 13
	dref_gear						= 14
	dref_brake						= 15
	dref_latitude					= 16
	dref_longitude					= 17
	dref_elevation					= 18
	dref_theta						= 19
	dref_phi						= 20
	dref_psi						= 21
	dref_alpha						= 22
	dref_indicated_airspeed			= 23
	dref_true_airspeed				= 24
	dref_groundspeed				= 25
	dref_nosewheel_steer			= 26
	dref_m_total					= 27
	dref_m_fuel_total				= 28
	dref_magnetic_variation			= 29
	dref_P							= 30
	dref_Q							= 31
	dref_R							= 32
	dref_vertical_speed				= 33
	dref_vectrqst					= 34
	dref_ENGN_thro					= 35


	'''
	beta
	vpath
	hpath
	
	(rotation, deg/s)
	P (roll)
	Q (pitch)
	R (yaw)
	
	(accel, deg/s^2)
	P_dot
	Q_dot
	R_dot
	
	(rad/s)
	Prad
	Qrad
	Rrad
	
	true_theta
	true_phi
	true_psi
	'''
	
	# get data from sim
	result = cl.getDREFs(drefs)
	
	# system
	override_joystick		= result[dref_override_joystick][0]
	override_joystick_pitch	= result[dref_override_joystick_pitch][0]
	override_joystick_roll	= result[dref_override_joystick_roll][0]
	override_joystick_yaw	= result[dref_override_joystick_yaw][0]
	override_throttle		= result[dref_override_throttle][0]
	
	# additional
	pitch_trim				= result[dref_pitch_trim][0]
	roll_trim				= result[dref_roll_trim][0]
	yaw_trim				= result[dref_yaw_trim][0]
	
	###########################################################################
	# main control
	pitch					= result[dref_pitch][0]
	roll					= result[dref_roll][0]
	yaw						= result[dref_yaw][0]
	throttle0				= result[dref_throttle][0]
	throttle1				= result[dref_throttle][1]
	throttle2				= result[dref_throttle][2]
	throttle3				= result[dref_throttle][3]
	
	# secondary control
	flap					= result[dref_flap][0]
	speedbrake				= result[dref_speedbrake][0]
	gear					= result[dref_gear][0]
	brake					= result[dref_brake][0]
	
	# aircraft "sensors" (INDependent)
	# gyro
	Q						= result[dref_Q][0]			# pitch rotation
	P						= result[dref_P][0]			# roll rotation
	R						= result[dref_R][0]			# yaw rotation
	# gyro + accel (IMU)
	theta					= result[dref_theta][0]		# pitch (angle to horizon)
	phi						= result[dref_phi][0]		# roll (angle to horizon)
	psi						= result[dref_psi][0]		# yaw (angle to ?..)
	# pito tube
	indicated_airspeed		= result[dref_indicated_airspeed][0]
	# baro
	elevation				= result[dref_elevation][0]
	vertical_speed			= result[dref_vertical_speed][0]
	
	# aircraft "sensors" (Dependent)
	# GPS
	latitude				= result[dref_latitude][0]
	longitude				= result[dref_longitude][0]
	groundspeed				= result[dref_groundspeed][0]
	# other
	magnetic_variation		= result[dref_magnetic_variation][0]
	###########################################################################
	
	# ?
	true_airspeed			= result[dref_true_airspeed][0]
	m_total					= result[dref_m_total][0]
	m_fuel_total			= result[dref_m_fuel_total][0]
	#nosewheel_steer			= result[dref_nosewheel_steer][0]
	vectrqst				= result[dref_vectrqst][0]
	ENGN_thro 				= result[dref_ENGN_thro][0]
	
	alpha					= result[dref_alpha][0]
	#beta
	
	if (elev == -1):
		elev = elevation
	
	if (heading == -1):
		heading = psi
	
	# show in window
	l[dref_override_joystick		].config(text=f'override_joystick: {override_joystick:5.1f}')
	l[dref_override_joystick_pitch	].config(text=f'override_joystick_pitch: {override_joystick_pitch:5.1f}')
	l[dref_override_joystick_roll	].config(text=f'override_joystick_roll: {override_joystick_roll:5.1f}')
	l[dref_override_joystick_yaw	].config(text=f'override_joystick_yaw: {override_joystick_yaw:5.1f}')
	l[dref_override_throttle		].config(text=f'override_throttle: {override_throttle:5.1f}')
	l[dref_pitch					].config(text=f'pitch: {pitch:8.4f}')
	l[dref_roll						].config(text=f'roll: {roll:8.4f}')
	l[dref_yaw						].config(text=f'yaw: {yaw:8.4f}')
	l[dref_pitch_trim				].config(text=f'pitch_trim: {pitch_trim:8.4f}')
	l[dref_roll_trim				].config(text=f'roll_trim: {roll_trim:8.4f}')
	l[dref_yaw_trim					].config(text=f'yaw_trim: {yaw_trim:8.4f}')
	l[dref_throttle					].config(text=f'fL:{throttle0:6.2f}; fR:{throttle1:6.2f}; rL:{throttle2:6.2f}; rR:{throttle3:6.2f}')
	l[dref_flap						].config(text=f'flap: {flap:6.2f}')
	l[dref_speedbrake				].config(text=f'speedbrake: {speedbrake:6.2f}')
	l[dref_gear						].config(text=f'gear: {gear:6.2f}')
	l[dref_brake					].config(text=f'brake: {brake:6.2f}')
	l[dref_latitude					].config(text=f'latitude: {latitude:8.4f}')
	l[dref_longitude				].config(text=f'longitude: {longitude:8.4f}')
	l[dref_elevation				].config(text=f'elevation: {elevation:6.2f}')
	l[dref_theta					].config(text=f'theta: {theta:6.2f}')
	l[dref_phi						].config(text=f'phi: {phi:6.2f}')
	l[dref_psi						].config(text=f'psi: {psi:6.2f}')
	l[dref_alpha					].config(text=f'alpha: {alpha:6.2f}')
	l[dref_indicated_airspeed		].config(text=f'indicated_airspeed: {indicated_airspeed:6.2f}')
	l[dref_true_airspeed			].config(text=f'true_airspeed: {true_airspeed:6.2f}')
	l[dref_groundspeed				].config(text=f'groundspeed: {groundspeed:6.2f}')
	#l[dref_nosewheel_steer			].config(text=f'nosewheel_steer: {nosewheel_steer:6.2f}')
	l[dref_m_total					].config(text=f'm_total: {m_total:6.2f}')
	l[dref_m_fuel_total				].config(text=f'm_fuel_total: {m_fuel_total:6.2f}')
	l[dref_magnetic_variation		].config(text=f'magnetic_variation: {magnetic_variation:6.2f}')
	l[dref_P						].config(text=f'P(roll): {P:8.4f}')
	l[dref_Q						].config(text=f'Q(pitch): {Q:8.4f}')
	l[dref_R						].config(text=f'R(yaw): {R:8.4f}')
	l[dref_vertical_speed			].config(text=f'vertical_speed: {vertical_speed:6.2f}')
	l[dref_vectrqst					].config(text=f'vectrqst: {vectrqst:6.2f}')
	l[dref_ENGN_thro				].config(text=f'ENGN_thro: {ENGN_thro:6.2f}')
	
	if f_override:
		cl.sendDREF(drefs[dref_override_joystick_pitch], False)
		cl.sendDREF(drefs[dref_override_joystick_roll], False)
		cl.sendDREF(drefs[dref_override_joystick_yaw], False)
		
		cl.sendDREF(drefs[dref_override_throttle], f_control)
		
		f_override = False
	
	if f_control:
		#hspeed 		= 80
		
		vspeed_lim	= 2
		pitch_lim	= 45
		roll_lim	= 45
		
		min_thr		= 0.01
		thr_mid		= 0.5
		
		thr_dz		= 0.3	# deadzone (thr_mid - thr_dz .. thr_mid + thr_dz, 0.5-0.3 .. 0.5+0.3 => 0.2 .. 0.8)
		yaw_dz		= 0.1
		
		vspeed_kP	= 0.2
		thr_kP		= 0.1
		
		thr_ctl_rate	= 10
		yaw_ctl_rate	= 10
		
		kP_p 	= 0.002
		kP_r 	= 0.002
		kP_y 	= 0.002
		
		rate_p 	= 0.01
		rate_r 	= rate_p
		rate_y 	= 0.2
		
		# нужно для диапазона "-180"..нужное_направление.."+180"
		# что бы расчитанное управляющее воздействие
		# было в диапазоне от -180 до +180 с 0 по середине.
		# изначальный диапазон направления (0..360 градусов)
		#
		# начинает делать круг на крайних значениях нужного_направления
		# потому что при уходе за 0 или 360 управляющее воздействие меняет направление...
		# что бы это избежать - делаем нужное_направление серединой значений
		# а отклонения как в минус так и в плюс, одинаковыми и = 180(+ и -) град.
		#
		# короче смещаем середину диапазона psi, делая серединой нужное_направление (heading).
		#
		# FIXME найти минимальное значение для выбора направления поворота. (поворот вправо на 45 град, делается через левый разворот...)
		hmin = heading - 180
		if psi < hmin:
			fix_psi = psi + 360
		else:
			fix_psi = psi
		
		if (yaw < (-1 * yaw_dz) or yaw > yaw_dz):
			heading = psi + (yaw * yaw_ctl_rate)
			
			if (heading > 360):
				heading = heading - 360
				
			if (heading < 0):
				heading = heading + 360
		
		if (ENGN_thro < (thr_mid - thr_dz) or ENGN_thro > (thr_mid + thr_dz)):
			elev = elevation + (ENGN_thro - thr_mid) * thr_ctl_rate
			
			if (elev < 0):
				elev = 0

		# отрицательная обратная связь - отрицательный коэффициент
		neg_feedback = -1
		
		vspeed		= negOne(			(elevation 		- elev) 	* neg_feedback * vspeed_kP) * vspeed_lim
		thr_rate 	= ENGN_thro #zeroOne(thr_mid + (vertical_speed - vspeed) 	* neg_feedback * thr_kP)
		
		#pitch_rate	= negOne(negOne(Q * neg_feedback * kP_p) + negOne((theta 	- pitch*pitch_lim) 	* neg_feedback 	* rate_p)) * thr_rate
		#roll_rate	= negOne(negOne(P * neg_feedback * kP_r) + negOne((phi 		- roll*roll_lim) 	* neg_feedback 	* rate_r)) * thr_rate
		#yaw_rate	= negOne(negOne(R * neg_feedback * kP_y) + negOne(((fix_psi - heading)  		* neg_feedback) * rate_y)) * thr_rate

		#max_rot_rate = 1 # deg/s
		rate = 400
		
		kD_p = (prev_Q - Q) / 60
		kD_r = (prev_P - P) / 60
		kD_y = (prev_R - R) / 60
		
		pitch_rate	= negOne( (Q - pitch*rate) * neg_feedback * kP_p )#+ kD_p) #* thr_rate
		roll_rate	= negOne( (P - roll *rate) * neg_feedback * kP_r )#+ kD_r) #* thr_rate
		yaw_rate	= negOne( (R - yaw  *rate) * neg_feedback * kP_y )#+ kD_y) #* thr_rate
		
		#pitch_rate	= negOne( Q * neg_feedback * kP_p )
		#roll_rate	= negOne( P * neg_feedback * kP_r )
		#yaw_rate	= negOne( R * neg_feedback * kP_y )
		
		prev_Q = Q
		prev_P = P
		prev_R = R

		# hack4stab... FIXME
		#if (thr_rate < 0.15):
		#	thr_rate = 0.15
		
		motors = list()
		
		if thr_rate > min_thr:
			motors.append(thr_rate + pitch_rate + roll_rate + yaw_rate)
			motors.append(thr_rate + pitch_rate - roll_rate - yaw_rate)
			motors.append(thr_rate - pitch_rate + roll_rate - yaw_rate)
			motors.append(thr_rate - pitch_rate - roll_rate + yaw_rate)
			
			motors = normal(motors)
		
			eng_fl = motors[0]
			eng_fr = motors[1]
			eng_bl = motors[2]
			eng_br = motors[3]
		else:
			eng_fl = 0
			eng_fr = 0
			eng_bl = 0
			eng_br = 0

		with open('log.csv', 'a') as f:
			f.write(f'{Q:6.2f};{P:6.2f};{R:6.2f};{pitch:6.2f};{roll:6.2f};{yaw:6.2f};{pitch_rate:6.2f};{roll_rate:6.2f};{yaw_rate:6.2f};{eng_fl:6.2f};{eng_fr:6.2f};{eng_bl:6.2f};{eng_br:6.2f};\n')

		mySendDREF(cl, drefs[dref_throttle], [ eng_fl, eng_fr, eng_bl, eng_br ])
	pass

def quit(event):
	global f_run
	
	f_run = False

def control(event):
	global f_control
	global f_override
	
	f_override = True
	if f_control:
		f_control = False
		print('[CONTROL] - Control is OFF')
	else:
		f_control = True
		print('[CONTROL] - Control is ON')

def main():
	global f_run
	global f_control
	global f_override

	client = xpc.XPlaneConnect()
	
	root = tk.Tk()
	root.title('XPC')
	root.minsize(300, 200)
	root.geometry('+1840+40')
	root.bind('q', quit)
	root.bind('c', control)
	labels = list()
	for i in range(0,40):
		l = tk.Label(root, text='')
		l.pack()
		labels.append(l)
	
	with open('log.csv', 'w') as f:
		f.write(f'Q;P;R;pitch;roll;yaw;pitch_rate;roll_rate;yaw_rate;eng_fl;eng_fr;eng_bl;eng_br;\n')
	
	while f_run:
		loop(client, labels)
		try:
			#loop(client, labels)
			pass
		except:
			print('[LOOP] - EXCEPTION')
			print('[LOOP] - Wait 2 second')
			time.sleep(2)
			print('[LOOP] - Restart...')
		
		root.update()
		
		# exit by press 'q' key in console
		if msvcrt.kbhit():
			key = msvcrt.getch()
			if ord(key) == ord('q'):
				f_run = False
			elif ord(key) == ord('c'):
				control(None)
	
	client.close()


if __name__ == '__main__':
	main()
