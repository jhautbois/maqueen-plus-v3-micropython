from microbit import*
def read_reg(reg,n=1):
	try:
		i2c.write(16,bytes([reg]));d=i2c.read(16,n)
		if n==1:return d[0]
		elif n==2:return d[0]|d[1]<<8
	except:return
def pid_distance(units,speed=2):'\n    Commande distance PID\n    units: unites de distance (hypothese: 1 unite = 2cm)\n    ';print('PID: %d unites @ speed %d'%(units,speed));i2c.write(16,bytes([64,1]));i2c.write(16,bytes([85,speed]));high=units>>8&255;low=units&255;i2c.write(16,bytes([65,high]));i2c.write(16,bytes([66,low]));i2c.write(16,bytes([60,6]))
def pid_angle(angle_deg,speed=2):print('PID rotate: %d deg @ speed %d'%(angle_deg,speed));direction=1 if angle_deg>=0 else 2;i2c.write(16,bytes([67,direction]));i2c.write(16,bytes([86,speed]));i2c.write(16,bytes([68,abs(angle_deg)]));i2c.write(16,bytes([60,6]))
def wait_done():
	'Attend status 0x57 = 0x01'
	while True:
		status=read_reg(87)
		if status==1:print('Done! (status=0x01)');sleep(100);return
		sleep(50)
TESTS=[('25u',25),('50u',50),('12u',12),('100u',100)]
current_test=0
print('=== Calibration PID ===')
print('A: Test suivant')
print('B: Rotation 180 deg')
while True:
	if button_a.was_pressed():
		if current_test<len(TESTS):name,units=TESTS[current_test];print('\n--- Test %s (%d unites) ---'%(name,units));print('Mesurez distance reelle!');display.show(Image.ARROW_N);pid_distance(units,speed=2);wait_done();display.show(Image.YES);current_test+=1
		else:
			print('Tous les tests termines!');print('Resultats:')
			for(name,units)in TESTS:print('  %s = %d unites'%(name,units))
			display.show(Image.HAPPY);current_test=0
	elif button_b.was_pressed():print('\n--- Rotation 180 deg ---');display.show(Image.ARROW_W);pid_angle(180,speed=2);wait_done();display.show(Image.YES)
	sleep(100)