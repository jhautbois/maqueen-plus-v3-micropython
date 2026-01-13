from microbit import*
def read_reg(reg,n=1):
	try:
		i2c.write(16,bytes([reg]));d=i2c.read(16,n)
		if n==1:return d[0]
		elif n==2:return d[0]|d[1]<<8
	except:return
def pid_distance(distance_cm,speed=2):print('PID: %dcm @ speed %d'%(distance_cm,speed));i2c.write(16,bytes([64,1]));i2c.write(16,bytes([85,speed]));dist_high=distance_cm>>8&255;dist_low=distance_cm&255;i2c.write(16,bytes([65,dist_high]));i2c.write(16,bytes([66,dist_low]));i2c.write(16,bytes([60,6]))
def pid_angle(angle_deg,speed=2):'Rotation PID (angle en degres)';print('PID rotate: %d deg @ speed %d'%(angle_deg,speed));direction=1 if angle_deg>=0 else 2;i2c.write(16,bytes([67,direction]));i2c.write(16,bytes([86,speed]));i2c.write(16,bytes([68,abs(angle_deg)]));i2c.write(16,bytes([60,6]))
def wait_pid_done(timeout_ms=5000):
	'Attend fin PID via registre 0x57';start=running_time()
	while running_time()-start<timeout_ms:
		status=read_reg(87);speed_l=read_reg(76);speed_r=read_reg(77);print('t=%d: status=0x%02X L=%d R=%d'%(running_time()-start,status or 0,speed_l or 0,speed_r or 0))
		if speed_l==0 and speed_r==0:print('Done! (vitesses = 0)');return True
		sleep(200)
	print('Timeout!');return False
print('=== Test PID avec Status ===')
print('A: Distance 50cm')
print('B: Rotation 90 deg')
while True:
	if button_a.was_pressed():print('\n--- PID Distance 50cm ---');display.show(Image.ARROW_N);pid_distance(50,speed=2);wait_pid_done();display.show(Image.YES)
	elif button_b.was_pressed():print('\n--- PID Rotation 90deg ---');display.show(Image.ARROW_E);pid_angle(90,speed=2);wait_pid_done();display.show(Image.YES)
	sleep(100)