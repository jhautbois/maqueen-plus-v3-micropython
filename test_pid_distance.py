from microbit import*
def read_reg(reg,n=1):
	try:
		i2c.write(16,bytes([reg]));d=i2c.read(16,n)
		if n==1:return d[0]
		elif n==2:return d[0]|d[1]<<8
	except:return
def pid_distance(distance_cm,speed=2):'\n    Commande PID distance (methode MakeCode)\n    distance_cm: distance en cm\n    speed: vitesse (2 = valeur par defaut MakeCode)\n    ';print('PID distance: %dcm, speed=%d'%(distance_cm,speed));i2c.write(16,bytes([64,1]));i2c.write(16,bytes([85,speed]));dist_high=distance_cm>>8&255;dist_low=distance_cm&255;i2c.write(16,bytes([65,dist_high]));i2c.write(16,bytes([66,dist_low]));i2c.write(16,bytes([60,6]));print('Commande envoyee!')
def pid_stop():'Arreter PID';i2c.write(16,bytes([60,16]));print('PID stop')
print('=== Test PID Distance Control ===')
print('A: Avancer 50cm avec PID')
print('B: Lire registres PID')
while True:
	if button_a.was_pressed():
		print('\n--- Test PID 50cm ---');speed_l1=read_reg(76);speed_r1=read_reg(77);print('Vitesses avant: L=%s R=%s'%(speed_l1,speed_r1));display.show(Image.ARROW_N);pid_distance(50,speed=2)
		for i in range(20):
			sleep(100);speed_l=read_reg(76);speed_r=read_reg(77);reg_3c=read_reg(60)
			if i%5==0:print('t=%dms: L=%s R=%s 0x3C=%s'%(i*100,speed_l,speed_r,reg_3c))
		pid_stop();sleep(100);speed_l2=read_reg(76);speed_r2=read_reg(77);print('Vitesses apres: L=%s R=%s'%(speed_l2,speed_r2));display.show(Image.YES)
	elif button_b.was_pressed():
		print('\n--- Lecture registres PID ---');regs=[60,64,65,66,67,68,85,86,87]
		for reg in regs:val=read_reg(reg);print('  0x%02X = %s'%(reg,val))
		display.show(Image.YES)
	sleep(100)