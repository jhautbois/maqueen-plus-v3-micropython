from microbit import*
def read_reg(reg,n=1):
	"Lire n bytes d'un registre"
	try:
		i2c.write(16,bytes([reg]));d=i2c.read(16,n)
		if n==1:return d[0]
		elif n==2:return d[0]|d[1]<<8
	except:return
def motors(left,right):'Controle moteurs';i2c.write(16,bytes([0,0,left]));i2c.write(16,bytes([2,0,right]))
print('=== Test PID Register 0x3C ===')
print('A: Test avec 0x3C = 0x06 (MakeCode method)')
print('B: Test lecture registre 0x3C')
while True:
	if button_a.was_pressed():print('\n--- Test MakeCode PID ---');val_before=read_reg(60);print('0x3C avant: %s'%val_before);print('Ecriture 0x06 vers 0x3C...');i2c.write(16,bytes([60,6]));sleep(100);val_after=read_reg(60);print('0x3C apres: %s'%val_after);speed_l1=read_reg(76);speed_r1=read_reg(77);print('Vitesses avant: L=%s R=%s'%(speed_l1,speed_r1));display.show(Image.ARROW_N);motors(100,100);sleep(1000);motors(0,0);sleep(100);speed_l2=read_reg(76);speed_r2=read_reg(77);print('Vitesses apres: L=%s R=%s'%(speed_l2,speed_r2));print('Arret PID (0x10 vers 0x3C)...');i2c.write(16,bytes([60,16]));sleep(100);val_stop=read_reg(60);print('0x3C apres stop: %s'%val_stop);display.show(Image.YES)
	elif button_b.was_pressed():
		print('\n--- Lecture 0x3C ---');val=read_reg(60);print('0x3C = %s'%val)
		if val is not None:display.scroll(str(val))
		else:display.show(Image.NO)
	sleep(100)