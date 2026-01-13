from microbit import*
def r(reg):
	try:i2c.write(16,bytes([reg]));return i2c.read(16,1)[0]
	except:return 0
def pd(u):
	i2c.write(16,bytes([64,1]));i2c.write(16,bytes([85,2]));i2c.write(16,bytes([65,u>>8]));i2c.write(16,bytes([66,u&255]));i2c.write(16,bytes([60,6]))
	while r(87)!=1:sleep(50)
	sleep(100)
t=0
print('A: Next test')
while True:
	if button_a.was_pressed():
		if t==0:print('25u');display.show('1');pd(25)
		elif t==1:print('50u');display.show('2');pd(50)
		elif t==2:print('12u');display.show('3');pd(12)
		elif t==3:print('100u');display.show('4');pd(100)
		else:print('Done!');display.show(Image.HAPPY);t=-1
		t+=1;display.show(Image.YES)
	sleep(100)