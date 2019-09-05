print('en fn ')
exit()
import serial
from time import sleep
ser = serial.Serial("/dev/ttyACM0", 9600)
sleep(2)
print("connection Est");
a=1
while a!=0 :
	ser.write(str.encode('0 0 0 0 '+str(a)+' 0 0 0\n'));
	# ser.write(str.encode('125 175 1 1\n'))
	print(ser.readline())
	a = int(input('Enter'))

ser.write(str.encode('0 0 0 0 0 0 0 0\n'));
print(ser.readline())


exit()