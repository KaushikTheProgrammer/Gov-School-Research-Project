import serial
import time

smartCar = serial.Serial('/dev/tty.HC-05-DevB', 115200, timeout=0.1) # Establish the connection on a specific port)
time.sleep(2)
smartCar.write(b'180')
time.sleep(0.1)
smartCar.close()