import serial
import struct

ser = serial.Serial(
    port='COM6',\
    baudrate=34800,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)
count=1

while True:
    reading = ser.readline()
    print(type(reading[0]))

ser.close()