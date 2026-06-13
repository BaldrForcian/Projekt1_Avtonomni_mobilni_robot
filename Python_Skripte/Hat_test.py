import serial
import time as sleep

ser = serial.Serial('/dev/ttySC0', 115200, timeout=1)

cmd = bytes.fromhex("01 A0 00 00 00 00 00 00 00 02")
ser.write(cmd)
sleep.sleep(1)

cmd = bytes.fromhex("01 74 00 00 00 00 00 00 00 04")
ser.write(cmd)
sleep.sleep(1)

cmd = bytes.fromhex("01 64 00 32 00 00 00 00 00 D3")
ser.write(cmd)
sleep.sleep(1)

cmd = bytes.fromhex("01 64 00 00 00 00 00 00 00 50")
ser.write(cmd)
sleep.sleep(1)

cmd = bytes.fromhex("01 64 FF CE 00 00 00 00 00 DA")
ser.write(cmd)
sleep.sleep(1)


data = ser.read(10)
print(data)