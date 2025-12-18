import serial
import time

port = '/dev/ttyS0'
baud = 115200

ser = serial.Serial(port, baud)
time.sleep(2)

while True:
    if ser.in_waiting > 0:
        response = ser.read()

        if response:
            decoded_byte = response.decode('utf-8')
            print(f"Received: {decoded_byte}")

            if decoded_byte == '0':
                ser.write(b'0')
            else:
                ser.write(b'1')

    time.sleep(0.5)
