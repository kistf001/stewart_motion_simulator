import serial
import time

ser = []

def setting( port = "COM3", bit_rate = 115200 ):
    global ser
    ser = serial.Serial(port, 115200, timeout = 1)

def send_data(op):
    global ser
    ser.write(op.encode())
    #while True:
    #    print("insert op :", end=' ')
    #    op = input()
    #    print("R: ", ser.readline())

    #    if op is 'q':
    #        ser.close()

setting(  )

def viewer():
    while(True):
        print(ser.read().decode(),end='')
