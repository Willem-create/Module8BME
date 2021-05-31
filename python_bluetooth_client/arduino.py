# Importing Libraries
import serial
import time
port="Com10"
baudrate = 9600
serialPort = 0

class Arduino:

    def __init__(self, port, baudrate):
        serialPort = serial.Serial(port=port, baudrate=baudrate, timeout=.1)
        #Idea: maybe a startup animation to indicate that the program started.
        # time.sleep(3)
        # serialPort.write(b"F2000\r\n")

    def frontUp(self, x):
        self.serialPort.write(bytes("F"+str(x)+"\r\n",encoding='UTF-8'))

    def frontDown(self, x):
        self.serialPort.write(bytes("f"+str(x)+"\r\n",encoding='UTF-8'))

    def backUp(self, x):
        self.serialPort.write(bytes("B"+str(x)+"\r\n",encoding='UTF-8'))

    def backDown(self, x):
        self.serialPort.write(bytes("b"+str(x)+"\r\n",encoding='UTF-8'))


