# Importing Libraries
import serial
import time
serialPort = 0
arduinoConnected = False

class Arduino:

    def __init__(self, port, baudrate):
        self.arduinoConnected=False
        try:
            self.serialPort = serial.Serial(port=port, baudrate=baudrate, timeout=.1)
            self.arduinoConnected = True
        except serial.serialutil.SerialException:
            print("No arduino found at this com port")

        #Idea: maybe a startup animation to indicate that the program started.
        # time.sleep(3)
        # serialPort.write(b"F2000\r\n")

    def frontUp(self, x):
        if self.arduinoConnected:
            self.serialPort.write(bytes("F"+str(x)+"\r\n",encoding='UTF-8'))

    def frontDown(self, x):
        if self.arduinoConnected:
            self.serialPort.write(bytes("f"+str(x)+"\r\n",encoding='UTF-8'))

    def backUp(self, x):
        if self.arduinoConnected:
           self.serialPort.write(bytes("B"+str(x)+"\r\n",encoding='UTF-8'))

    def backDown(self, x):
        if self.arduinoConnected:
            self.serialPort.write(bytes("b"+str(x)+"\r\n",encoding='UTF-8'))

    def startCycle(self):
        self.backUp(2000)
        time.sleep(1)
        self.backDown(2000)
        time.sleep(1)
        self.frontUp(2000)
        time.sleep(1)
        self.frontDown(2000)
