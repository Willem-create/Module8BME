import bluetooth  # import bluetooth libary for communication with the esp32
from numpy import nanmean  # import numpy to calculate mean for moving average

class Imu:
    service=[]

    output = [None] * 6  # Create list to store data
    output_real = [None] * 6
    sensitivity_acc = 0
    sensitivity_gyro = 0
    sensor= None

    def __init__(self,addr, name, sensitivity_acc, sensitivity_gyro, bluetoothSocket):
        self.sensitivity_acc=sensitivity_acc
        self.sensitivity_gyro=sensitivity_gyro
        print("Connecting to: ", addr)
        services = bluetooth.find_service(address=addr)
        for serv in services:
            if serv['name'] == b'ESP32SPP\x00':
                self.service=serv
        self.sensor=bluetoothSocket
        self.connect()

    def connect(self):
        self.sensor.connect((self.service['host'], self.service['port']))
        self.sensor.setblocking(0)
        self.sensor.settimeout(1000)
        self.sensor.send('a')

    def receive(self):
        inbytes = b''
        inbyte = [inbytes] * 6
        while len(inbytes) < 12:
            inbytes += self.sensor.recv(12 - len(inbytes))  # Collects data from sensor in bytes
        for z in range(0, 6):
            inbyte[z] += inbytes[z * 2:z * 2 + 2]
            inbyte[z] = int.from_bytes(inbyte[z], "big", signed="True")  # converts from bytes to int
            # output[z] = moving_average(inbyte[z], z)  # Calls moving average function
            self.output_real[z] = self.real_numbers(inbyte[z], z)  # calls real_numbers function
        return self.output_real

    # Not sure if we need this function, but I pasted it in just in case
    def moving_average(input, k):  # moving average function !!for one sensor only now!!  returns the moving average of the data
        filter_list = [[] for _ in range(6)]  # creates list for moving average
        filter_list[k].append(input)  # add new value to the list
        if (len(filter_list[
                    k]) >= 2):  # if list is larger then N remove the oldest data point, N determines the size of your list for the moving average
            filter_list[k].pop(0)
        final = nanmean(filter_list[k])  # Calculate the mean of the list
        return final  # Return mean


    def real_numbers(input, k):  # real numbers function transfers into actual values
        if k <= 2:  # first 3 are acc data so divide by that sensitivity
            converted = input / self.sensitivity_acc
        if k >= 3:  # Last 3 are gyro data so divide by that sensitivity
            converted = input / self.sensitivity_gyro
        return converted  # returns converted value

