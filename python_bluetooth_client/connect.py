import bluetooth  # import bluetooth libary for communication with the esp32
from numpy import nanmean  # import numpy to calculate mean for moving average
import csv
from pprint import pprint
import math
import time
from python_bluetooth_client import Imu
from python_bluetooth_client import ImuSensor
oldAngle = 0
oldTime = time.time()

print("Scanning...")
devices = bluetooth.discover_devices(lookup_names=True)  # searches for bluetooth devices
print(devices)
filter_list = [[] for _ in range(6)]  # creates list for moving average
wirelessIMUs = []
sensitivity_acc = 2048
sensitivity_gyro = 16.4
output_A = list()
kneeAngle= list()

csv_index1 = 0
f = open("sensorA.csv", "w")
f.write(" ")
f.close()

f = open("sensorB.csv", "w")
f.write(" ")
f.close()

with open('sensorA.csv', mode='w') as sensorA:
    fieldnames = ['index', 'accX', 'accY', 'accZ', 'gyroX', 'gyroY', 'gyroZ']
    writerA= csv.DictWriter(sensorA, fieldnames=fieldnames)
    writerA.writeheader()

with open('sensorB.csv', mode='w') as sensorB:
    fieldnames = ['index', 'accX', 'accY', 'accZ', 'gyroX', 'gyroY', 'gyroZ']
    writerB= csv.DictWriter(sensorB, fieldnames=fieldnames)




for device in devices:
    if device[1] == 'WirelessIMU-6642' or device[1] == 'WirelessIMU-5F16':  # searches for a device called: WirelessIMUX. in which X is the number on your casing
        wirelessIMUs.append(device)

print("Found these devices: ", wirelessIMUs)
bluetoothSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sensors = []
for addr, name in wirelessIMUs:  # if correct devices are found add them to the list for connection
    #sensors.append(Imu.Imu(addr, name, sensitivity_acc, sensitivity_gyro, bluetoothSocket))
    sensors.append(ImuSensor.ImuSensor(1, name))

while True:
    switch = True
    for sensor in sensors:

        csv_index1 += 1
        if switch:
            f = open("sensorA.csv", "a")
            #output_A=sensor.receive()
            output_A =sensor.take_measurement()
            switch = False
        else:
            f = open("sensorB.csv", "a")
            #output_B=sensor.receive()
            output_B = sensor.take_measurement()
            first_part=output_A[0]*output_B[0]+output_A[1]*output_B[1]+output_A[2]*output_B[2]
            sqrA=math.sqrt(pow(output_A[0],2)+pow(output_A[1],2)+pow(output_A[2],2))
            sqrB=math.sqrt(pow(output_B[0],2)+pow(output_B[1],2)+pow(output_B[2],2))
            angle=first_part/(sqrA*sqrB)
            if angle>1:
                angle=1
            angle_radian=math.acos(angle)
            angle_degree=angle_radian*180/math.pi
            print(str(angle_degree))
            kneeAngle.append(angle_degree)
            switch = True

        #csv_output=str(output_real)
        csv_output=str([0.06591796875,-5029296875,-10830078125,-1.15854,2.0122,-0.12195121951219513])
        csv_output=csv_output.replace("[","")
        csv_output=csv_output.replace("]","")
        f.write(str(csv_index1)+","+csv_output+"\n")
        f.close()

        #sensor.send('a')
        # print(sensor, output_real[3])

