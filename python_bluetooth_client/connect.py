import bluetooth  # import bluetooth libary for communication with the esp32
import math
import time
from python_bluetooth_client import ImuSensor
from python_bluetooth_client import csv_writer
oldAngle = 0
oldTime = time.time()

#initialize csv files
Csv=csv_writer.CsvWriter()

print("Scanning...")
devices = bluetooth.discover_devices(lookup_names=True)  # searches for bluetooth devices
print(devices)
filter_list = [[] for _ in range(6)]  # creates list for moving average
wirelessIMUs = []
sensitivity_acc = 2048
sensitivity_gyro = 16.4
output_A = list()
kneeAngle= list()

for device in devices:
    if device[1] == 'WirelessIMU-6642' or device[1] == 'WirelessIMU-5F16':  # searches for a device called: WirelessIMUX. in which X is the number on your casing
        wirelessIMUs.append(device)

print("Found these devices: ", wirelessIMUs)
sensors = []
for addr, name in wirelessIMUs:  # if correct devices are found add them to the list for connection
    sensors.append(ImuSensor.ImuSensor(1, name))

while True:
    switch = True
    for sensor in sensors:

        if switch:
            output_A =sensor.take_measurement()
            Csv.write_value(output_A)
            switch = False
        else:
            output_B = sensor.take_measurement()
            Csv.write_value(output_B)
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



