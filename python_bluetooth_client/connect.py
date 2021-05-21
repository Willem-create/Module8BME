import bluetooth  # import bluetooth libary for communication with the esp32
from numpy import nanmean  # import numpy to calculate mean for moving average
import numpy as np
import csv
from pprint import pprint
import math
import time
from scipy.signal import find_peaks
from scipy import interpolate
import matplotlib.pyplot as plt


print("startplz")
calculated= True
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
kneeTime = list()

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

IMUservices = []

for addr, name in wirelessIMUs:  # if correct devices are found add them to the list for connection
    print("Connecting to: ", addr)
    services = bluetooth.find_service(address=addr)
    for serv in services:
        if serv['name'] == b'ESP32SPP\x00':
            IMUservices.append(serv)

sensors = []

for IMU in IMUservices:  # initialize the IMU's as sensors
    sensor = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sensor.connect((IMU['host'], IMU['port']))
    sensor.setblocking(0)
    sensor.settimeout(1000)
    sensors.append(sensor)

output = [None] * 6  # Create list to store data
output_real = [None] * 6

for sensor in sensors:
    sensor.send('a')


def moving_average(input, k):  # moving average function !!for one sensor only now!!  returns the moving average of the data
    filter_list[k].append(input)  # add new value to the list
    if (len(filter_list[
                k]) >= 2):  # if list is larger then N remove the oldest data point, N determines the size of your list for the moving average
        filter_list[k].pop(0)
    final = nanmean(filter_list[k])  # Calculate the mean of the list
    return final  # Return mean


def real_numbers(input, k):  # real numbers function transfers into actual values
    if k <= 2:  # first 3 are acc data so divide by that sensitivity
        converted = input / sensitivity_acc
    if k >= 3:  # Last 3 are gyro data so divide by that sensitivity
        converted = input / sensitivity_gyro
    return converted  # returns converted value


while True:
    switch = True
    for sensor in sensors:

        inbytes = b''
        inbyte = [inbytes] * 6
        while len(inbytes) < 12:
            inbytes += sensor.recv(12 - len(inbytes))  # Collects data from sensor in bytes
        for z in range(0, 6):
            inbyte[z] += inbytes[z * 2:z * 2 + 2]
            inbyte[z] = int.from_bytes(inbyte[z], "big", signed="True")  # converts from bytes to int
            #output[z] = moving_average(inbyte[z], z)  # Calls moving average function
            output_real[z] = real_numbers(inbyte[z], z)  # calls real_numbers function
            # print(sensor, output_real[2])

        csv_index1 += 1

        if switch:
            f = open("sensorA.csv", "a")
            output_A=list(output_real)
            switch = False
        else:
            f = open("sensorB.csv", "a")
            output_B=str(output_real)
            first_part=output_A[0]*output_real[0]+output_A[1]*output_real[1]+output_A[2]*output_real[2]
            sqrA=math.sqrt(pow(output_A[0],2)+pow(output_A[1],2)+pow(output_A[2],2))
            sqrB=math.sqrt(pow(output_real[0],2)+pow(output_real[1],2)+pow(output_real[2],2))
            angle=first_part/(sqrA*sqrB)
            if angle>1:
                angle=1
            angle_radian=math.acos(angle)
            angle_degree=angle_radian*180/math.pi
            print(str(angle_degree))
            kneeAngle.append(angle_degree)
            kneeTime.append(time.time())  #get the timestamp
            

            peaksy, _ = find_peaks(kneeAngle,prominence=3)  #get the peaks
            switch = True
            xpoints2 = np.array(kneeTime)
            ypoints2 = np.array(kneeAngle)

            averages = 10 #number of averages to take
            waitforpeaks=5 #wait for this amount of peaks before calculating
            if len(peaksy)==(averages*2)+1+waitforpeaks and calculated:  
                calculated = False
                averagelength=round((peaksy[averages*2]-peaksy[0])/averages)

                sampley=[]
                samplex=[]
                taa=[0]*averagelength
                outputsampley=[]
                
                for x in range(waitforpeaks,(averages*2)+waitforpeaks,2):
                    
                    samplex.append(xpoints2[peaksy[x]:peaksy[x+2]])
                    sampley.append(ypoints2[peaksy[x]:peaksy[x+2]])

                for x in range(0,len(samplex)):
                    newx= np.linspace(samplex[x][0],samplex[x][-1],averagelength*len(samplex[x]))
                    intf = interpolate.interp1d(samplex[x],sampley[x])
                    interpsample = intf(newx)
                    print(len(interpsample))

                    for i in range(0,averagelength):
                        taa[i] = (interpsample[i*len(samplex[x])])

                    outputsampley.append(taa)
                    taa=[0]*averagelength

                average_stride=[0]*averagelength
                    
                for x in range(0,averages):
                        average_stride = [average_stride[i] + outputsampley[x][i] for i in range(len(outputsampley[x]))]
                average_stride[:]=[x/(averages) for x in average_stride]
                plt.plot(average_stride)
                plt.show()
                    



        csv_output=str(output_real)
        csv_output=csv_output.replace("[","")
        csv_output=csv_output.replace("]","")
        f.write(str(csv_index1)+","+csv_output+"\n")
        f.close()

        sensor.send('a')
        # print(sensor, output_real[3])

