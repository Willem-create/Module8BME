import bluetooth  # import bluetooth libary for communication with the esp32
from numpy import nanmean  # import numpy to calculate mean for moving average
import numpy as np
import math
import time
import ImuSensor
import gui
import csv_writer
from scipy.signal import find_peaks
from scipy import interpolate
import matplotlib.pyplot as plt

#initialize csv files
Csv=csv_writer.CsvWriter()

print("startplz")
calculated= True
oldAngle = 0

intanglexA=0
intangleyA=0
intanglezA=0
intanglexB=0
intangleyB=0
intanglezB=0

alpha = 0.5
prevgyroangle=0
lowpassout=0
highpassout=0

prevtime=time.time()
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

front_end = gui.Gui()

for device in devices:
    if device[1] == 'WirelessIMU-6642' or device[1] == 'WirelessIMU-5F16':  # searches for a device called: WirelessIMUX. in which X is the number on your casing
        wirelessIMUs.append(device)

front_end.register_imus(wirelessIMUs)
sensors = []
for addr, name in wirelessIMUs:  # if correct devices are found add them to the list for connection
    front_end.set_imu_status(name, "Pairing")
    connectedSensor = ImuSensor.ImuSensor(1, name)
    front_end.set_imu_status(name, "Connected")
    sensors.append(connectedSensor)

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
            #print(str(angle_degree))

            anglexA = output_A[3]
            angleyA = output_A[4]
            anglezA = output_A[5]

            anglexB = output_B[3]
            angleyB = output_B[4]
            anglezB = output_B[5]

            dt = time.time() - prevtime
            intanglexA = anglexA * dt + intanglexA
            intangleyA = angleyA * dt + intangleyA
            intanglezA = anglezA * dt + intanglezA

            intanglexB = anglexB * dt + intanglexB
            intangleyB = angleyB * dt + intangleyB
            intanglezB = anglezB * dt + intanglezB
            prevtime = time.time()

            gyroangle = math.sqrt(math.pow((intanglexA - intanglexB), 2) + math.pow((intangleyA - intangleyB), 2) + math.pow((intanglezA - intanglezB), 2))
            lowpassout = (1 - alpha) * angle_degree + alpha * lowpassout
            highpassout = (1 - alpha) * highpassout + (1 - alpha) * (gyroangle - prevgyroangle)
            prevgyroangle = gyroangle

            total_angle = lowpassout + highpassout
            front_end.update_angle(total_angle)

            kneeAngle.append(total_angle)
            kneeTime.append(time.time())  # get the timestamp

            peaksy, _ = find_peaks(kneeAngle, prominence=3)  # get the peaks
            switch = True
            xpoints2 = np.array(kneeTime)
            ypoints2 = np.array(kneeAngle)

            averages = 10  # number of averages to take
            waitforpeaks = 5  # wait for this amount of peaks before calculating
            if len(peaksy) == (averages * 2) + 1 + waitforpeaks and calculated:
                calculated = False
                averagelength = round((peaksy[averages * 2] - peaksy[0]) / averages)

                sampley = []
                samplex = []
                taa = [0] * averagelength
                outputsampley = []

                for x in range(waitforpeaks, (averages * 2) + waitforpeaks, 2):
                    samplex.append(xpoints2[peaksy[x]:peaksy[x + 2]])
                    sampley.append(ypoints2[peaksy[x]:peaksy[x + 2]])

                for x in range(0, len(samplex)):
                    newx = np.linspace(samplex[x][0], samplex[x][-1], averagelength * len(samplex[x]))
                    intf = interpolate.interp1d(samplex[x], sampley[x])
                    interpsample = intf(newx)
                    # print(len(interpsample))

                    for i in range(0, averagelength):
                        taa[i] = (interpsample[i * len(samplex[x])])

                    outputsampley.append(taa)
                    taa = [0] * averagelength

                average_stride = [0] * averagelength

                for x in range(0, averages):
                    average_stride = [average_stride[i] + outputsampley[x][i] for i in range(len(outputsampley[x]))]
                average_stride[:] = [x / (averages) for x in average_stride]
                plt.plot(average_stride)
                plt.show()
                kneeAngle=[]
                kneeTime=[]
            switch = True



