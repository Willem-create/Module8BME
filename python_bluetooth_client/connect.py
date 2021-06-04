import bluetooth  # import bluetooth libary for communication with the esp32
# pip install python_bluetooth_client\\PyBluez-0.22-cp38-cp38-win_amd64.whl
import eel
import numpy as np
import math
import time
import ImuSensor
import gui
import csv_writer
import arduino
from scipy.signal import find_peaks
from scipy import interpolate
import matplotlib.pyplot as plt
import keyboard

uselargepeak = False #true: giving feedback on largepeak, false: giving feedback on smallpeak
errorlargetrigger=[-4,4] #first value is the trigger for understretching, second is for overstretching
errorsmalltrigger=[-15,15]

front_end = gui.Gui()

# initialize csv files
print("initializing csv")
writeCSVs = False
Csv = csv_writer.CsvWriter()

# initialize serial communicaiton (com_port and baudrate)
print("starting arduino")
Arduino = arduino.Arduino("Com10", 9600)
time.sleep(1)
Arduino.startCycle()

print("startplz")
calculated = True
calculated_baseline = False
oldAngle = 0

intanglexA = 0
intangleyA = 0
intanglezA = 0
intanglexB = 0
intangleyB = 0
intanglezB = 0

alpha = 0.3
prevgyroangle = 0
lowpassout = 0
highpassout = 0

prevtime = time.time()
oldTime = time.time()

devices = bluetooth.discover_devices(lookup_names=True)  # searches for bluetooth devices
print(devices)
filter_list = [[] for _ in range(6)]  # creates list for moving average
wirelessIMUs = []
sensitivity_acc = 2048
sensitivity_gyro = 16.4
output_A = list()
kneeAngle = list()
kneeTime = list()

average_stridelength = 200

for device in devices:
    if device[1] == 'WirelessIMU-6642' or device[
        1] == 'WirelessIMU-5F16':  # searches for a device called: WirelessIMUX. in which X is the number on your casing
        wirelessIMUs.append(device)
print("found the two sensors: "+str(wirelessIMUs))

front_end.register_imus(wirelessIMUs)
sensors = []
for addr, name in wirelessIMUs:  # if correct devices are found add them to the list for connection
    front_end.set_imu_status(name, "Pairing")
    front_end.sleep(0.001)
    connectedSensor = ImuSensor.ImuSensor(1, name)
    front_end.set_imu_status(name, "Connected")
    sensors.append(connectedSensor)


def compute_averagestride(averages, averagelength, waitforpeaks, xpoints2, ypoints2, peaksy):
    stridetime = (xpoints2[0] - xpoints2[-1]) / averages
    sampley = []
    samplex = []
    taa = [0] * averagelength
    outputsampley = []

    for x in range(waitforpeaks, (averages * 2) + waitforpeaks, 2):
        samplex.append(xpoints2[peaksy[x]:peaksy[x + 2]])
        sampley.append(ypoints2[peaksy[x]:peaksy[x + 2]])

    for x in range(0, len(samplex)):
        newx = np.linspace(samplex[x][0], samplex[x][-1], averagelength * average_stridelength)
        intf = interpolate.interp1d(samplex[x], sampley[x])
        interpsample = intf(newx)
        # print(len(interpsample))

        for i in range(0, averagelength):
            taa[i] = (interpsample[i * average_stridelength])

        outputsampley.append(taa)
        taa = [0] * averagelength

    average_stride = [0] * averagelength

    for x in range(0, averages):
        average_stride = [average_stride[i] + outputsampley[x][i] for i in range(len(outputsampley[x]))]
    average_stride[:] = [x / (averages) for x in average_stride]
    # plt.plot(average_stride)
    # plt.show()

    return average_stride, stridetime

def resetandrecal():
    global calculated, calculated_baseline, kneeAngle, kneeTime, peaksy
    front_end.notification('Reseting stride detection!')
    calculated = True
    calculated_baseline = False
    kneeAngle = []
    kneeTime = []
    peaksy = []


while True:
    switch = True
    for sensor in sensors:

        if switch:
            output_A = sensor.take_measurement()
            if writeCSVs:
                Csv.write_value(output_A)
            switch = False
        else:
            output_B = sensor.take_measurement()
            if writeCSVs:
                Csv.write_value(output_B)
            first_part = output_A[0] * output_B[0] + output_A[1] * output_B[1] + output_A[2] * output_B[2]
            sqrA = math.sqrt(pow(output_A[0], 2) + pow(output_A[1], 2) + pow(output_A[2], 2))
            sqrB = math.sqrt(pow(output_B[0], 2) + pow(output_B[1], 2) + pow(output_B[2], 2))
            # if (sqrA * sqrB) < 0.0001 & (sqrA * sqrB) > -0.0001:
            #     angle = 0
            # else:
            angle = first_part / (sqrA * sqrB)
            if angle > 1:
                angle = 1
            angle_radian = math.acos(angle)
            angle_degree = angle_radian * 180 / math.pi
            # print(str(angle_degree))

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

            gyroangle = math.sqrt(
                math.pow((intanglexA - intanglexB), 2) + math.pow((intangleyA - intangleyB), 2) + math.pow(
                    (intanglezA - intanglezB), 2))
            lowpassout = (1 - alpha) * angle_degree + alpha * lowpassout
            highpassout = (1 - alpha) * highpassout + (1 - alpha) * (gyroangle - prevgyroangle)
            prevgyroangle = gyroangle

            total_angle = lowpassout + highpassout
            front_end.update_angle(round(total_angle, 1))
            # print(total_angle)

            kneeAngle.append(total_angle)
            kneeTime.append(time.time())  # get the timestamp

            peaksy, _ = find_peaks(kneeAngle, prominence=3)  # get the peaks
            switch = True
            xpoints2 = np.array(kneeTime)
            ypoints2 = np.array(kneeAngle)
            averages = 6  # number of averages to take
            waitforpeaks = 2  # wait for this amount of peaks before calculating
            # print("len(peaksy) is "+str(len(peaksy))+", calculated is "+str(calculated)+", calculated_baseline is "+str(calculated_baseline))
            if len(peaksy) == (averages * 2) + 1 + waitforpeaks and calculated and calculated_baseline == False:
                averagelength = round((peaksy[averages * 2] - peaksy[0]) / averages)
                average_stride1, stridetime = compute_averagestride(averages, averagelength, waitforpeaks, xpoints2,
                                                                    ypoints2, peaksy)
                calculated = False
                calculated_baseline = True
                kneeAngle = []
                kneeTime = []
                peaksy = []

                baselinelabel = [None] * len(average_stride1)
                baselinevalue = [None] * len(average_stride1)
                for i in range(0, len(average_stride1)):
                    baselinelabel[i] = round(i * (stridetime / len(average_stride1)), 2)
                    baselinevalue[i] = average_stride1[i]
                front_end.set_baseline_graph(baselinelabel, baselinevalue)


                print("determined the baseline")
                front_end.notification('Baseline determined')
            if len(peaksy) == (averages * 2) + 2 + waitforpeaks:
                calculated = True
            if len(peaksy) >= (averages * 2) + 1 + waitforpeaks and calculated and calculated_baseline:

                average_stride2, stridetime = compute_averagestride(averages, averagelength, waitforpeaks, xpoints2,
                                                                    ypoints2, peaksy)
                calculated = False

                averagelabel = [None] * len(average_stride2)
                averagevalue = [None] * len(average_stride2)
                for i in range(0, len(average_stride2)):
                    averagelabel[i] = round(i * (stridetime / len(average_stride2)), 2)
                    averagevalue[i] = average_stride2[i]
                front_end.set_average_graph(averagelabel, averagevalue)

                kneeAngle = []
                kneeTime = []
                peaksy = []
                stride2peak, _ = find_peaks(average_stride2, prominence=1)
                firstpeak = average_stride2[0]
                if len(stride2peak) == 0:
                    stride2peak = [0]
                secondpeak = average_stride2[int(stride2peak[0])]
                secondpeak = 0
                if (firstpeak > secondpeak):
                    largepeak = firstpeak
                    smallpeak = secondpeak
                    tlargepeak = 0
                    tsmallpeak = stride2peak[0] * stridetime / 200
                    smallpeakx = stride2peak[0]
                    largepeakx = 0
                else:
                    largepeak = secondpeak
                    smallpeak = firstpeak
                    tlargepeak = stride2peak[0] * stridetime / 200
                    tsmallpeak = 0
                    smallpeakx = 0
                    largepeakx = stride2peak[0]

                error = []
                for i in range(0, len(average_stride1)):
                    error.append(average_stride1[i] - average_stride2[i])

                errorlabel = [None] * len(error)
                errorvalue = [None] * len(error)
                for i in range(0, len(error)):
                    errorvalue[i] = -1 * error[i]
                    errorlabel[i] = round(i * (stridetime / len(error)), 2)
                front_end.set_error_graph(errorlabel, errorvalue)

                errorsmallpeak = error[smallpeakx]
                errorlargepeak = error[largepeakx]
                # give feedback() using error
                print("error largepeak calculated: " + str(errorlargepeak))
                print("error smallpeak calculated: " + str(errorsmallpeak))
                if uselargepeak: #boolean to switch between using small or large peak for feedback
                    if errorlargepeak < errorlargetrigger[0]:
                        Arduino.backUp(500)
                        Arduino.backDown(500)
                        print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tyou are understretching (large peak)")
                        front_end.feedback('Keep your leg straighter during its forward motion', 'Forward swing')
                    else:
                        if errorlargepeak > errorlargetrigger[1]:
                            Arduino.frontUp(500)
                            Arduino.frontDown(500)
                            print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tyou might be overstretching (large peak)")
                            front_end.feedback('Bend you leg more during its forward motion', 'Forward swing')
                        else:
                            print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tyou are doing fine (large peak)")
                            front_end.feedback('You are doing great', 'Forward swing')
                else:
                    if errorsmallpeak < errorsmalltrigger[0]:
                        Arduino.backUp(500)
                        Arduino.backDown(500)
                        print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tyou are understretching (small peak)")
                        front_end.feedback('Keep your leg straighter during its backward motion', 'Backward swing')
                    else:
                        if errorsmallpeak > errorsmalltrigger[1]:
                            Arduino.frontUp(500)
                            Arduino.frontDown(500)
                            print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tyou might be overstretching (small peak)")
                            front_end.feedback('Bend you leg more during its backward motion', 'Backward swing')
                        else:
                            print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tyou are doing fine (small peak)")
                            front_end.feedback('You are doing great', 'Backward swing')

            if len(peaksy) == (averages * 2) + 2 + waitforpeaks:
                calculated = True
            switch = True
    front_end.sleep(0.1)
    try:  # used try so that if user pressed other than the given key error will not be shown
        if keyboard.is_pressed('r'):  # if key 'r' is pressed
            resetandrecal()
        if keyboard.is_pressed('a'):  # if key 'a' is pressed
            front_end.notification('Testing Arduino!')
            Arduino.startCycle()
        if keyboard.is_pressed('s'):  # if key 's' is pressed
            if uselargepeak:
                front_end.notification('Set mode to: Backward swing')
                print('Set mode to: Backward swing, smallpeak')
                uselargepeak=False
            else:
                front_end.notification('Set mode to: Forward swing')
                print('Set mode to: Forward swing, largepeak')
                uselargepeak = True
    except:
        temp = 0
