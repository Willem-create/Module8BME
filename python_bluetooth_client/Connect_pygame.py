import bluetooth
import time
import matplotlib.pyplot as plt  # libary to plot
from scipy.ndimage.filters import uniform_filter1d  # moving average filter
from numpy import nanmean
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *

print("Scanning...")
devices = bluetooth.discover_devices(lookup_names=True)
print(devices)
N = 50
readings = arr = [[] for _ in range(6)]
IMU_data = [[] for _ in range(6)]
x_co_gy = 0

wirelessIMUs = []
for device in devices:
    if device[1] == 'WirelessIMU4':
        wirelessIMUs.append(device)

print("Found these devices: ", wirelessIMUs)

IMUservices = []

for addr, name in wirelessIMUs:
    print("Connecting to: ", addr)
    services = bluetooth.find_service(address=addr)
    for serv in services:
        if serv['name'] == b'ESP32SPP\x00':
            IMUservices.append(serv)

sensors = []

for IMU in IMUservices:
    sensor = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sensor.connect((IMU['host'], IMU['port']))
    sensor.setblocking(0)
    sensor.settimeout(1000)
    sensors.append(sensor)

start = time.time()

output = [None] * 6

for sensor in sensors:
    sensor.send('a')

filter_list = [[] for _ in range(6)]


def moving_average(input, k):
    filter_list[k].append(input)
    if (len(filter_list[k]) >= 50):
        filter_list[k].pop(0)
    # x_set_angle = x_angle
    x_final = nanmean(filter_list[k])
    # turn = (x_set_angle - x_angle) / 180
    return int(x_final)


# def my_filter(x):  # movingaverage filter + plots
# x_co_gy = uniform_filter1d(x, size=N)
# y_co = uniform_filter1d(y, size=N)
# z_co = uniform_filter1d(z, size=N)
# my_plot(x, x_co_gy, 'x_co')
# my_plot(y, y_co, 'y_co')
# my_plot(z, z_co, 'z_co')
# def my_plot(x_1, y_1, title):  # create the plots
#     plt.plot(x_1)
#     plt.plot(y_1)
#     plt.xlabel(title)
#     plt.show()
#
# def my_3dplot(xs, ys, zs):  # create 3d plots with all the data
#     fig = plt.figure()
#     ax = plt.axes(projection="3d")
#     ax.plot3D(xs, ys, zs)
#     plt.show()


line = ((0, 0, 0), (1, 0, 0))
# pygame.init()
display = (800, 600)
# pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
# gluPerspective(100, (display[0] / display[1]), 0.1, 50.0)
# glTranslatef(0.0, 0.0, -5)
x_angle = 0
x_set_angle = 0
while True:

    for sensor in sensors:
        inbytes = b''
        inbyte = [inbytes] * 6
        while len(inbytes) < 12:
            inbytes += sensor.recv(12 - len(inbytes))
        for z in range(0, 6):
            inbyte[z] += inbytes[z * 2:z * 2 + 2]
            inbyte[z] = int.from_bytes(inbyte[z], "big", signed="True")
            output[z] = moving_average(inbyte[z], z)
        sensor.send('a')

        print(inbyte, output)

    # for event in pygame.event.get():
    #     if event.type == pygame.QUIT:
    #         pygame.quit()
    #         quit()
    # glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    # # glRotatef((x_prev_angle-x_angle)/100, 0, 0, 1)
    # glRotatef(turn, 0, 0, 1)
    # glBegin(GL_LINES)
    # glVertex3fv(line[0])
    #
    # glVertex3fv(line[1])
    # glEnd()
    # pygame.display.flip()
    # # pygame.time.wait(10)

end = time.time()
avg = (end - start) / 1000
f = 1 / avg
print('frequency: %f' % f)

