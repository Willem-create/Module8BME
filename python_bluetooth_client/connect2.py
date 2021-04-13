import bluetooth
import time
import matplotlib.pyplot as plt  # libary to plot
from scipy.ndimage.filters import uniform_filter1d  # moving average filter
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
ax = ay = az = 0.0
yaw_mode = False

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


# def my_filter(x):  # movingaverage filter + plots
# x_co_gy = uniform_filter1d(x, size=N)
# y_co = uniform_filter1d(y, size=N)
# z_co = uniform_filter1d(z, size=N)
# my_plot(x, x_co_gy, 'x_co')
# my_plot(y, y_co, 'y_co')
# my_plot(z, z_co, 'z_co')
def my_plot(x_1, y_1, title):  # create the plots
    plt.plot(x_1)
    plt.plot(y_1)
    plt.xlabel(title)
    plt.show()


def my_3dplot(xs, ys, zs):  # create 3d plots with all the data
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.plot3D(xs, ys, zs)
    plt.show()


def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def drawText(position, textString):
    font = pygame.font.SysFont("Courier", 18, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


def draw():
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax))

    if yaw_mode:
        osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))
    else:
        osd_line = osd_text

    drawText((-2, -2, 2), osd_line)

    # the way I'm holding the IMU board, X and Y axis are switched
    # with respect to the OpenGL coordinate system
    if yaw_mode:  # experimental
        glRotatef(az, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(ay, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
    glRotatef(-1 * ax, 0.0, 0.0, 1.0)  # Roll,  rotate around z-axis

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def read_data(data):
    global ax, ay, az
    ax = ay = az = 0.0
    # line_done = 0

    # request data by sending a dot
    # ser.write(b".")  # * encode string to bytes
    # # while not line_done:
    # line = ser.readline()
    # angles = line.split(b", ")
    if len(data) >= 3:
        ax = float(data[0])
        ay = float(data[1])
        az = float(data[2])
        # line_done = 1


def main():
    global yaw_mode
    video_flags = OPENGL | DOUBLEBUF

    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(640, 480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()  # * quit pygame properly
            break
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
            # ser.write(b"z")


        pygame.display.flip()
        frames = frames + 1

    print("fps:  %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))
    # ser.close()


#if __name__ == '__main__': main()

while True:
    for sensor in sensors:
        inbytes = b''
        while len(inbytes) < 12:
            inbytes += sensor.recv(12 - len(inbytes))
        sensor.send('a')
        # input = sensor.recv(12)
        for j in range(0, 12, 2):
            output[int(j / 2)] = inbytes[j] << 8 | inbytes[j + 1]
        for k in range(6):
            readings[k].append(output[k])
            IMU_data[k] = uniform_filter1d(readings[k], size=N)
            print(IMU_data)
            read_data(IMU_data)
            draw()

end = time.time()
avg = (end - start) / 1000
f = 1 / avg
print('frequency: %f' % f)

# for i in range(6):
#     my_plot(readings[i], IMU_data[i], 'x_co')

# my_3dplot(IMU_data[0], IMU_data[1], IMU_data[2])
# my_3dplot(IMU_data[3], IMU_data[4], IMU_data[5])
