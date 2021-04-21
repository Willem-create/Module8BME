import bluetooth                                            # import bluetooth libary for communication with the esp32
import time                                                 # import time to calculate the frequency
from numpy import nanmean                                   # import numpy to calculate mean for moving average

print("Scanning...")
devices = bluetooth.discover_devices(lookup_names=True)     # searches for bluetooth devices
print(devices)
filter_list = [[] for _ in range(6)]                        # creates list for moving average
wirelessIMUs = []

for device in devices:
    if device[1] == 'WirelessIMU4':                         # searches for a device called: WirelessIMUX. in which X is the number on your casing
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


def moving_average(input, k):
    filter_list[k].append(input)
    if (len(filter_list[k]) >= 50):
        filter_list[k].pop(0)
    # x_set_angle = x_angle
    x_final = nanmean(filter_list[k])
    # turn = (x_set_angle - x_angle) / 180
    return int(x_final)


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


# end = time.time()
# avg = (end - start) / 1000
# f = 1 / avg
# print('frequency: %f' % f)

