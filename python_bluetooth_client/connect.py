import bluetooth
import time

devices = bluetooth.discover_devices(lookup_names=True)
print(devices)

wirelessIMUs = []
for device in devices:
    if device[1] == 'WirelessIMU':
        wirelessIMUs.append(device)

print(wirelessIMUs)

IMUservices = []

for addr, name in wirelessIMUs:
    print(addr)
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

for i in range(1000):
    for sensor in sensors:
        inbytes = b''
        while len(inbytes)<12:
            inbytes+=sensor.recv(12-len(inbytes))
        sensor.send('a')
        #input = sensor.recv(12)
        for j in range(0, 12, 2):
            output[int(j/2)] = inbytes[j] << 8 | inbytes[j + 1]


end = time.time()
avg = (end - start) / 1000
f = 1 / avg
print('frequency: %f' % f)
