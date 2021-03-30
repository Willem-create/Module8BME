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
    sensors.append(sensor)

start = time.time()

for i in range(100):
    for sensor in sensors:
        sensor.send('a')
        input = sensor.recv(12)
        print(input)


end = time.time()
avg = (end-start)/100
print('Average time: %f' %avg)