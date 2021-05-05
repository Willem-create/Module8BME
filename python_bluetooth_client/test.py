from ImuSensor import ImuSensor, MODE_WIRED, MODE_WIRELESS

sensor = ImuSensor(MODE_WIRELESS, 'WirelessIMU-0FD6')

print(sensor.take_measurement())
