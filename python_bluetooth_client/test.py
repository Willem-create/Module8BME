import csv

csv_index1 = 0
csv_index2 = 0
start = False
with open('sensorA.csv', mode='w') as sensorA:
    fieldnames = ['index', 'accX', 'accY', 'accZ', 'gyroX', 'gyroY', 'gyroZ']
    writerA = csv.DictWriter(sensorA, fieldnames=fieldnames)
    writerA.writeheader()
    start = True



while start:
    writerA.writerow('index': '1', 'accX': '1', 'accY': '1', 'accZ': '1', 'gyroX': '1',
                      'gyroY': '1', 'gyroZ': '1')
    csv_index1 += 1