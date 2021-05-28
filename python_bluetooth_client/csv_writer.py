import csv
csv_index =0

class CsvWriter:
    def __init__(self):
        self.csv_index = 0
        f = open("sensorA.csv", "w")
        f.write(" ")
        f.close()

        f = open("sensorB.csv", "w")
        f.write(" ")
        f.close()

        with open('sensorA.csv', mode='w') as sensorA:
            fieldnames = ['index', 'accX', 'accY', 'accZ', 'gyroX', 'gyroY', 'gyroZ']
            writerA = csv.DictWriter(sensorA, fieldnames=fieldnames)
            writerA.writeheader()

        with open('sensorB.csv', mode='w') as sensorB:
            fieldnames = ['index', 'accX', 'accY', 'accZ', 'gyroX', 'gyroY', 'gyroZ']
            writerB = csv.DictWriter(sensorB, fieldnames=fieldnames)
            writerB.writeheader()

    def write_value(self,values):
        if self.csv_index % 2==0:
            f = open("sensorA.csv", "a")
        else:
            f = open("sensorB.csv", "a")

        csv_output = str(values)
        csv_output = csv_output.replace("[", "")
        csv_output = csv_output.replace("]", "")
        f.write(str(self.csv_index) + "," + csv_output + "\n")
        f.close()
        self.csv_index += 1
