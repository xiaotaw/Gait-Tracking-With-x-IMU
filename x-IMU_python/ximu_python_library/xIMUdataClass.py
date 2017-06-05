import csv

class xIMUdata:
    def __init__(self,
                 filePath,
                 samplePeriod):

        # load csv file
        f = open(filePath, 'Ur')
        data = csv.DictReader(f)

        self.Time = []
        self.Gyroscope_X = []
        self.Gyroscope_Y = []
        self.Gyroscope_Z = []
        self.Accelerometer_X = []
        self.Accelerometer_Y = []
        self.Accelerometer_Z = []

        for value in data:
            if self.Time == []:
                self.Time += [0.0]
            else:
                self.Time += [self.Time[-1] + float(samplePeriod)]            
            self.Gyroscope_X += [float(value['Gyroscope_X_deg_s'])]
            self.Gyroscope_Y += [float(value['Gyroscope_Y_deg_s'])]
            self.Gyroscope_Z += [float(value['Gyroscope_Z_deg_s'])]
            self.Accelerometer_X += [float(value['Accelerometer_X_g'])]
            self.Accelerometer_Y += [float(value['Accelerometer_Y_g'])]
            self.Accelerometer_Z += [float(value['Accelerometer_Z_g'])]
        
        f.close()
        
