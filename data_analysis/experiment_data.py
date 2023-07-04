import numpy as np

class ExpData:
    def __init__(self, path, pre_crop, post_crop):
        # Containers for formating and storing the raw data into
        self.motor_pos = []
        self.cam_pos = []

        self.motor_current = []
        self.motor_speed = []

        self.load_cell = []

        self.time = []

        with open(path) as f:
            lines = f.readlines() #Get lines from csv / txt file
            lines = lines[pre_crop:post_crop] # crop unwanted data.
            t0 = int(lines[0].split(',')[5]) #get time for the first data point

            # Extract and format the data
            for line in lines:
                split = line.split(',')
                self.motor_pos.append(int(split[0])*2*np.pi/1000)
                self.cam_pos.append(int(split[1])*2*np.pi/4096)
                self.motor_current.append(float(split[2]))
                self.motor_speed.append(float(split[3]))
                self.load_cell.append(float(split[4]))
                self.time.append(int(split[5])-t0)

