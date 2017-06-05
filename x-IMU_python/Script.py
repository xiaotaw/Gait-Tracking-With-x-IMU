import os, sys
print("sys.version:{}".format(sys.version))

sys.path.append("./Quaternions/")
sys.path.append("./ximu_python_library/")

import numpy as np
import math
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, bode

import xIMUdataClass
import AHRS
from QuaternionsCALC import *

# select dataset

filePath = './Datasets/straightLine_CalInertialAndMag.csv'
startTime = 6
stopTime = 26
#stopTime = 60

# Import data

samplePeriod = 1.0/256.0
xIMUdata = xIMUdataClass.xIMUdata(filePath, samplePeriod)
time = xIMUdata.Time
gyrX = xIMUdata.Gyroscope_X
gyrY = xIMUdata.Gyroscope_Y
gyrZ = xIMUdata.Gyroscope_Z
accX = xIMUdata.Accelerometer_X
accY = xIMUdata.Accelerometer_Y
accZ = xIMUdata.Accelerometer_Z

#print("time:{}".format(time))

# Manually frame data
idxStart = -1
idxEnd = -1
for i in range(len(time)):
	if time[i] < startTime:
		idxStart = i
	if time[i] <  stopTime:
		idxEnd = i
print("idxStart:{}, idxEnd:{}".format(idxStart, idxEnd))
time = np.array(time[idxStart:idxEnd])
gyrX = np.array(gyrX[idxStart:idxEnd])
gyrY = np.array(gyrY[idxStart:idxEnd])
gyrZ = np.array(gyrZ[idxStart:idxEnd])
accX = np.array(accX[idxStart:idxEnd])
accY = np.array(accY[idxStart:idxEnd])
accZ = np.array(accZ[idxStart:idxEnd])

# Detect stationary periods
# Compute accelerometer magnitude
acc_mag = np.sqrt(accX*accX + accY*accY + accZ*accZ);
#print("acc_mag:{}".format(acc_mag))
# HP filter accelerometer data
filtCutOff = 0.001;
b, a = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high')
acc_magFilt = lfilter(b, a, acc_mag)

# Compute absolute value
acc_magFilt = np.abs(acc_magFilt)

# LP filter accelerometer data
filtCutOff = 5
b, a = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low')
acc_magFilt = lfilter(b, a, acc_magFilt)
#print("acc_magFilt:{}".format(acc_magFilt))

# Threshold detection
stationary = np.less(acc_magFilt, 1.05)
print("stationary:{}".format(stationary))
'''
for i in range(len(stationary)):
	if stationary[i] == False:
		print i
'''
# Plot data raw sensor data and stationary periods
fig = plt.figure(figsize=[15,8])
ax = fig.add_subplot(211)
ax.plot(time, gyrX, 'r', label='X')
ax.plot(time, gyrY, 'g', label='Y')
ax.plot(time, gyrZ, 'b', label='Z')
ax.set_title('Gyroscope')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angular velocity (^\circ/s)')
ax.legend(loc='upper left')
ax = fig.add_subplot(212)
ax.plot(time, accX, 'r', label='X')
ax.plot(time, accY, 'g', label='Y')
ax.plot(time, accZ, 'b', label='Z')
ax.plot(time, acc_magFilt, 'm', label='Filtered')
ax.plot(time, stationary, 'k', label='Stationary', linewidth=2)
ax.set_title('Accelerometer')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Acceleration (g)')
ax.legend(loc='upper left')


# Compute orientation
quat = np.zeros((len(time), 4))
AHRSalgorithm = AHRS.AHRS(SamplePeriod=1.0/256.0, Kp=1, Ki=0, KpInit=1)

# Initial convergence
initPeriod = time[0] + 2.0;
for i in range(len(time)):
	#print("time[i]:{}".format(time[i]))
	if time[i] < initPeriod:
		idxEnd = i
#print("idxEnd:{}".format(idxEnd))
for i in range(2000):
	AHRSalgorithm.UpdateIMU(np.array([0,0,0]), np.array([np.mean(accX[0:idxEnd]),np.mean(accY[0:idxEnd]),np.mean(accZ[0:idxEnd])]))

# For all data
for t in range(len(time)):
	if stationary[t] == True:
		AHRSalgorithm.Kp = 0.5
	else:
		AHRSalgorithm.Kp = 0
	AHRSalgorithm.UpdateIMU(np.deg2rad([gyrX[t], gyrY[t], gyrZ[t]]),
		                    [accX[t], accY[t], accZ[t]])
	quat[t,:] = AHRSalgorithm.Quaternion

# Compute translational accelerations
# Rotate body accelerations to Earth frame
test = np.column_stack((accX, accY, accZ))
print("test:{}".format(test))
print("quat:{}".format(quat))
print("quaternConj(quat):{}".format(quaternConj(quat)))
acc = quaternRotate(np.column_stack((accX, accY, accZ)), quaternConj(quat));
print("acc:{}".format(acc))

# test only
#acc = np.column_stack((accX, accY, accZ))

# Convert acceleration measurements to m/s/s
acc = acc * 9.81

# Plot translational accelerations
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(time, acc[:,0], 'r', label='X')
ax.plot(time, acc[:,1], 'g', label='Y')
ax.plot(time, acc[:,2], 'b', label='Z')
ax.set_title('Acceleration')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Acceleration (m/s/s)')
ax.legend(loc='upper left')

# Compute translational velocities

acc[:,2] = acc[:,2] - 9.81;

# Integrate acceleration to yield velocity

vel = np.zeros_like(acc)
for t in range(1,len(vel)):
	vel[t,:] = vel[t-1,:] + acc[t,:] * samplePeriod
	if stationary[t] == True:
		vel[t,:] = [0, 0, 0] # force zero velocity when foot stationary

# Plot translational velocity
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(time, vel[:,0], 'r', label='X')
ax.plot(time, vel[:,1], 'g', label='Y')
ax.plot(time, vel[:,2], 'b', label='Z')
ax.set_title('Velocity')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Velocity (m/s)')
ax.legend(loc='upper left')

# Compute translational position

# Integrate velocity to yield position
pos = np.zeros_like(vel)
for t in range(1,len(pos)):
	pos[t,:] = pos[t-1,:] + vel[t,:] * samplePeriod # integrate velocity to yield position

# Plot translational position
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(time, pos[:,0], 'r', label='X')
ax.plot(time, pos[:,1], 'g', label='Y')
ax.plot(time, pos[:,2], 'b', label='Z')
ax.set_title('Position')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (m)')
ax.legend(loc='upper left')

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(pos[:,0], pos[:,1], pos[:,2], label='3D foot position')
ax.set_ylim(-5,5)
ax.set_zlim(-5,5)
ax.legend()



def update_lines(num, dataLines, lines) :
    for line, data in zip(lines, dataLines) :
        line.set_data(data[0:2, num-1:num])
        line.set_3d_properties(data[2,num-1:num])
    return lines

fig = plt.figure()
ax = p3.Axes3D(fig)

n = len(time)
data = [np.vstack((pos[:,0],pos[:,1],pos[:,2]))]
print("pos:{}".format(pos))
print("data:{}".format(data))

lines = [ax.plot(data[0][0,0:1], data[0][1,0:1], data[0][2,0:1], 'o')[0]]


# Setthe axes properties
ax.set_xlim3d([-10.0, 2.0])
ax.set_xlabel('X')

ax.set_ylim3d([-5.0, 5.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-5.0, 5.0])
ax.set_zlabel('Z')

ax.set_title('foot position animation')

# Creating the Animation object
ani = animation.FuncAnimation(fig, update_lines, n, fargs=(data, lines),
                              interval=1, blit=False)

plt.show()


















