from __future__ import print_function
import os, sys

import rosbag
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
from scipy.signal import butter, lfilter, filtfilt, bode

import AHRS
from QuaternionsCALC import *
from RosBagExtractor import RosBagExtractor, write_pose_pc_into_bag

"""
filePath = './Datasets/straightLine_CalInertialAndMag.csv'
startTime = 6
stopTime = 26

#filePath = './Datasets/spiralStairs_CalInertialAndMag.csv';
#startTime = 4;
#stopTime = 47;
"""

filePath = "/data/DATASETS/IMU/6_HorizontalPutOnDesk_1gACC.bag"
filePath = "/data/DATASETS/IMU/9_HorizontalHand_1gACC.bag"
#filePath = "/data/DATASETS/IMU/10_VerticalPutOnDesk_1gACC.bag"

#filePath = "/data/DATASETS/IMU/15_horizontalPotOnDesk_static.bag"
# filePath = "/data/DATASETS/IMU/16_HorizontalSquare.bag"
filePath = "/data/DATASETS/IMU/17_VerticalSquare.bag"
# filePath = "/data/DATASETS/IMU/18_VerticalSquare_slower.bag"
# filePath = "/data/DATASETS/IMU/19_VerticalSquare_anchor.bag"
# filePath = "/data/DATASETS/IMU/20_VerticalSquare_more_slower.bag"
filePath = "/data/DATASETS/IMU/21_VerticalSquare_more_step.bag"


#filePath = "/data/DATASETS/IMU/22_VerticalSquare_free_rotate.bag"
#filePath = "/data/DATASETS/IMU/23_Horizontal_Desk_Hand_Desk_static.bag"
filePath = "/data/DATASETS/IMU/24_Horizontal_StraightLine_DeskFast.bag"
#filePath = "/data/DATASETS/IMU/25_Horizontal_StraightLineBack_DeskFaster.bag"


filePath = "/data/DATASETS/IMU_CAM/1_room_DeskFast.bag"



if len(sys.argv) == 2:
    filePath = sys.argv[1]

# intrinsic of imu
# /data/DATASETS/IMU/imu_tk/YIS100-U-DK-horizontal-imu_acc.calib
acc_misalignment = np.array([
    1, 0.00714771, 0.000162677,
    -0.00784456, 1, -0.000988988,
    -0.00304139, 0.000783805, 1
]).reshape([3, 3])
acc_scale = np.array([0.996896, 0.998036, 0.996834])
acc_bias = np.array([0.588407, 0.0300896, 0.099099])


Gravity = 9.8
# Import data
extractor = RosBagExtractor(filePath)
print(extractor.rosbag_path)
ts, acc, gyr = extractor.read_imu("/imu/data", "g", "deg/s")
ts_, _, ori = extractor.read_pose("/imu/pose")

acc_mean_1000 = 9.8 * acc[:1000].mean(axis=0)
print("before calib: {}, {}".format(acc_mean_1000, np.linalg.norm(acc_mean_1000)))
acc = np.matmul(acc_misalignment, (acc_scale * (9.8 * acc - acc_bias)).T).T
acc_mean_1000 = acc[:1000].mean(axis=0)
print("before calib: {}, {}".format(acc_mean_1000, np.linalg.norm(acc_mean_1000)))
acc = acc / 9.8


samplePeriod_ = (ts[-1] - ts[0]) / (len(ts) - 1)
samplePeriod = round(samplePeriod_, 4)
print("read {} imu data with period={}({})".format(len(ts), samplePeriod, samplePeriod_))

time = ts
gyrX, gyrY, gyrZ = gyr[:, 0], gyr[:, 1], gyr[:, 2]
accX, accY, accZ = acc[:, 0], acc[:, 1], acc[:, 2]

# Detect stationary periods
# Compute accelerometer magnitude
acc_mag = np.linalg.norm(acc, axis=1);
#print("acc_mag:{}".format(acc_mag))
# HP filter accelerometer data
filtCutOff = 0.001;
b, a = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high')
#acc_magFilt1 = lfilter(b, a, acc_mag)
acc_magFilt1 = filtfilt(b, a, acc_mag)

# Compute absolute value
acc_magFilt1 = np.abs(acc_magFilt1)

# LP filter accelerometer data
filtCutOff = 5
b, a = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low')
#acc_magFilt = lfilter(b, a, acc_magFilt1)
acc_magFilt = filtfilt(b, a, acc_magFilt1)
#print("acc_magFilt:{}".format(acc_magFilt))

# Threshold detection
fig = plt.figure(figsize=[15,8])
ax = fig.add_subplot(311)
ax.plot(acc_mag)
ax = fig.add_subplot(312)
ax.plot(acc_magFilt1)
ax = fig.add_subplot(313)
ax.plot(acc_magFilt)
plt.show()
#exit(-1)

stationary = np.less(acc_magFilt, 0.050)
#print("stationary:{}".format(stationary))

print("stationary rate:{}".format(stationary.sum() / len(stationary)))
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
AHRSalgorithm = AHRS.AHRS(SamplePeriod=samplePeriod, Kp=1, Ki=0, KpInit=1)

# Initial convergence
initPeriod = time[0] + 5.0;
idxEnd = 0
for i in range(len(time)):
    #print("time[i]:{}".format(time[i]))
    if time[i] < initPeriod:
        idxEnd = i
print("idxEnd:{}".format(idxEnd))

Gravity_obs = Gravity * np.mean(acc_mag[:1000])
print("Gravity obs: %f" % Gravity_obs)


for _ in range(3):
    ret = True
    for i in range(2000):
        ret = AHRSalgorithm.UpdateIMU(np.array([0,0,0]), np.array([np.mean(accX[0:idxEnd]),np.mean(accY[0:idxEnd]),np.mean(accZ[0:idxEnd])]))
    if (not ret):
        print("initial convergence failed at line: %d" % _)
    print("AHRSalgorithm.q", AHRSalgorithm.q)

# For all data
for t in range(len(time)):
    if stationary[t] == True:
        AHRSalgorithm.Kp = 0.5
    else:
        AHRSalgorithm.Kp = 0
    ret = AHRSalgorithm.UpdateIMU(np.deg2rad([gyrX[t], gyrY[t], gyrZ[t]]),
                            [accX[t], accY[t], accZ[t]])
    if (not ret):
        print("update all data failed at line: %d" % t, end="\r")
    quat[t,:] = AHRSalgorithm.Quaternion


print("")

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
acc = acc * Gravity

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

acc[:,2] = acc[:,2] - Gravity;



stationary_diff = stationary.astype(int)[1:] - stationary.astype(int)[:-1] 
moving_begin, moving_end = [], []
for i, v in enumerate(stationary_diff):
    if v == -1:
        moving_begin.append(i)
    elif v == 1:
        moving_end.append(i)

# Integrate acceleration to yield velocity
vel = np.zeros_like(acc)
for t in range(1,len(vel)):
    vel[t,:] = vel[t-1,:] + acc[t,:] * samplePeriod
    if t in moving_end:
        print("at {}, vel: {}".format(t, vel[t]))
    if stationary[t] == True:
        vel[t,:] = [0, 0, 0] # force zero velocity when foot stationary

# Compute integral drift during non-stationary periods


velDrift = np.zeros(vel.shape)
Delta_POS = np.zeros([3])
for b, e in zip(moving_begin, moving_end):
    driftRate = (vel[e] - vel[b]) / (e - b)
    delta_pos = 0.5 * (vel[e]-vel[b]) *(e-b)*samplePeriod
    print("vel drift from {} to {}: {}, delta_pos: {}".format(b, e, vel[e]-vel[b], delta_pos))
    Delta_POS += delta_pos
    drift = np.arange(e - b + 1).reshape([-1, 1]).repeat(3, axis=1) * driftRate
    velDrift[b: e+1] = drift

print("total delta_pos: {}".format(Delta_POS))
vel = vel - velDrift

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


print("end_pos - start_pos: {}".format(pos[-1] - pos[0]))

print("length time: %d, acc: %d, vel: %d, pos: %d, ori: %d, quat: %d" % 
    (len(time), len(acc), len(vel), len(pos), len(ori), len(quat)))

print(time[:5]) 
## write data into rosbag
ts_pc_lst, pc_lst = extractor.read_pointcloud("/camera_argus100/publish_point_cloud")

# get corresponding pose of IMU
output_lst = []
cur_pc_idx = 0
for i in range(len(time)):
    if cur_pc_idx >= len(pc_lst):
        break
    pc_time = ts_pc_lst[cur_pc_idx]
    pc = pc_lst[cur_pc_idx]
    if pc_time < time[i] - 0.5 * samplePeriod:
        print("WARNING: drop pointcloud: %d" % cur_pc_idx)
    elif pc_time < time[i] + 0.5 * samplePeriod: 
        #output_lst.append([time[i], pos[i], ori[i], pc_lst[cur_pc_idx]])
        output_lst.append([time[i], pos[i], quat[i], pc_lst[cur_pc_idx]])
        cur_pc_idx += 1

del pc_lst

# get pose and pointcloud at static 
output_lst_lst = []
cur_pc_idx = 0
for b, e in zip([0,] + moving_end[:-1], moving_begin):
    tmp_lst = []
    if(cur_pc_idx >= len(output_lst)):
        break
    while(cur_pc_idx < len(output_lst)):
        pc_time = output_lst[cur_pc_idx][0]
        if pc_time <= time[b]:
            cur_pc_idx += 1
        elif pc_time < time[e]:
            tmp_lst.append(output_lst[cur_pc_idx])
            cur_pc_idx += 1
        else:
            if len(tmp_lst)>0:
                output_lst_lst.append(tmp_lst)
                tmp_lst = []
            break
    else:
        if len(tmp_lst) > 0:
            output_lst_lst.append(tmp_lst)
            tmp_lst = []

output_lst = []
for tmp_lst in output_lst_lst:
    mid_idx = len(tmp_lst) // 2
    output_lst.append(tmp_lst[mid_idx])

write_pose_pc_into_bag(filePath.replace(".bag", "_output.bag"), output_lst)






# Plot translational position
#displacement = np.linalg.norm(pos, axis=1)
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(time, pos[:,0], 'r', label='X')
ax.plot(time, pos[:,1], 'g', label='Y')
ax.plot(time, pos[:,2], 'b', label='Z')
#ax.plot(time, displacement, 'c', label='D')
ax.set_title('Position')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (m)')
ax.legend(loc='upper left')

b = 1
xlim = [-b, b]
ylim = [-b, b]
zlim = [-b, b]

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(pos[:,0], pos[:,1], pos[:,2], label='3D foot position')
ax.set_xlim(*xlim)
ax.set_ylim(*ylim)
ax.set_zlim(*zlim)
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
ax.set_xlim3d(xlim)
ax.set_xlabel('X')
ax.set_ylim3d(ylim)
ax.set_ylabel('Y')
ax.set_zlim3d(ylim)
ax.set_zlabel('Z')

ax.set_title('foot position animation')

# Creating the Animation object
ani = animation.FuncAnimation(fig, update_lines, n, fargs=(data, lines),
                              interval=1, blit=False)

plt.show()


