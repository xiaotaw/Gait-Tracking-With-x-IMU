import roslib
import rosbag
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import numpy as np
import pandas as pd
import sys

class RosBagExtractor(object):
  def __init__(self, rosbag_path):
    self.rosbag_path = rosbag_path
    self.supported_ret_acc_unit = ["m/s^2", "g"]
    self.supported_ret_gyr_unit = ["rad/s", "deg/s"]

  def read_imu(self, imu_topic_name, ret_acc_unit="m/s^2", ret_gyr_unit="rad/s", gravity=9.8):
    # deal with units
    if ret_acc_unit not in self.supported_ret_acc_unit:
      print("{} not in supported_ret_acc_unit: {}, keep unit unchanged".format(
        ret_acc_unit, self.supported_ret_acc_unit))
    if ret_gyr_unit not in self.supported_ret_gyr_unit:
          print("{} not in supported_ret_gyr_unit: {}, keep unit unchanged".format(
        ret_gyr_unit, self.supported_ret_gyr_unit))
    # read data
    ts_lst, acc_lst, gyr_lst = [], [], []
    with rosbag.Bag(self.rosbag_path, 'r') as bag:
      for topic, msg, t in bag.read_messages():
        if topic == imu_topic_name:
          ts_lst.append(msg.header.stamp.to_sec())
          acc_lst.append([msg.linear_acceleration.x, 
                          msg.linear_acceleration.y, 
                          msg.linear_acceleration.z])
          gyr_lst.append([msg.angular_velocity.x, 
                          msg.angular_velocity.y, 
                          msg.angular_velocity.z])
    ts = np.array(ts_lst, np.float64)
    acc = np.array(acc_lst, np.float32)
    gyr = np.array(gyr_lst, np.float32)
    # unit transform
    if ret_acc_unit == "g":
      acc /= gravity if gravity else 9.8
    if ret_gyr_unit == "deg/s":
      gyr = gyr / 3.1415926 * 180
    print(ts.shape, acc.shape, gyr.shape, acc[:100].mean(axis=0))
    return ts, acc, gyr

  def read_pose(self, pose_topic_name):
    ts_lst, pose_lst, ori_lst = [], [], []
    with rosbag.Bag(self.rosbag_path, 'r') as bag:
      for topic, msg, t in bag.read_messages():
        if topic == pose_topic_name:
          ts_lst.append(msg.header.stamp.to_sec())
          pose_lst.append([msg.pose.position.x, 
                           msg.pose.position.y, 
                           msg.pose.position.z])
          ori_lst.append([msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w])
    ts = np.array(ts_lst)
    pose = np.array(pose_lst)
    ori = np.array(ori_lst)
    return ts, pose, ori

  def read_pointcloud(self, pointcloud_topic_name):
    ts_lst, pc_lst = [], []
    with rosbag.Bag(self.rosbag_path, 'r') as bag:
      for topic, msg, t, in bag.read_messages():
        if topic == pointcloud_topic_name:
          ts_lst.append(msg.header.stamp.to_sec())
          pc_lst.append(msg)
    return ts_lst, pc_lst

def write_pose_pc_into_bag(filePath, output_lst):
  with rosbag.Bag(filePath, "w") as bag:
      for t, pos, ori, pc in output_lst:
          pose = PoseStamped()
          pose.header.frame_id = "YIS100"
          pose.header.stamp = pc.header.stamp
          pose.pose.position.x = pos[0]
          pose.pose.position.y = pos[1]
          pose.pose.position.z = pos[2]
          pose.pose.orientation.x = ori[0]
          pose.pose.orientation.y = ori[1]
          pose.pose.orientation.z = ori[2]
          pose.pose.orientation.w = ori[3]
          bag.write("/estimated/pose", pose)
          bag.write("/camera_argus100/publish_point_cloud", pc)


def extract_imu_to_csv(input_bag, output_csv, imu_topic):
  extractor = RosBagExtractor(input_bag)
  columns = ["timestamp","Gyroscope_X_deg_s","Gyroscope_Y_deg_s","Gyroscope_Z_deg_s",
                         "Accelerometer_X_g","Accelerometer_Y_g","Accelerometer_Z_g",
                         "Magnetometer_X_G","Magnetometer_Y_G","Magnetometer_Z_G"] 
  ts, gyr, acc = extractor.read_imu(imu_topic, ret_acc_unit="g", ret_gyr_unit="deg/s")
  mag_data = np.zeros([len(ts), 3], dtype=np.float64)
  data = np.hstack([ts.reshape([-1, 1]), gyr, acc, mag_data])
  df = pd.DataFrame(data, columns=columns)
  df.to_csv(output_csv, index=False)

def test_extractor(input_bag, imu_topic, pose_topic):
  extractor = RosBagExtractor(input_bag)  
  imu = extractor.read_imu(imu_topic)
  pose = extractor.read_pose(pose_topic)
  print("#imu data: {}".format(len(imu[0])))
  print("#pose data: {}".format(len(pose[0])))

if __name__ == "__main__":
  if len(sys.argv) == 1:
    input_bag = "imu_zupt.bag"
    output_csv = "imu_zupt.csv"
    imu_topic = "/camera/imu"
  elif len(sys.argv) == 4:
    input_bag = sys.argv[1]
    output_csv = sys.argv[2]
    imu_topic = sys.argv[3]
  else:
    print("python ImuExtractor.py input_bag output_csv imu_topic")
    exit(-1)

  #extract_imu_to_csv(input_bag, output_csv, imu_topic)

  test_extractor("/data/DATASETS/IMU/14.bag", "/imu/data", "/imu/pose")
