import pypcd
import rospy
from sensor_msgs.msg import _PointCloud
from sensor_msgs import point_cloud2 as pc2
import rosbag
import math
import numpy as np

import struct

BAG_FILEPATH = '/mnt/c/mm_data/very-important-3-robot-loop-closure-LOOP2-ONLY-With-Segmentation-falcon_pennovation_2022-05-13-16-27-46.bag'
INTENSITY_TOPIC = '/os_node/llol_odom/sweep'
LIDAR_TOPIC = '/os_node/segmented_point_cloud_no_destagger'
POSE_TOPIC = '/os_node/llol_odom/pose'
SAMPLE_DIST = 20.0

start_pose = None
prev_pose = None
pose_init = False
pose_timestamp = 0
sample_lidar = False
sample_intensity = False

lidar_buffer = []
intensity_buffer = []
lidar_timestamp = 0

pose_msgs = {}
intensity_msgs = {}
lidar_msgs = {}

# read bag once and sample poses first
bag = rosbag.Bag(BAG_FILEPATH)
for topic, msg, t in bag.read_messages(topics=[POSE_TOPIC]):
    timestamp = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
    if topic == POSE_TOPIC:
        if not pose_init:
            start_pose = msg.pose
            prev_pose = msg.pose
            pose_init = True
            continue
        # sample based on pose
        curr_x, curr_y, curr_z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        prev_x, prev_y, prev_z = prev_pose.position.x, prev_pose.position.y, prev_pose.position.z
        dist = math.sqrt(pow((curr_x - prev_x),2) + pow((curr_y - prev_y), 2) + pow((curr_z - prev_z), 2))
        # print(dist)
        if dist > SAMPLE_DIST:
            prev_pose = msg.pose
            timestamp = round(timestamp, -7) # round to nearest millisecond
            pose_msgs[timestamp] = msg

# read bag again to retrieve corresponding lidar data
for topic, msg, t in bag.read_messages(topics=[LIDAR_TOPIC, INTENSITY_TOPIC]):
    timestamp = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
    timestamp = round(timestamp, -7)
    if timestamp in pose_msgs:
        sample = True
    else:
        sample = False
    # for pose_timestamp in pose_msgs:
    #     sample = False
    #     # 1ms soft match on timestamp since they aren't exactly aligned
    #     # this is fine since keyframe poses are significantly far apart
    #     if abs(timestamp - pose_timestamp) < 1000000:
    #         sample = True
    #         break
    if sample:
        if topic == LIDAR_TOPIC:
            lidar_msgs[timestamp] = msg
        if topic == INTENSITY_TOPIC:
            intensity_msgs[timestamp] = msg
    
print(len(pose_msgs))
print(len(lidar_msgs))
print(len(intensity_msgs))

# write kitti format files
for timestamp in pose_msgs:
    # intensity
    intensity_msg = intensity_msgs[timestamp]
    pc = pc2.read_points(intensity_msg, skip_nans=True, field_names=None)
    intensity = []
    for p in pc:
        intensity.append([p[3]])
    # x,y,z,label
    lidar_msg = lidar_msgs[timestamp]
    pc = pc2.read_points(lidar_msg, skip_nans=True, field_names=None)
    xyz = []
    label = []
    for p in pc:
        xyz.append([p[0], p[1], p[2]])
        label.append(p[3])

    xyz = np.array(xyz)        
    intensity = np.array(intensity)
    print(xyz.shape)
    print(intensity.shape)
    xyzi = np.concatenate((xyz, intensity), axis=0)
    print(xyzi.shape)
    label = np.array(label)
    
    # save scan and label
