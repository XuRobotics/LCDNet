import pypcd
import rospy
import tf
from sensor_msgs.msg import _PointCloud
from sensor_msgs import point_cloud2 as pc2
import rosbag

import math
import numpy as np
import struct
import os

BAG_FILEPATH = '../pennovation_dataset/LCDNet-1st-parking-lot-falcon-xmas-slam-pennovation_2023-10-20-13-07-35.bag'
SEMANTIC_TOPIC = '/os_node/segmented_point_cloud_organized' # ignored
LIDAR_TOPIC = '/cloud_registered_body'
POSE_TOPIC = '/Odometry'
SAMPLE_DIST = 2.0

start_time = 0
start_pose = None
prev_pose = None
pose_init = False
pose_timestamp = 0
sample_lidar = False
sample_intensity = False

lidar_buffer = []
semantic_buffer = []
lidar_timestamp = 0

pose_msgs = {}
lidar_msgs = {}
semantic_msgs = {}

# print something
print("Converting bag to KITTI format...")


# read bag once and sample poses first
bag = rosbag.Bag(BAG_FILEPATH)
for topic, msg, t in bag.read_messages(topics=[POSE_TOPIC]):
    timestamp = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
    if topic == POSE_TOPIC:
        
        if not pose_init:
            start_time = int(msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs)
            start_pose = msg.pose.pose
            prev_pose = msg.pose.pose
            pose_init = True
            continue
        # sample based on pose
        curr_x, curr_y, curr_z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        prev_x, prev_y, prev_z = prev_pose.position.x, prev_pose.position.y, prev_pose.position.z
        dist = math.sqrt(pow((curr_x - prev_x),2) + pow((curr_y - prev_y), 2) + pow((curr_z - prev_z), 2))
        # print(dist)
        if dist > SAMPLE_DIST:
            print("Sampled pose at timestamp: ", timestamp)
            print("Pose: ", msg.pose.pose)
            prev_pose = msg.pose.pose
            timestamp = round(timestamp, -7) # round to nearest millisecond
            pose_msgs[timestamp] = msg.pose

# read bag again to retrieve corresponding lidar data
for topic, msg, t in bag.read_messages(topics=[LIDAR_TOPIC, SEMANTIC_TOPIC]):
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
        if topic == SEMANTIC_TOPIC:
            semantic_msgs[timestamp] = msg
    
# print total number of poses and lidar messages sampled
print("Total poses sampled: ", len(pose_msgs))
print("Total lidar messages sampled: ", len(lidar_msgs))    
print("Total semantic messages sampled: ", len(semantic_msgs))

t = tf.TransformerROS()


if not os.path.exists('output/velodyne'):
    os.makedirs('output/velodyne')
    print("Created output/velodyne directory")
if not os.path.exists('output'):
    os.makedirs('output')
    print("Created output directory")

pose_file = open('output/poses.txt', 'w')
times_file = open('output/times.txt', 'w')
for timestamp in pose_msgs:
    # semantics
    # semantic_msg = semantic_msgs[timestamp]
    # pc = pc2.read_points(semantic_msg, skip_nans=True, field_names=None)
    # labels = []
    # for p in pc:
    #     labels.append([p[3]])
    
    # x,y,z,label
    lidar_msg = lidar_msgs[timestamp]
    pc = pc2.read_points(lidar_msg, skip_nans=True, field_names=None)
    xyz = []
    # label = []
    for p in pc:
        xyz.append([p[0], p[1], p[2], p[3]])
        # label.append(p[3])

    # save scan and label
    xyz = np.array(xyz)        
    # intensity = np.array(intensity)
    # print(xyz.shape)
    xyz.tofile('output/velodyne/' + str(round(timestamp, -7)) + '.bin')
    # print(intensity.shape)
    # xyzi = np.concatenate((xyz, intensity), axis=0)
    # print(xyzi.shape)
    # label = np.array(label)
    
    # save pose
    pose_msg = pose_msgs[timestamp]
    # print(pose_msg)
    kitti_pose_arr = []
    tx = (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
    qt = (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)
    tf_mat = t.fromTranslationRotation(translation=tx, rotation=qt)
    
    pose_file.write(' '.join(map(str, tf_mat.flatten()[:-4])))
    pose_file.write('\n')
    times_file.write(str(timestamp - start_time) + '\n')
    # np.savetxt('poses.txt', tf_mat.flatten(), delimiter=' ')
    

