#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations
import argparse
import math

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None

# pose to matrix
def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )

# pointcloud to numpy array
def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc

# scan point와 map 정합 -> 변환 행렬, 피트니스 점수
def registration_at_scale(pc_scan, pc_map, initial, scale):
    # result_icp = o3d.pipelines.registration.registration_icp(
    #     voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
    #     1.0 * scale, initial,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    #     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
    # )
    result_icp = o3d.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale, initial,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=20)
    )

    return result_icp.transformation, result_icp.fitness

# 역변환 계산
def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse

# pointcloud publishing
def publish_point_cloud(publisher, header, pc):
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)

# 시야 내의 글로벌 맵 자르기
def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # 현재 스캔 원점의 위치
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 맵을 LiDAR 좌표계로 변환
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 시야 내의 맵 포인트를 추출
    if FOV > 3.14:
        # 환경 LiDAR: 거리만 필터링
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 비환경 LiDAR: 전방 시야 유지
        # FOV_FAR > x > 0 이며 각도가 FOV보다 작은 경우
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    # 시야 내의 포인트 클라우드 발행
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV



def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    # ICP를 이용한 정합
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)

    # 대략적인 정합
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 정밀한 정합
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    rospy.loginfo('Time: {}'.format(toc - tic))
    rospy.loginfo('')

     # 글로벌 로컬라이제이션이 성공했을 때만 map2odom을 업데이트
    if fitness > LOCALIZATION_TH:
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation

        # map_to_odom 발행
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        rospy.logwarn('Matched!')
        return True
    else:
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map

    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    rospy.loginfo('Global map received.')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan
    # 여기서는 fastlio가 스캔을 직접적으로 odom 좌표계로 변환한 것에 유의해야 합니다. 이는 lidar의 지역 좌표계가 아닙니다.
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time.now()
    pub_pc_in_map.publish(pc_msg)

    # PointCloud 데이터로 변환
    # fastlio에서 주어진 field가 문제가 있어 처리합니다.
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)

    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])


def thread_localization():
    global T_map_to_odom
    while True:
        # 일정 시간마다 전역 위치 추정을 수행합니다.
        rospy.sleep(1 / FREQ_LOCALIZATION)
        # TODO: 여기서 Fast lio가 발행한 스캔이 이미 odom 좌표계로 변환되었기 때문에, 각 전역 위치 추정마다 초기 해를 이전의 map2odom으로 가져올 필요가 없습니다.
        global_localization(T_map_to_odom)



if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.4
    SCAN_VOXEL_SIZE = 0.1

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 0.5

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.8

    # FOV(rad), modify this according to your LiDAR type
    FOV = 6.28319

    # The farthest distance(meters) within FOV
    FOV_FAR = 300

    rospy.init_node('fast_lio')
    rospy.loginfo('Localization Node Inited...')

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)

    # launch 파일에서 파라미터 읽기
    x = rospy.get_param('~x', 0.0)
    y = rospy.get_param('~y', 0.0)
    z = rospy.get_param('~z', 0.0)
    roll = rospy.get_param('~roll', 0.0)
    pitch = rospy.get_param('~pitch', 0.0)
    yaw = rospy.get_param('~yaw', 0.0)

    # 각도 단위를 도에서 라디안으로 변환
    yaw = math.radians(yaw)
    pitch = math.radians(pitch)
    roll = math.radians(roll)

    # 변환을 적용하여 pose 생성
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    xyz = [x, y, z]

    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
    initial_pose_msg.header.stamp = rospy.Time().now()
    initial_pose_msg.header.frame_id = 'map'
    rospy.loginfo('Initial Pose: {} {} {} {} {} {}'.format(
            x, y, z, yaw, pitch, roll))

    # initialize global map
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('map', PointCloud2))

    # initialize
    while not initialized:
        # rospy.logwarn('Waiting for initial pose....')

        # waiting initial pose
        # pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        pose_msg = initial_pose_msg
        initial_pose = pose_to_mat(pose_msg)
        if cur_scan:
            initialized = global_localization(initial_pose)
        else:
            rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    # estimate global localization (cycle)
    _thread.start_new_thread(thread_localization, ())

    rospy.spin()