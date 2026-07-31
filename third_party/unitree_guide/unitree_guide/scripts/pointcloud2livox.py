#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@brief: 将 /scan (sensor_msgs/PointCloud) 变换到 odom 坐标系后发布为 PointCloud2
@Editor: CJH + 修改完善版
@Date: 2025-10-22 → 2025-11-22
"""

import tf
import rospy
import struct
import numpy as np
from threading import Lock

from sensor_msgs.msg import PointCloud, PointCloud2,PointField
# from unitree_guide.msg import CustomMsg, CustomPoint
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry

# 定义发布出去的点云所在的坐标系名字
# [极其关键]：这里硬编码为了 "odom"，这就是为什么 grid_map 收到的消息的 frame_id 是 "odom"

# 2026.05.08 yzk zhushi
# SENSOR_FRAME = "odom"
# 2026.05.08 yzk zhushi

ODOM_TOPIC = "/Odometry_gazebo"

# 2026.05.08 yzk
SENSOR_FRAME = "laser_livox"
BODY_FRAME = "base"
GLOBAL_FRAME = "world"
# 2026.05.08 yzk

# 线程锁，防止在处理点云时，里程计数据突然更新导致数据撕裂
m_buf = Lock()
latest_odom = None
latest_odom_time = None

'''
# 辅助函数：解析 PointCloud2 的数据结构
def _get_struct_fmt(pointcloud2):
    # 根据PointCloud2 的 field 类型，生成 Python struct 解包所需的格式字符串
    fmt = ''
    for field in pointcloud2.fields:
        if field.datatype == PointField.FLOAT32:
            fmt += 'f'
        elif field.datatype == PointField.UINT8:
            fmt += 'B'
        elif field.datatype == PointField.INT8:
            fmt += 'b'
        elif field.datatype == PointField.UINT16:
            fmt += 'H'
        elif field.datatype == PointField.INT16:
            fmt += 'h'
        elif field.datatype == PointField.UINT32:
            fmt += 'I'
        elif field.datatype == PointField.INT32:
            fmt += 'i'
        else:
            rospy.logwarn("Unsupported field type: %d", field.datatype)
    return fmt


def pointcloud2_to_custommsg(pointcloud2):
    # 将标准的 ROS PointCloud2 转换为 Livox 雷达专属的 CustomMsg 格式
    # 通常像 FAST-LIO 这种特定的紧耦合 SLAM 算法，要求输入这种带有精确单点时间戳(offset_time)的自定义消息
    custom_msg = CustomMsg()
    custom_msg.header = pointcloud2.header
    custom_msg.timebase = rospy.Time.now().to_nsec()        # 记录这一帧的基础时间
    custom_msg.point_num = pointcloud2.width
    custom_msg.lidar_id = 1                                 # 假设雷达ID为1
    custom_msg.rsvd = [0, 0, 0]                             # 保留位

    # 解析二进制点云数据
    fmt = _get_struct_fmt(pointcloud2)
    for i in range(0, len(pointcloud2.data), pointcloud2.point_step):
        point_data = pointcloud2.data[i:i + pointcloud2.point_step]
        x, y, z = struct.unpack(fmt, point_data)

        custom_point = CustomPoint()
        # 计算每个点相对于 timebase 的纳秒级时间偏移量 (这里是近似模拟)
        custom_point.offset_time = rospy.Time.now().to_nsec() - custom_msg.timebase
        custom_point.x = x
        custom_point.y = y
        custom_point.z = z
        custom_point.tag = 0
        custom_point.line = 0

        custom_msg.points.append(custom_point)

    return custom_msg
'''

# 核心空间几何处理函数
def rotate_pointcloud_y(points, theta):
    # 绕 Y 轴旋转点云 (Pitch 俯仰角)
    # 注意：目前在使用时传入的 theta=0，实际上没有起作用
    # 这是因为后面使用了 TF 树去查那个精确的 45度角（0.785弧度），所以不需要在这里硬编码旋转
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    R_y = np.array([
        [ cos_t, 0.0,  sin_t],
        [ 0.0,   1.0,  0.0 ],
        [-sin_t, 0.0,  cos_t]
    ])
    points_array = np.array(points, dtype = np.float32)
    rotated = (R_y @ points_array.T).T
    return rotated.tolist()

def odom_callback(odom_msg):
    # 里程计回调：实时更新机器狗在虚拟世界中的绝对位置
    global latest_odom, latest_odom_time

    # 2026.05.08 yzk
    global tf_broadcaster
    # 2026.05.08 yzk

    with m_buf:
        latest_odom = odom_msg
        latest_odom_time = odom_msg.header.stamp

        # 2026.05.08 yzk
        # 提取 P3D 插件发出来的真值位置和姿态
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation

        # 核心：发布 world -> base 的坐标变换到 TF 树
        tf_broadcaster.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            odom_msg.header.stamp,  # 使用仿真时间戳，保证与 Gazebo 时间完全同步
            "base",            # 子节点 (child)
            "world"            # 父节点 (parent)
        )
        # 2026.05.08 yzk

def quat_to_rot_matrix(q):
    # 数学工具：四元数 (Quaternion) -> 3x3 旋转矩阵 (Rotation Matrix)
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z),   2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),       1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),       2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]
    ])

# 2026.05.08 yzk zhushi
"""
def transform_points_to_odom(points_sensor, odom_msg):
    # [最核心的函数]：将雷达点云从当前时刻的 "laser_livox" 坐标系转换到全局 "odom" 坐标系
    global tf_listener
    if odom_msg is None:
        return points_sensor

    try:
        # 1. 查 TF：获取 base 到 laser_livox 的硬件安装关系，对应robot.xacro中的设置（比如前移 20cm，低头 45 度）
        (trans_base, rot_base) = tf_listener.lookupTransform('base', 'laser_livox', rospy.Time(0))
        rot_base_matrix = tf.transformations.quaternion_matrix(rot_base)[:3, :3]

        points_np = np.array(points_sensor, dtype = np.float32)
        if points_np.size == 0:
            return []

        # 2. 将点云从当前时刻的雷达坐标系转换到当前时刻的 base 坐标系
        points_base = (rot_base_matrix @ points_np.T).T + trans_base
        
        # 3. 提当前时刻的 base 到全局 odom(也就是上电时刻的base) 的坐标变换关系
        trans = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])

        rot = quat_to_rot_matrix(odom_msg.pose.pose.orientation)

        # 4. 将当前时刻 base 坐标系下的点云转换到全局 odom 坐标系
        transformed = (rot @ points_base.T).T + trans
        return transformed.tolist()

    except Exception as e:
        rospy.logwarn("Exception in transform_points_to_odom: %s", str(e))
        # 降级处理：如果查不到 TF 外参，就假装雷达安装在狗的绝对正中心，直接乘里程计
        trans = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])
        rot = quat_to_rot_matrix(odom_msg.pose.pose.orientation)
        points_np = np.array(points_sensor, dtype = np.float32)
        if points_np.size == 0:
            return []
        transformed = (rot @ points_np.T).T + trans
        return transformed.tolist()
"""
# 2026.05.08 yzk zhushi

# 2026.05.08 yzk shiyong
def transform_points_to_odom(points_sensor, odom_msg):
    # [优化后的函数]：一次性将点云从 "laser_livox" 转换到 "world"
    global tf_listener
    if odom_msg is None:
        return points_sensor

    try:
        # 1. 查 TF：获取 base 到 laser_livox 的静态安装外参 
        (trans_lb, rot_lb) = tf_listener.lookupTransform('base', 'laser_livox', rospy.Time(0))
        
        # 2. 提里程计：获取 world 到 base 的动态位姿 [cite: 25]
        trans_bw = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])
        rot_bw = quat_to_rot_matrix(odom_msg.pose.pose.orientation)

        # 3. 复合变换矩阵计算：
        # R_total = R_world_base * R_base_lidar
        # T_total = R_world_base * T_base_lidar + T_world_base
        rot_lb_matrix = tf.transformations.quaternion_matrix(rot_lb)[:3, :3]
        
        total_rot = rot_bw @ rot_lb_matrix
        total_trans = (rot_bw @ np.array(trans_lb)) + trans_bw

        # 4. 执行单次变换
        points_np = np.array(points_sensor, dtype=np.float32)
        if points_np.size == 0:
            return []
            
        transformed = (total_rot @ points_np.T).T + total_trans
        return transformed.tolist()

    except Exception as e:
        rospy.logwarn("Exception in transform_points_to_odom: %s", str(e))
        # 降级处理：如果查不到 TF 外参，就假装雷达安装在狗的绝对正中心，直接乘里程计
        trans = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])
        rot = quat_to_rot_matrix(odom_msg.pose.pose.orientation)
        points_np = np.array(points_sensor, dtype = np.float32)
        if points_np.size == 0:
            return []
        transformed = (rot @ points_np.T).T + trans
        return transformed.tolist()
# 2026.05.08 yzk shiyong


def filter_points_by_angle(points, min_angle_deg, max_angle_deg):
    # 视场角(FOV)过滤器：砍掉太高或太低的点
    # 例如设定 min = 2.5 度，max = 60 度，则只会保留这个锥角范围内的点
    points_np = np.array(points, dtype = np.float32)
    if points_np.size == 0:
        return []
    
    # 利用三角函数计算每个点相对于雷达的垂直仰角
    distances = np.linalg.norm(points_np[:, :2], axis = 1)      # XY平面上的投影距离
    angles = np.arctan2(points_np[:, 2], distances)             # Z高度与距离的比值即为仰角
    angles_deg = np.rad2deg(angles)
    
    # 角度过滤：生成布尔掩玛，保留符合条件的点
    mask = (angles_deg >= min_angle_deg) & (angles_deg <= max_angle_deg)
    return points_np[mask].tolist()

# 2026.05.08 yzk
def voxel_downsample(points_np, voxel_size=0.1):
    # 基于 Numpy 的高效体素降采样 (等效于 PCL 的 VoxelGrid)
    if len(points_np) == 0:
        return points_np
        
    # 1. 计算每个点所在的体素三维网格索引
    voxel_coords = np.floor(points_np / voxel_size).astype(np.int32)
    
    # 2. 去重：寻找唯一的体素坐标，并保留该体素内第一个出现的点的索引
    _, unique_indices = np.unique(voxel_coords, axis=0, return_index=True)
    
    # 3. 提取出降采样后的点集
    return points_np[unique_indices]
# 2026.05.08 yzk

# 点云主处理回调函数
def mmw_handler(mmw_cloud_msg):
    global latest_odom, pub_laser_cloud,pub_laser_livox, laser_blind,min_angle,max_angle

    with m_buf:
        odom_now = latest_odom
        stamp = mmw_cloud_msg.header.stamp

    # [Step 1]：提取 Gazebo 发出的原始点云 (此时是 sensor_msgs/PointCloud 老格式)
    header = rospy.Header()
    header.stamp = stamp
    header.frame_id = GLOBAL_FRAME
    x = np.fromiter((p.x for p in mmw_cloud_msg.points), dtype = np.float32)
    y = np.fromiter((p.y for p in mmw_cloud_msg.points), dtype = np.float32)
    z = np.fromiter((p.z for p in mmw_cloud_msg.points), dtype = np.float32)
    raw_points = np.column_stack((x, y, z)).tolist()

    if not raw_points:
        return

    # [Step 2]：软件层面的固定偏角旋转（当前为 0，实际靠 TF 树）
    rotated_points = rotate_pointcloud_y(raw_points, theta = 0)

    # [Step 2.5]：视场角裁剪
    angle_filtered_points = filter_points_by_angle(rotated_points, min_angle, max_angle)

    # [Step 3]：盲区过滤
    # 雷达可能会扫到狗自己的脑袋或背部，这些极近的点会影响导航，这里按距离切除
    points_np = np.array(angle_filtered_points, dtype = np.float32)
    distances = np.linalg.norm(points_np, axis = 1)     # 计算到雷达中心点的欧式距离

    # 2026.05.08 yzk zhushi
    # filtered_points = points_np[distances >= laser_blind].tolist()      # 滤掉距离雷达中心点 0.5m 以内的点
    # 2026.05.08 yzk zhushi

    # 2026.05.08 yzk shiyong
    # 获取盲区过滤后的 Numpy 数组（先不转成 list）
    filtered_points_np = points_np[distances >= laser_blind]

    # [Step 3.5]：体素网格(VoxelGrid)降采样
    down_voxel_size = 0.1
    downsampled_points_np = voxel_downsample(filtered_points_np, down_voxel_size)

    filtered_points = downsampled_points_np.tolist()
    # 2026.05.08 yzk shiyong

    # [Step 3.5]: 将“局部系”下清洗干净的点，发给 SLAM 模块
    # 注意：此时点云还没乘里程计矩阵！发出去的还是相对于雷达的局部点！
    m_buf.acquire()
    
    # 建立局部坐标系的 header
    local_header = rospy.Header()
    local_header.stamp = stamp
    local_header.frame_id = SENSOR_FRAME  # 这里使用 "laser_livox" 而不是 "world"
    
    # 直接生成并发布 PointCloud2
    cloud_msg_local = pc2.create_cloud_xyz32(local_header, filtered_points)
    pub_laser_livox.publish(cloud_msg_local)
    
    m_buf.release()

    # [Step 4]: 将点从当前时刻的雷达坐标系转换到 odom 坐标系 (一般为初始时刻的base坐标系)
    transformed_points = transform_points_to_odom(filtered_points, odom_now)

    # [Step 5]: 创建标准的 PointCloud2，并以 frame_id = "odom" 的形式发布转换后的点云
    cloud_msg = pc2.create_cloud_xyz32(header, transformed_points)
    pub_laser_cloud.publish(cloud_msg)

# ROS 节点入口
def main():
    global pub_laser_cloud, pub_laser_livox, laser_blind, min_angle, max_angle, tf_listener

    # 2026.05.08 yzk
    global tf_broadcaster
    # 2026.05.08 yzk

    rospy.init_node('pre_mmw_to_odom', anonymous = True)

    # 初始化 TF 监听器
    tf_listener = tf.TransformListener()

    # 2026.05.08 yzk
    # 初始化 TF 广播器
    tf_broadcaster = tf.TransformBroadcaster()
    # 2026.05.08 yzk

    # 读取 ROS 参数服务器上的配置
    laser_blind = rospy.get_param('~laser_blind', 0.2)      # 盲区半径，默认干掉 0.2m 内的点，但 launch 文件传入的实际参数是 0.5m
    rospy.loginfo(f"Blind range : {laser_blind} m")

    min_angle = rospy.get_param('~min_angle', 2.5)          # 俯仰角下限
    max_angle = rospy.get_param('~max_angle', 60)           # 俯仰角上限
    rospy.loginfo(f"Angle filter : {min_angle} ~ {max_angle} deg")

    # 订阅原始仿真点云和 gazebo 里程计 (当前时刻的base 到 odom)
    rospy.Subscriber('/scan', PointCloud, mmw_handler, queue_size = 10)
    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback, queue_size = 10)

    # /livox/lidar2 发送给 SLAM (雷达坐标系下的点云)
    pub_laser_livox = rospy.Publisher('/livox/point_cloud', PointCloud2, queue_size = 10)
    # /livox/Pointcloud2 发送给 Planner 或者 Exploration（odom坐标系下的点云）
    pub_laser_cloud = rospy.Publisher("/world/point_cloud", PointCloud2, queue_size = 10)

    rospy.loginfo("=== Pointcloud2livox (published in odom) STARTED ===")
    rospy.loginfo(f"Global frame: {GLOBAL_FRAME}")
    rospy.loginfo(f"Odom topic : {ODOM_TOPIC}")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass