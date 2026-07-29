#!/usr/bin/env python3
"""Bridge 3d-navi/Unitree A1 Gazebo topics to the CMU exploration stack."""

import math
import struct

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_ros


class CmuA1Bridge:
    def __init__(self):
        self.state_source = rospy.get_param("~state_source", "gazebo_model_states")
        self.model_name = rospy.get_param("~model_name", "a1_gazebo")
        self.odom_in = rospy.get_param("~odom_in", "/odom")
        self.scan_in = rospy.get_param("~scan_in", "/velodyne_points")
        self.cmd_in = rospy.get_param("~cmd_in", "/cmu_cmd_vel")
        self.cmd_out = rospy.get_param("~cmd_out", "/cmd_vel")
        self.state_out = rospy.get_param("~state_estimation_out", "/state_estimation")
        self.scan_out = rospy.get_param("~registered_scan_out", "/registered_scan")
        self.laserscan_out = rospy.get_param("~laserscan_out", "/scan")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "sensor")
        self.scan_frame = rospy.get_param("~scan_frame", "")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.tf_time_offset = rospy.get_param("~tf_time_offset", 0.02)
        self.publish_laserscan = rospy.get_param("~publish_laserscan", True)
        self.transform_scan_to_map = rospy.get_param("~transform_scan_to_map", True)
        self.scan_offset_x = rospy.get_param("~scan_offset_x", 0.12)
        self.scan_offset_y = rospy.get_param("~scan_offset_y", 0.0)
        self.scan_offset_z = rospy.get_param("~scan_offset_z", 0.19)
        self.laserscan_min_z = rospy.get_param("~laserscan_min_z", -0.2)
        self.laserscan_max_z = rospy.get_param("~laserscan_max_z", 1.0)
        self.laserscan_angle_min = rospy.get_param("~laserscan_angle_min", -math.pi)
        self.laserscan_angle_max = rospy.get_param("~laserscan_angle_max", math.pi)
        self.laserscan_angle_increment = rospy.get_param("~laserscan_angle_increment", math.radians(1.0))
        self.laserscan_range_min = rospy.get_param("~laserscan_range_min", 0.3)
        self.laserscan_range_max = rospy.get_param("~laserscan_range_max", 40.0)
        self.cmd_scale_linear = rospy.get_param("~cmd_scale_linear", 1.0)
        self.cmd_scale_angular = rospy.get_param("~cmd_scale_angular", 1.0)
        self.max_linear = rospy.get_param("~max_linear", 1.0)
        self.max_angular = rospy.get_param("~max_angular", 1.5)
        self.last_odom = None
        self.last_tf_stamp = None
        self.scan_fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]

        self.state_pub = rospy.Publisher(self.state_out, Odometry, queue_size=10)
        self.scan_pub = rospy.Publisher(self.scan_out, PointCloud2, queue_size=5)
        self.laserscan_pub = rospy.Publisher(self.laserscan_out, LaserScan, queue_size=5)
        self.cmd_pub = rospy.Publisher(self.cmd_out, Twist, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        if self.state_source == "odom":
            rospy.Subscriber(self.odom_in, Odometry, self._odom_cb, queue_size=10)
            rospy.loginfo("Using odometry source %s", self.odom_in)
        elif self.state_source == "gazebo_model_states":
            rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=10)
            rospy.loginfo("Using Gazebo model state source for model %s", self.model_name)
        else:
            raise rospy.ROSException("Unsupported ~state_source: %s" % self.state_source)

        rospy.Subscriber(self.scan_in, PointCloud2, self._scan_cb, queue_size=5)
        rospy.Subscriber(self.cmd_in, TwistStamped, self._cmd_cb, queue_size=10)

    def _odom_cb(self, msg):
        msg.header.frame_id = msg.header.frame_id or self.map_frame
        msg.child_frame_id = msg.child_frame_id or self.base_frame
        self.state_pub.publish(msg)
        self.last_odom = msg
        self._publish_tf(msg)

    def _model_states_cb(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            if not rospy.get_param("~suppress_missing_model_warning", False):
                rospy.logwarn_throttle(5.0, "Model %s not found in /gazebo/model_states", self.model_name)
            return

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = msg.pose[index]
        odom.twist.twist = msg.twist[index]
        self.state_pub.publish(odom)
        self.last_odom = odom
        self._publish_tf(odom)

    def _scan_cb(self, msg):
        if not self.transform_scan_to_map:
            if self.scan_frame:
                msg.header.frame_id = self.scan_frame
            elif not msg.header.frame_id:
                msg.header.frame_id = self.map_frame
            self.scan_pub.publish(msg)
            return

        if self.last_odom is None:
            rospy.logwarn_throttle(5.0, "Waiting for state estimate before registering scans")
            return

        raw_points = list(self._iter_xyz_intensity(msg))
        self._publish_laserscan(msg.header.stamp, raw_points)

        pose = self.last_odom.pose.pose
        q = pose.orientation
        rotation = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        origin = pose.position
        points = []

        for raw_point in raw_points:
            x, y, z, intensity = raw_point
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if not math.isfinite(intensity):
                intensity = 0.0

            local_x = x + self.scan_offset_x
            local_y = y + self.scan_offset_y
            local_z = z + self.scan_offset_z
            map_x = origin.x + rotation[0][0] * local_x + rotation[0][1] * local_y + rotation[0][2] * local_z
            map_y = origin.y + rotation[1][0] * local_x + rotation[1][1] * local_y + rotation[1][2] * local_z
            map_z = origin.z + rotation[2][0] * local_x + rotation[2][1] * local_y + rotation[2][2] * local_z
            points.append((map_x, map_y, map_z, intensity))

        header = msg.header
        header.frame_id = self.map_frame
        registered_scan = point_cloud2.create_cloud(header, self.scan_fields, points)
        self.scan_pub.publish(registered_scan)

    def _publish_laserscan(self, stamp, raw_points):
        if not self.publish_laserscan:
            return
        if self.laserscan_angle_increment <= 0:
            rospy.logwarn_throttle(5.0, "~laserscan_angle_increment must be positive")
            return

        bin_count = int(math.ceil((self.laserscan_angle_max - self.laserscan_angle_min) /
                                  self.laserscan_angle_increment))
        if bin_count <= 0:
            rospy.logwarn_throttle(5.0, "Invalid LaserScan angle range")
            return

        ranges = [float("inf")] * bin_count
        for x, y, z, _ in raw_points:
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            local_x = x + self.scan_offset_x
            local_y = y + self.scan_offset_y
            local_z = z + self.scan_offset_z
            if local_z < self.laserscan_min_z or local_z > self.laserscan_max_z:
                continue

            distance = math.hypot(local_x, local_y)
            if distance < self.laserscan_range_min or distance > self.laserscan_range_max:
                continue

            angle = math.atan2(local_y, local_x)
            if angle < self.laserscan_angle_min or angle >= self.laserscan_angle_max:
                continue

            index = int((angle - self.laserscan_angle_min) / self.laserscan_angle_increment)
            if 0 <= index < bin_count:
                ranges[index] = min(ranges[index], distance)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = self.laserscan_angle_min
        scan.angle_max = self.laserscan_angle_min + self.laserscan_angle_increment * (bin_count - 1)
        scan.angle_increment = self.laserscan_angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.laserscan_range_min
        scan.range_max = self.laserscan_range_max
        scan.ranges = ranges
        self.laserscan_pub.publish(scan)

    @staticmethod
    def _iter_xyz_intensity(msg):
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            return

        endian = ">" if msg.is_bigendian else "<"
        data_len = len(msg.data)
        point_step = msg.point_step
        if point_step <= 0:
            return

        point_count = data_len // point_step
        x_offset = fields["x"].offset
        y_offset = fields["y"].offset
        z_offset = fields["z"].offset
        intensity_offset = fields["intensity"].offset if "intensity" in fields else None

        for index in range(point_count):
            base = index * point_step
            x = struct.unpack_from(endian + "f", msg.data, base + x_offset)[0]
            y = struct.unpack_from(endian + "f", msg.data, base + y_offset)[0]
            z = struct.unpack_from(endian + "f", msg.data, base + z_offset)[0]
            if intensity_offset is None:
                intensity = 0.0
            else:
                intensity = struct.unpack_from(endian + "f", msg.data, base + intensity_offset)[0]
            yield x, y, z, intensity

    def _cmd_cb(self, msg):
        cmd = Twist()
        cmd.linear.x = self._clamp(msg.twist.linear.x * self.cmd_scale_linear, self.max_linear)
        cmd.linear.y = self._clamp(msg.twist.linear.y * self.cmd_scale_linear, self.max_linear)
        cmd.linear.z = self._clamp(msg.twist.linear.z * self.cmd_scale_linear, self.max_linear)
        cmd.angular.x = self._clamp(msg.twist.angular.x * self.cmd_scale_angular, self.max_angular)
        cmd.angular.y = self._clamp(msg.twist.angular.y * self.cmd_scale_angular, self.max_angular)
        cmd.angular.z = self._clamp(msg.twist.angular.z * self.cmd_scale_angular, self.max_angular)
        self.cmd_pub.publish(cmd)

    def _publish_tf(self, odom):
        if not self.publish_tf:
            return
        if self.last_tf_stamp == odom.header.stamp:
            return
        self.last_tf_stamp = odom.header.stamp

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp + rospy.Duration.from_sec(self.tf_time_offset)
        transform.header.frame_id = odom.header.frame_id or self.map_frame
        transform.child_frame_id = odom.child_frame_id or self.base_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0 or math.isnan(norm):
            return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

        x /= norm
        y /= norm
        z /= norm
        w /= norm
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        return (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )

    @staticmethod
    def _clamp(value, limit):
        if limit <= 0 or math.isnan(limit):
            return value
        return max(-limit, min(limit, value))


def main():
    rospy.init_node("cmu_a1_bridge")
    CmuA1Bridge()
    rospy.spin()


if __name__ == "__main__":
    main()
