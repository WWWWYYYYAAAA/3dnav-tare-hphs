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
from std_msgs.msg import Bool
import tf2_ros


class CmuA1Bridge:
    def __init__(self):
        self.state_source = rospy.get_param("~state_source", "gazebo_model_states")
        self.model_name = rospy.get_param("~model_name", "a1_gazebo")
        self.odom_in = rospy.get_param("~odom_in", "/odom")
        self.scan_in = rospy.get_param("~scan_in", "/livox/point_cloud")
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
        self.scan_offset_x = rospy.get_param("~scan_offset_x", 0.25)
        self.scan_offset_y = rospy.get_param("~scan_offset_y", 0.0)
        self.scan_offset_z = rospy.get_param("~scan_offset_z", 0.1)
        self.scan_mount_roll = rospy.get_param("~scan_mount_roll", 0.0)
        self.scan_mount_pitch = rospy.get_param("~scan_mount_pitch", 0.785)
        self.scan_mount_yaw = rospy.get_param("~scan_mount_yaw", 0.0)
        self.scan_mount_rotation = self._rpy_to_matrix(
            self.scan_mount_roll,
            self.scan_mount_pitch,
            self.scan_mount_yaw,
        )
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
        self.command_timeout = rospy.get_param("~command_timeout", 0.5)
        self.cmd_publish_rate = rospy.get_param("~cmd_publish_rate", 20.0)
        self.startup_cmd_hold_time = rospy.get_param("~startup_cmd_hold_time", 0.0)
        self.startup_cmd_ramp_time = rospy.get_param("~startup_cmd_ramp_time", 0.0)
        self.startup_wait_for_policy = rospy.get_param("~startup_wait_for_policy", False)
        self.policy_active_topic = rospy.get_param(
            "~policy_active_topic",
            "/a1_rl_policy_driver/policy_active",
        )
        self.head_mode = str(rospy.get_param("~head_mode", "none")).lower()
        self.head_yaw_kp = rospy.get_param("~head_yaw_kp", 1.5)
        self.head_yaw_kd = rospy.get_param("~head_yaw_kd", 0.15)
        self.head_min_speed = rospy.get_param("~head_min_speed", 0.05)
        self.head_max_yaw_rate = rospy.get_param("~head_max_yaw_rate", 0.0)
        self.startup_reference_time = None
        self.policy_active_time = None
        self.head_target_yaw = None
        self.head_target_direction_sign = 0
        self.latest_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
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
        if self.startup_wait_for_policy:
            rospy.Subscriber(self.policy_active_topic, Bool, self._policy_active_cb, queue_size=1)
        period = 1.0 / max(1.0, self.cmd_publish_rate)
        self.cmd_timer = rospy.Timer(rospy.Duration.from_sec(period), self._cmd_timer_cb)

    def _odom_cb(self, msg):
        msg.header.frame_id = msg.header.frame_id or self.map_frame
        msg.child_frame_id = msg.child_frame_id or self.base_frame
        self.state_pub.publish(msg)
        self.last_odom = msg
        self._mark_startup_reference()
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
        self._mark_startup_reference()
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

            local_x, local_y, local_z = self._scan_to_base(x, y, z)
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

            local_x, local_y, local_z = self._scan_to_base(x, y, z)
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

    def _scan_to_base(self, x, y, z):
        rotation = self.scan_mount_rotation
        return (
            rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + self.scan_offset_x,
            rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + self.scan_offset_y,
            rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + self.scan_offset_z,
        )

    def _cmd_cb(self, msg):
        self.latest_cmd = self._scale_cmd(msg.twist)
        self.last_cmd_time = rospy.Time.now()
        self.cmd_pub.publish(self._output_cmd())

    def _cmd_timer_cb(self, _event):
        self.cmd_pub.publish(self._output_cmd())

    def _policy_active_cb(self, msg):
        if msg.data:
            if self.policy_active_time is None:
                self.policy_active_time = rospy.Time.now()
                rospy.loginfo("A1 policy active; holding planner command for %.2f more seconds",
                              self.startup_cmd_hold_time)
        else:
            self.policy_active_time = None
            self._clear_head_target()

    def _output_cmd(self):
        cmd = Twist()
        if self._holding_startup_cmd() or self._planner_cmd_stale():
            self._clear_head_target()
            return cmd

        ramp = self._startup_cmd_ramp()
        cmd.linear.x = ramp * self.latest_cmd.linear.x
        cmd.linear.y = ramp * self.latest_cmd.linear.y
        cmd.linear.z = ramp * self.latest_cmd.linear.z
        cmd.angular.x = ramp * self.latest_cmd.angular.x
        cmd.angular.y = ramp * self.latest_cmd.angular.y
        cmd.angular.z = ramp * self.latest_cmd.angular.z
        cmd = self._apply_head_mode(cmd)
        return cmd

    def _scale_cmd(self, twist):
        cmd = Twist()
        cmd.linear.x = self._clamp(twist.linear.x * self.cmd_scale_linear, self.max_linear)
        cmd.linear.y = self._clamp(twist.linear.y * self.cmd_scale_linear, self.max_linear)
        cmd.linear.z = self._clamp(twist.linear.z * self.cmd_scale_linear, self.max_linear)
        cmd.angular.x = self._clamp(twist.angular.x * self.cmd_scale_angular, self.max_angular)
        cmd.angular.y = self._clamp(twist.angular.y * self.cmd_scale_angular, self.max_angular)
        cmd.angular.z = self._clamp(twist.angular.z * self.cmd_scale_angular, self.max_angular)
        return cmd

    def _apply_head_mode(self, cmd):
        if self.head_mode in ("none", "off", "false", "0", ""):
            self._clear_head_target()
            return cmd
        if self.head_mode not in ("velocity", "head", "headed"):
            rospy.logwarn_throttle(5.0, "Unsupported ~head_mode '%s'; using no-head behavior", self.head_mode)
            return cmd
        if self.last_odom is None:
            return cmd

        speed = math.hypot(cmd.linear.x, cmd.linear.y)
        if speed < self.head_min_speed:
            self._clear_head_target()
            return cmd

        yaw = self._yaw_from_quaternion(self.last_odom.pose.pose.orientation)
        cmd_heading = math.atan2(cmd.linear.y, cmd.linear.x)
        direction_sign = 1 if math.cos(cmd_heading) >= 0.0 else -1
        if self.head_target_yaw is None or direction_sign != self.head_target_direction_sign:
            self.head_target_yaw = self._normalize_angle(yaw + math.atan2(cmd.linear.y, cmd.linear.x))
            self.head_target_direction_sign = direction_sign

        yaw_error = self._normalize_angle(self.head_target_yaw - yaw)

        yaw_rate = self.last_odom.twist.twist.angular.z
        max_yaw_rate = self.head_max_yaw_rate if self.head_max_yaw_rate > 0.0 else self.max_angular
        cmd.angular.z = self._clamp(self.head_yaw_kp * yaw_error - self.head_yaw_kd * yaw_rate, max_yaw_rate)

        cmd.linear.x = self._clamp(speed * math.cos(yaw_error), self.max_linear)
        cmd.linear.y = self._clamp(speed * math.sin(yaw_error), self.max_linear)
        return cmd

    def _clear_head_target(self):
        self.head_target_yaw = None
        self.head_target_direction_sign = 0

    def _planner_cmd_stale(self):
        if self.last_cmd_time == rospy.Time(0):
            return True
        if self.command_timeout <= 0.0:
            return False
        return (rospy.Time.now() - self.last_cmd_time).to_sec() > self.command_timeout

    def _holding_startup_cmd(self):
        elapsed = self._startup_elapsed()
        if elapsed is None:
            return self.startup_wait_for_policy or self.startup_cmd_hold_time > 0.0
        if self.startup_cmd_hold_time <= 0.0:
            return False
        return elapsed < self.startup_cmd_hold_time

    def _startup_cmd_ramp(self):
        if self.startup_cmd_ramp_time <= 0.0:
            return 1.0
        elapsed = self._startup_elapsed()
        if elapsed is None:
            return 0.0
        elapsed -= self.startup_cmd_hold_time
        return max(0.0, min(1.0, elapsed / self.startup_cmd_ramp_time))

    def _startup_elapsed(self):
        if self.startup_wait_for_policy:
            if self.policy_active_time is None:
                return None
            return (rospy.Time.now() - self.policy_active_time).to_sec()
        if self.startup_reference_time is None:
            return None
        return (rospy.Time.now() - self.startup_reference_time).to_sec()

    def _mark_startup_reference(self):
        if self.startup_reference_time is None:
            self.startup_reference_time = rospy.Time.now()

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
    def _rpy_to_matrix(roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

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
    def _yaw_from_quaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

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
