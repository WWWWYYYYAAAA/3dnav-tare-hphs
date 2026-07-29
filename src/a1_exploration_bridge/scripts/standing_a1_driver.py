#!/usr/bin/env python3
"""Temporary A1 Gazebo driver: fixed standing pose plus cmd_vel body motion."""

import math

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Quaternion, Twist
from unitree_legged_msgs.msg import MotorCmd


class StandingA1Driver:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "a1_gazebo")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.joint_command_ns = rospy.get_param("~joint_command_ns", "/a1_gazebo")
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.command_timeout = rospy.get_param("~command_timeout", 0.5)
        self.stand_height = rospy.get_param("~stand_height", 0.38)
        self.drive_model_state = rospy.get_param("~drive_model_state", True)
        self.max_linear = rospy.get_param("~max_linear", 1.0)
        self.max_angular = rospy.get_param("~max_angular", 1.2)

        self.hip_q = rospy.get_param("~hip_q", 0.0)
        self.thigh_q = rospy.get_param("~thigh_q", 0.67)
        self.calf_q = rospy.get_param("~calf_q", -1.3)
        self.hip_kp = rospy.get_param("~hip_kp", 180.0)
        self.hip_kd = rospy.get_param("~hip_kd", 8.0)
        self.thigh_kp = rospy.get_param("~thigh_kp", 180.0)
        self.thigh_kd = rospy.get_param("~thigh_kd", 8.0)
        self.calf_kp = rospy.get_param("~calf_kp", 300.0)
        self.calf_kd = rospy.get_param("~calf_kd", 15.0)

        self.latest_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.last_step_time = None
        self.pose_initialized = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.joint_publishers = {}
        for leg in ("FL", "FR", "RL", "RR"):
            for joint in ("hip", "thigh", "calf"):
                topic = "%s/%s_%s_controller/command" % (self.joint_command_ns, leg, joint)
                self.joint_publishers[(leg, joint)] = rospy.Publisher(topic, MotorCmd, queue_size=1)

        self.set_model_state = None
        if self.drive_model_state:
            self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_cb, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=10)

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = rospy.Timer(rospy.Duration.from_sec(period), self._timer_cb)
        rospy.loginfo("Standing A1 driver active for model %s", self.model_name)

    def _cmd_vel_cb(self, msg):
        self.latest_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def _model_states_cb(self, msg):
        if self.pose_initialized:
            return
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return

        pose = msg.pose[index]
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = self._yaw_from_quaternion(pose.orientation)
        self.pose_initialized = True

    def _timer_cb(self, event):
        now = rospy.Time.now()
        self._publish_stand_pose()

        if not self.drive_model_state:
            return
        if not self.pose_initialized:
            rospy.logwarn_throttle(5.0, "Waiting for %s in /gazebo/model_states", self.model_name)
            return
        if now == rospy.Time(0):
            return

        if self.last_step_time is None:
            self.last_step_time = now
            return

        dt = (now - self.last_step_time).to_sec()
        self.last_step_time = now
        if dt <= 0.0 or dt > 0.2:
            return

        cmd = self._active_cmd(now)
        vx = self._clamp(cmd.linear.x, self.max_linear)
        vy = self._clamp(cmd.linear.y, self.max_linear)
        wz = self._clamp(cmd.angular.z, self.max_angular)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = self._normalize_angle(self.yaw + wz * dt)

        state = ModelState()
        state.model_name = self.model_name
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.stand_height
        state.pose.orientation = self._quaternion_from_yaw(self.yaw)
        state.twist.linear.x = vx
        state.twist.linear.y = vy
        state.twist.angular.z = wz
        state.reference_frame = "world"

        try:
            self.set_model_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "Failed to set A1 model state: %s", exc)

    def _active_cmd(self, now):
        if self.last_cmd_time == rospy.Time(0):
            return Twist()
        if (now - self.last_cmd_time).to_sec() > self.command_timeout:
            return Twist()
        return self.latest_cmd

    def _publish_stand_pose(self):
        for leg in ("FL", "FR", "RL", "RR"):
            self._publish_joint(leg, "hip", self.hip_q, self.hip_kp, self.hip_kd)
            self._publish_joint(leg, "thigh", self.thigh_q, self.thigh_kp, self.thigh_kd)
            self._publish_joint(leg, "calf", self.calf_q, self.calf_kp, self.calf_kd)

    def _publish_joint(self, leg, joint, q, kp, kd):
        msg = MotorCmd()
        msg.mode = 0x0A
        msg.q = q
        msg.dq = 0.0
        msg.tau = 0.0
        msg.Kp = kp
        msg.Kd = kd
        self.joint_publishers[(leg, joint)].publish(msg)

    @staticmethod
    def _clamp(value, limit):
        if limit <= 0.0:
            return value
        return max(-limit, min(limit, value))

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw):
        q = Quaternion()
        q.w = math.cos(yaw * 0.5)
        q.z = math.sin(yaw * 0.5)
        return q


def main():
    rospy.init_node("standing_a1_driver")
    StandingA1Driver()
    rospy.spin()


if __name__ == "__main__":
    main()
