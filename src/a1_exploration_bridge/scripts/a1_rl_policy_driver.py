#!/usr/bin/env python3
"""A1 Gazebo RL policy driver for TorchScript policies."""

import math
import os

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from unitree_legged_msgs.msg import MotorCmd


JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

DEFAULT_JOINT_ANGLES = {
    "FL_hip_joint": 0.0,
    "RL_hip_joint": 0.0,
    "FR_hip_joint": -0.0,
    "RR_hip_joint": -0.0,
    "FL_thigh_joint": 0.75,
    "RL_thigh_joint": 0.75,
    "FR_thigh_joint": 0.75,
    "RR_thigh_joint": 0.75,
    "FL_calf_joint": -1.5,
    "RL_calf_joint": -1.5,
    "FR_calf_joint": -1.5,
    "RR_calf_joint": -1.5,
}


class A1RlPolicyDriver:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "a1_gazebo")
        self.policy_path = os.path.expanduser(rospy.get_param("~policy_path", ""))
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/a1_gazebo/joint_states")
        self.joint_command_ns = rospy.get_param("~joint_command_ns", "/a1_gazebo")
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.policy_decimation = max(1, int(rospy.get_param("~policy_decimation", 1)))
        self.command_timeout = rospy.get_param("~command_timeout", 0.5)
        self.startup_stand_time = rospy.get_param("~startup_stand_time", 1.0)
        self.action_scale = rospy.get_param("~action_scale", 0.25)
        self.max_action = rospy.get_param("~max_action", 1.0)
        self.max_linear = rospy.get_param("~max_linear", 1.0)
        self.max_angular = rospy.get_param("~max_angular", 1.2)

        self.ang_vel_scale = rospy.get_param("~ang_vel_scale", 0.25)
        self.dof_pos_scale = rospy.get_param("~dof_pos_scale", 1.0)
        self.dof_vel_scale = rospy.get_param("~dof_vel_scale", 0.05)
        self.cmd_lin_x_scale = rospy.get_param("~cmd_lin_x_scale", 1.0)
        self.cmd_lin_y_scale = rospy.get_param("~cmd_lin_y_scale", 1.0)
        self.cmd_ang_z_scale = rospy.get_param("~cmd_ang_z_scale", 1.0)

        self.hip_kp = rospy.get_param("~hip_kp", 30.0)
        self.hip_kd = rospy.get_param("~hip_kd", 1.0)
        self.thigh_kp = rospy.get_param("~thigh_kp", 30.0)
        self.thigh_kd = rospy.get_param("~thigh_kd", 1.0)
        self.calf_kp = rospy.get_param("~calf_kp", 30.0)
        self.calf_kd = rospy.get_param("~calf_kd", 1.0)

        self.default_q = [DEFAULT_JOINT_ANGLES[name] for name in JOINT_ORDER]
        self.latest_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.joint_pos = None
        self.joint_vel = None
        self.base_quat = None
        self.base_ang_vel_world = None
        self.last_action = [0.0] * len(JOINT_ORDER)
        self.last_targets = list(self.default_q)
        self.control_tick = 0
        self.start_time = rospy.Time.now()

        self.torch = self._load_torch()
        self.policy = self._load_policy()

        self.joint_publishers = {}
        for joint_name in JOINT_ORDER:
            leg, joint = self._split_joint_name(joint_name)
            topic = "%s/%s_%s_controller/command" % (self.joint_command_ns, leg, joint)
            self.joint_publishers[joint_name] = rospy.Publisher(topic, MotorCmd, queue_size=1)

        rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_cb, queue_size=10)
        rospy.Subscriber(self.joint_state_topic, JointState, self._joint_state_cb, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=10)

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = rospy.Timer(rospy.Duration.from_sec(period), self._timer_cb)
        rospy.loginfo("A1 RL policy driver active: %s", self.policy_path)
        rospy.loginfo("A1 RL joint/action order: %s", ", ".join(JOINT_ORDER))

    @staticmethod
    def _load_torch():
        try:
            import torch
            return torch
        except ImportError as exc:
            raise rospy.ROSException(
                "PyTorch is required for motion_mode:=rl. Install torch in the ROS Docker "
                "container, or run motion_mode:=standing. Import error: %s" % exc
            )

    def _load_policy(self):
        if not self.policy_path:
            raise rospy.ROSException("~policy_path is empty")
        if not os.path.exists(self.policy_path):
            raise rospy.ROSException("Policy file does not exist: %s" % self.policy_path)

        policy = self.torch.jit.load(self.policy_path, map_location="cpu")
        policy.eval()
        with self.torch.no_grad():
            test_action = policy(self.torch.zeros(1, 45, dtype=self.torch.float32))
        if tuple(test_action.shape) != (1, 12):
            raise rospy.ROSException(
                "Expected policy input/output shape 45 -> 12, got output %s" %
                (tuple(test_action.shape),)
            )
        return policy

    def _cmd_vel_cb(self, msg):
        self.latest_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def _joint_state_cb(self, msg):
        pos_by_name = dict(zip(msg.name, msg.position))
        vel_by_name = dict(zip(msg.name, msg.velocity))
        try:
            self.joint_pos = [pos_by_name[name] for name in JOINT_ORDER]
            self.joint_vel = [vel_by_name.get(name, 0.0) for name in JOINT_ORDER]
        except KeyError as exc:
            rospy.logwarn_throttle(5.0, "Waiting for joint state %s in %s", exc, self.joint_state_topic)

    def _model_states_cb(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return
        self.base_quat = msg.pose[index].orientation
        self.base_ang_vel_world = msg.twist[index].angular

    def _timer_cb(self, _event):
        if self._in_startup_stand() or not self._state_ready():
            self.last_targets = list(self.default_q)
            self._publish_targets(self.last_targets)
            return

        if self.control_tick % self.policy_decimation == 0:
            obs = self._build_observation()
            with self.torch.no_grad():
                action = self.policy(obs).detach().cpu().view(-1).tolist()
            action = [self._clamp(value, self.max_action) for value in action]
            self.last_targets = [
                self.default_q[index] + self.action_scale * action[index]
                for index in range(len(JOINT_ORDER))
            ]
            self.last_action = action

        self.control_tick += 1
        self._publish_targets(self.last_targets)

    def _in_startup_stand(self):
        if self.startup_stand_time <= 0.0:
            return False
        return (rospy.Time.now() - self.start_time).to_sec() < self.startup_stand_time

    def _state_ready(self):
        ready = self.joint_pos is not None and self.joint_vel is not None
        ready = ready and self.base_quat is not None and self.base_ang_vel_world is not None
        if not ready:
            rospy.logwarn_throttle(5.0, "Waiting for joint states and Gazebo model state before RL inference")
        return ready

    def _build_observation(self):
        cmd = self._active_cmd(rospy.Time.now())
        base_ang_vel = self._rotate_inverse(
            self.base_quat,
            [self.base_ang_vel_world.x, self.base_ang_vel_world.y, self.base_ang_vel_world.z],
        )
        projected_gravity = self._rotate_inverse(self.base_quat, [0.0, 0.0, -1.0])
        commands = [
            self._clamp(cmd.linear.x, self.max_linear) * self.cmd_lin_x_scale,
            self._clamp(cmd.linear.y, self.max_linear) * self.cmd_lin_y_scale,
            self._clamp(cmd.angular.z, self.max_angular) * self.cmd_ang_z_scale,
        ]
        joint_pos_error = [
            (self.joint_pos[index] - self.default_q[index]) * self.dof_pos_scale
            for index in range(len(JOINT_ORDER))
        ]
        joint_vel = [value * self.dof_vel_scale for value in self.joint_vel]
        obs = (
            [value * self.ang_vel_scale for value in base_ang_vel] +
            projected_gravity +
            commands +
            joint_pos_error +
            joint_vel +
            self.last_action
        )
        return self.torch.tensor(obs, dtype=self.torch.float32).unsqueeze(0)

    def _active_cmd(self, now):
        if self.last_cmd_time == rospy.Time(0):
            return Twist()
        if (now - self.last_cmd_time).to_sec() > self.command_timeout:
            return Twist()
        return self.latest_cmd

    def _publish_targets(self, targets):
        for index, joint_name in enumerate(JOINT_ORDER):
            leg, joint = self._split_joint_name(joint_name)
            kp, kd = self._gains_for_joint(joint)
            msg = MotorCmd()
            msg.mode = 0x0A
            msg.q = targets[index]
            msg.dq = 0.0
            msg.tau = 0.0
            msg.Kp = kp
            msg.Kd = kd
            self.joint_publishers[joint_name].publish(msg)

    def _gains_for_joint(self, joint):
        if joint == "hip":
            return self.hip_kp, self.hip_kd
        if joint == "thigh":
            return self.thigh_kp, self.thigh_kd
        return self.calf_kp, self.calf_kd

    @staticmethod
    def _split_joint_name(joint_name):
        parts = joint_name.split("_")
        return parts[0], parts[1]

    @staticmethod
    def _rotate_inverse(q, vector):
        matrix = A1RlPolicyDriver._quat_to_matrix(q.x, q.y, q.z, q.w)
        return [
            matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
            matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
            matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2],
        ]

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
        if limit <= 0.0:
            return value
        return max(-limit, min(limit, value))


def main():
    rospy.init_node("a1_rl_policy_driver")
    A1RlPolicyDriver()
    rospy.spin()


if __name__ == "__main__":
    main()
