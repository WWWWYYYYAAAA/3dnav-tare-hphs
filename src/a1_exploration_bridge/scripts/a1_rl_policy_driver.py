#!/usr/bin/env python3
"""A1 Gazebo RL policy driver for the Unitree guide stair policy."""

import math
import os
import threading
import time

import rospy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from unitree_legged_msgs.msg import MotorCmd


# Policy order used by unitree_guide State_RL_test:
# FL, FR, RL, RR, each as hip/thigh/calf.
JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

DEFAULT_Q = [
    -0.15, 0.55, -1.5,
    0.15, 0.55, -1.5,
    -0.15, 0.70, -1.5,
    0.15, 0.70, -1.5,
]

# Keep the policy reference pose intact, but use the original fixed-stand calf
# angle during startup so a safe spawn at -1.5 is not mistaken for a settled
# standing command.
STARTUP_STAND_CALF_INDICES = (2, 5, 8, 11)

PRONE_Q = [
    0.0, 1.3, -2.4,
    0.0, 1.3, -2.4,
    0.0, 1.3, -2.4,
    0.0, 1.3, -2.4,
]

OBS_DIM = 45
ACTION_DIM = 12
HISTORY_LEN = 5


class A1RlPolicyDriver:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "a1_gazebo")
        self.link_name = rospy.get_param("~link_name", "%s::base" % self.model_name)
        self.policy_path = os.path.expanduser(
            rospy.get_param("~policy_path", self._default_policy_path())
        )
        self.policy_method = rospy.get_param("~policy_method", "act_inference")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/a1_gazebo/joint_states")
        self.joint_command_ns = rospy.get_param("~joint_command_ns", "/a1_gazebo")
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.policy_decimation = max(1, int(rospy.get_param("~policy_decimation", 1)))
        self.command_timeout = rospy.get_param("~command_timeout", 0.5)
        self.unpause_after_ready = rospy.get_param("~unpause_after_ready", False)
        self.startup_wait_timeout = rospy.get_param("~startup_wait_timeout", 10.0)
        self.skip_startup_poses = rospy.get_param("~skip_startup_poses", False)
        self.startup_prepare_time = rospy.get_param(
            "~startup_prepare_time",
            rospy.get_param("~startup_damping_time", 5.0),
        )
        self.startup_prepare_kp = rospy.get_param(
            "~startup_prepare_kp",
            rospy.get_param("~startup_damping_kp", 80.0),
        )
        self.startup_prepare_kd = rospy.get_param(
            "~startup_prepare_kd",
            rospy.get_param("~startup_damping_kd", 1.0),
        )
        self.startup_prone_kp = rospy.get_param("~startup_prone_kp", 80.0)
        self.startup_prone_kd = rospy.get_param("~startup_prone_kd", 1.0)
        self.startup_prone_rate = rospy.get_param("~startup_prone_rate", 1.0)
        self.startup_prone_min_time = rospy.get_param("~startup_prone_min_time", 2.0)
        self.startup_prone_max_time = rospy.get_param("~startup_prone_max_time", 8.0)
        self.startup_prone_settle_time = rospy.get_param("~startup_prone_settle_time", 0.5)
        self.startup_prone_pos_tolerance = rospy.get_param("~startup_prone_pos_tolerance", 0.10)
        self.startup_prone_vel_tolerance = rospy.get_param("~startup_prone_vel_tolerance", 0.35)
        self.startup_prone_contact_tolerance = rospy.get_param(
            "~startup_prone_contact_tolerance", 0.35
        )
        self.startup_prone_contact_vel_tolerance = rospy.get_param(
            "~startup_prone_contact_vel_tolerance", 0.75
        )
        self.startup_stand_kp = rospy.get_param("~startup_stand_kp", 80.0)
        self.startup_stand_kd = rospy.get_param("~startup_stand_kd", 1.0)
        self.startup_stand_rate = rospy.get_param("~startup_stand_rate", 0.75)
        self.startup_stand_min_time = rospy.get_param(
            "~startup_stand_min_time",
            rospy.get_param("~startup_stand_time", 5.0),
        )
        self.startup_stand_max_time = rospy.get_param("~startup_stand_max_time", 8.0)
        self.startup_stand_settle_time = rospy.get_param("~startup_stand_settle_time", 1.0)
        self.startup_stand_pos_tolerance = rospy.get_param("~startup_stand_pos_tolerance", 0.12)
        self.startup_stand_vel_tolerance = rospy.get_param("~startup_stand_vel_tolerance", 0.35)
        self.startup_base_ang_vel_tolerance = rospy.get_param("~startup_base_ang_vel_tolerance", 0.6)
        self.startup_stand_calf_q = rospy.get_param("~startup_stand_calf_q", -1.3)
        self.policy_active_topic = rospy.get_param(
            "~policy_active_topic",
            "/a1_rl_policy_driver/policy_active",
        )
        self.action_scale = rospy.get_param("~action_scale", 0.25)
        self.max_action = rospy.get_param("~max_action", 1.0)
        self.max_linear = rospy.get_param("~max_linear", 1.0)
        self.max_angular = rospy.get_param("~max_angular", 1.0)

        self.ang_vel_scale = rospy.get_param("~ang_vel_scale", 0.25)
        self.dof_pos_scale = rospy.get_param("~dof_pos_scale", 1.0)
        self.dof_vel_scale = rospy.get_param("~dof_vel_scale", 0.05)
        self.cmd_lin_x_scale = rospy.get_param("~cmd_lin_x_scale", 2.0)
        self.cmd_lin_y_scale = rospy.get_param("~cmd_lin_y_scale", 2.0)
        self.cmd_ang_z_scale = rospy.get_param("~cmd_ang_z_scale", 0.25)

        self.hip_kp = rospy.get_param("~hip_kp", 80.0)
        self.hip_kd = rospy.get_param("~hip_kd", 1.0)
        self.thigh_kp = rospy.get_param("~thigh_kp", 80.0)
        self.thigh_kd = rospy.get_param("~thigh_kd", 1.0)
        self.calf_kp = rospy.get_param("~calf_kp", 80.0)
        self.calf_kd = rospy.get_param("~calf_kd", 1.0)
        self.startup_hold_kp = max(
            self.hip_kp, self.thigh_kp, self.calf_kp,
            rospy.get_param("~startup_hold_kp", 80.0),
        )
        self.startup_hold_kd = max(
            self.hip_kd, self.thigh_kd, self.calf_kd,
            rospy.get_param("~startup_hold_kd", 1.0),
        )

        self.default_q = list(DEFAULT_Q)
        self.prone_q = list(PRONE_Q)
        self.initial_hold_q = list(self.default_q if self.skip_startup_poses else self.prone_q)
        self.startup_stand_q = list(self.default_q)
        for index in STARTUP_STAND_CALF_INDICES:
            self.startup_stand_q[index] = self.startup_stand_calf_q
        self.latest_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.joint_pos = None
        self.joint_vel = None
        self.base_quat = None
        self.base_ang_vel_world = None
        self.last_link_state_time = rospy.Time(0)
        self.obs_history = []
        self.last_action = [0.0] * ACTION_DIM
        self.last_targets = list(self.default_q)
        self.control_tick = 0
        self.state_ready_time = None
        self.startup_prone_start_time = None
        self.startup_prone_step_time = None
        self.startup_prone_settle_start_time = None
        self.startup_prone_done = False
        self.startup_stand_start_time = None
        self.startup_stand_step_time = None
        self.startup_stand_settle_start_time = None
        self.startup_stand_done = False
        self.policy_active = False
        self.physics_unpaused = not self.unpause_after_ready

        self.torch = self._load_torch()
        self.policy = self._load_policy()

        self.policy_active_pub = rospy.Publisher(self.policy_active_topic, Bool, queue_size=1, latch=True)
        self.joint_publishers = {}
        for joint_name in JOINT_ORDER:
            leg, joint = self._split_joint_name(joint_name)
            topic = "%s/%s_%s_controller/command" % (self.joint_command_ns, leg, joint)
            self.joint_publishers[joint_name] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        self._publish_policy_active(False)

        self.unpause_physics = None
        if self.unpause_after_ready:
            self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
            try:
                rospy.wait_for_service("/gazebo/unpause_physics", timeout=10.0)
            except rospy.ROSException:
                rospy.logwarn("Gazebo unpause_physics service is not ready yet")

        rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_cb, queue_size=10)
        rospy.Subscriber(self.joint_state_topic, JointState, self._joint_state_cb, queue_size=10)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self._link_states_cb, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=10)

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = rospy.Timer(rospy.Duration.from_sec(period), self._timer_cb)
        self._publish_targets(
            self.initial_hold_q,
            fixed_kp=self.startup_hold_kp,
            fixed_kd=self.startup_hold_kd,
        )
        if self.unpause_after_ready:
            self.startup_thread = threading.Thread(target=self._startup_unpause_worker)
            self.startup_thread.daemon = True
            self.startup_thread.start()
        rospy.loginfo("A1 RL policy driver active: %s", self.policy_path)
        rospy.loginfo("A1 RL policy method: %s, history: %dx%d", self.policy_method, HISTORY_LEN, OBS_DIM)
        rospy.loginfo("A1 RL joint/action order: %s", ", ".join(JOINT_ORDER))

    @staticmethod
    def _default_policy_path():
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        workspace_dir = os.path.dirname(os.path.dirname(package_dir))
        return os.path.join(
            workspace_dir,
            "third_party",
            "rl_policy",
            "a1",
            "policy_act_inference_stair.pt",
        )

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
        methods = list(policy._c._method_names())
        if self.policy_method not in methods:
            raise rospy.ROSException(
                "Policy %s does not expose method %s. Available methods: %s" %
                (self.policy_path, self.policy_method, ", ".join(methods))
            )
        inference = getattr(policy, self.policy_method)
        with self.torch.no_grad():
            test_action = inference(
                self.torch.zeros(1, HISTORY_LEN * OBS_DIM, dtype=self.torch.float32)
            )
        if test_action.numel() != ACTION_DIM:
            raise rospy.ROSException(
                "Expected policy input/output shape %d -> %d via %s, got output %s" %
                (HISTORY_LEN * OBS_DIM, ACTION_DIM, self.policy_method,
                 tuple(test_action.shape))
            )
        return policy

    def _cmd_vel_cb(self, msg):
        self.latest_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def _joint_state_cb(self, msg):
        pos_by_name = dict(zip(msg.name, msg.position))
        vel_by_name = dict(zip(msg.name, msg.velocity))
        try:
            self.joint_pos = [
                self._nearest_equivalent_angle(pos_by_name[name], self.last_targets[index])
                for index, name in enumerate(JOINT_ORDER)
            ]
            self.joint_vel = [vel_by_name.get(name, 0.0) for name in JOINT_ORDER]
        except KeyError as exc:
            rospy.logwarn_throttle(5.0, "Waiting for joint state %s in %s", exc, self.joint_state_topic)

    def _model_states_cb(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return
        now = rospy.Time.now()
        if self.last_link_state_time != rospy.Time(0) and (now - self.last_link_state_time).to_sec() <= 0.5:
            return
        self.base_quat = msg.pose[index].orientation
        self.base_ang_vel_world = msg.twist[index].angular

    def _link_states_cb(self, msg):
        try:
            index = msg.name.index(self.link_name)
        except ValueError:
            return
        self.base_quat = msg.pose[index].orientation
        self.base_ang_vel_world = msg.twist[index].angular
        self.last_link_state_time = rospy.Time.now()

    def _timer_cb(self, _event):
        if not self.physics_unpaused:
            self._publish_targets(
                self.initial_hold_q,
                fixed_kp=self.startup_hold_kp,
                fixed_kd=self.startup_hold_kd,
            )
            return
        if not self._state_ready():
            self._publish_policy_active(False)
            self._publish_prepare()
            return

        if self.state_ready_time is None:
            self.state_ready_time = rospy.Time.now()
            self.obs_history = []
            self.startup_prone_start_time = None
            self.startup_prone_step_time = None
            self.startup_prone_settle_start_time = None
            self.startup_prone_done = False
            self.startup_stand_start_time = None
            self.startup_stand_step_time = None
            self.startup_stand_settle_start_time = None
            self.startup_stand_done = False
            rospy.loginfo(
                "A1 prepare started: holding fixed startup pose for %.2fs with Kp=%.1f Kd=%.1f",
                self.startup_prepare_time,
                self.startup_prepare_kp,
                self.startup_prepare_kd,
            )

        now = rospy.Time.now()
        if self._in_startup_prepare(now):
            self._publish_policy_active(False)
            self._publish_prepare()
            return

        if not self.skip_startup_poses:
            if not self._startup_prone_complete(now):
                self._publish_policy_active(False)
                self._publish_startup_prone(now)
                return

            if not self._startup_stand_complete(now):
                self._publish_policy_active(False)
                self._publish_startup_stand(now)
                return

        if not self.policy_active:
            self.policy_active = True
            self.obs_history = []
            self.last_action = [0.0] * ACTION_DIM
            self.last_targets = list(self.default_q)
            self.control_tick = 0
            if self.skip_startup_poses:
                rospy.loginfo("A1 prepare complete; skipping prone/stand startup and entering RL policy")
            else:
                rospy.loginfo("A1 startup stand settled; entering RL policy")
        self._publish_policy_active(True)

        if self.control_tick % self.policy_decimation == 0:
            obs = self._build_observation()
            with self.torch.no_grad():
                action = getattr(self.policy, self.policy_method)(obs).detach().cpu().view(-1).tolist()
            action = [self._clamp(value, self.max_action) for value in action]
            self.last_targets = [
                self.default_q[index] + self.action_scale * action[index]
                for index in range(ACTION_DIM)
            ]
            self.last_action = action

        self.control_tick += 1
        self._publish_targets(self.last_targets)

    def _in_startup_prepare(self, now):
        if self.startup_prepare_time <= 0.0:
            return False
        if self.state_ready_time is None:
            return True
        return (now - self.state_ready_time).to_sec() < self.startup_prepare_time

    def _publish_prepare(self):
        self._publish_targets(
            self.initial_hold_q,
            fixed_kp=self.startup_prepare_kp,
            fixed_kd=self.startup_prepare_kd,
        )

    def _startup_unpause_worker(self):
        deadline = time.monotonic() + max(0.0, self.startup_wait_timeout)
        while not rospy.is_shutdown():
            self._publish_targets(
                self.initial_hold_q,
                fixed_kp=self.startup_hold_kp,
                fixed_kd=self.startup_hold_kd,
            )
            connected = all(
                publisher.get_num_connections() > 0
                for publisher in self.joint_publishers.values()
            )
            if connected or (self.startup_wait_timeout > 0.0 and time.monotonic() >= deadline):
                break
            time.sleep(0.01)

        if rospy.is_shutdown():
            return

        if not connected:
            rospy.logwarn("RL joint controllers did not all connect before timeout; unpausing anyway")
        try:
            self.unpause_physics()
            self.physics_unpaused = True
            rospy.loginfo("Gazebo physics unpaused after RL startup commands were published")
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to unpause Gazebo after RL startup: %s", exc)

    def _publish_startup_prone(self, now):
        if self.startup_prone_start_time is None:
            self.startup_prone_start_time = now
            self.startup_prone_step_time = now
            self.last_targets = list(self.joint_pos if self.joint_pos is not None else self.prone_q)
            rospy.loginfo(
                "A1 startup prepare complete; moving to prone pose with Kp=%.1f Kd=%.1f rate=%.2f rad/s",
                self.startup_prone_kp,
                self.startup_prone_kd,
                self.startup_prone_rate,
            )

        self.last_targets = self._step_targets(
            now,
            self.startup_prone_step_time,
            self.last_targets,
            self.prone_q,
            self.startup_prone_rate,
        )
        self.startup_prone_step_time = now
        self._publish_targets(
            self.last_targets,
            fixed_kp=self.startup_prone_kp,
            fixed_kd=self.startup_prone_kd,
        )

    def _publish_startup_stand(self, now):
        if self.startup_stand_start_time is None:
            self.startup_stand_start_time = now
            self.startup_stand_step_time = now
            self.last_targets = list(self.joint_pos if self.joint_pos is not None else self.prone_q)
            rospy.loginfo(
                "A1 startup prone pose settled; slowly standing with Kp=%.1f Kd=%.1f rate=%.2f rad/s",
                self.startup_stand_kp,
                self.startup_stand_kd,
                self.startup_stand_rate,
            )

        self.last_targets = self._step_targets(
            now,
            self.startup_stand_step_time,
            self.last_targets,
            self.startup_stand_q,
            self.startup_stand_rate,
        )
        self.startup_stand_step_time = now
        self._publish_targets(
            self.last_targets,
            fixed_kp=self.startup_stand_kp,
            fixed_kd=self.startup_stand_kd,
        )

    def _startup_prone_complete(self, now):
        if self.startup_prone_done:
            return True
        complete, settle_start = self._startup_target_complete(
            now,
            self.prone_q,
            self.startup_prone_start_time,
            self.startup_prone_settle_start_time,
            self.startup_prone_min_time,
            self.startup_prone_settle_time,
            self.startup_prone_pos_tolerance,
            self.startup_prone_vel_tolerance,
            "prone pose",
        )
        self.startup_prone_settle_start_time = settle_start
        if not complete and self._startup_prone_contact_complete(now):
            complete = True
        if complete:
            self.startup_prone_done = True
        return complete

    def _startup_prone_contact_complete(self, now):
        """Allow a stable ground-contact pose to transition into standing."""
        if self.startup_prone_max_time <= 0.0 or self.startup_prone_start_time is None:
            return False
        if (now - self.startup_prone_start_time).to_sec() < self.startup_prone_max_time:
            return False

        errors = [
            self._angle_distance(self.prone_q[index], self.joint_pos[index])
            for index in range(ACTION_DIM)
        ]
        target_error = max(
            self._angle_distance(self.prone_q[index], self.last_targets[index])
            for index in range(ACTION_DIM)
        )
        actual_error = max(errors)
        joint_speed = max(abs(value) for value in self.joint_vel)
        base_ang_speed = math.sqrt(
            self.base_ang_vel_world.x * self.base_ang_vel_world.x +
            self.base_ang_vel_world.y * self.base_ang_vel_world.y +
            self.base_ang_vel_world.z * self.base_ang_vel_world.z
        )
        if (
            target_error <= self.startup_prone_pos_tolerance and
            actual_error <= self.startup_prone_contact_tolerance and
            joint_speed <= self.startup_prone_contact_vel_tolerance and
            base_ang_speed <= self.startup_base_ang_vel_tolerance
        ):
            rospy.logwarn(
                "A1 startup prone pose is contact-limited; proceeding to stand "
                "after %.1fs (actual_err=%.3f, joint_vel=%.3f)",
                self.startup_prone_max_time,
                actual_error,
                joint_speed,
            )
            return True
        return False

    def _startup_stand_complete(self, now):
        if self.startup_stand_done:
            return True
        complete, settle_start = self._startup_target_complete(
            now,
            self.startup_stand_q,
            self.startup_stand_start_time,
            self.startup_stand_settle_start_time,
            self.startup_stand_min_time,
            self.startup_stand_settle_time,
            self.startup_stand_pos_tolerance,
            self.startup_stand_vel_tolerance,
            "stand pose",
        )
        self.startup_stand_settle_start_time = settle_start
        if not complete and self._startup_stand_timed_out(now):
            rospy.logwarn(
                "A1 startup stand reached target but did not fully settle before %.2fs; entering RL policy",
                self.startup_stand_max_time,
            )
            complete = True
        if complete:
            self.startup_stand_done = True
        return complete

    def _startup_stand_timed_out(self, now):
        if self.startup_stand_max_time <= 0.0 or self.startup_stand_start_time is None:
            return False
        elapsed = (now - self.startup_stand_start_time).to_sec()
        if elapsed < self.startup_stand_max_time:
            return False
        target_error = max(
            self._angle_distance(self.startup_stand_q[index], self.last_targets[index])
            for index in range(ACTION_DIM)
        )
        base_ang_speed = math.sqrt(
            self.base_ang_vel_world.x * self.base_ang_vel_world.x +
            self.base_ang_vel_world.y * self.base_ang_vel_world.y +
            self.base_ang_vel_world.z * self.base_ang_vel_world.z
        )
        return (
            target_error <= self.startup_stand_pos_tolerance and
            base_ang_speed <= self.startup_base_ang_vel_tolerance
        )

    def _startup_target_complete(self, now, target, start_time, settle_start_time,
                                 min_time, settle_time, pos_tolerance, vel_tolerance, label):
        if start_time is None:
            return False, None

        elapsed = (now - start_time).to_sec()
        if elapsed < min_time:
            return False, None

        errors = [
            self._angle_distance(target[index], self.joint_pos[index])
            for index in range(ACTION_DIM)
        ]
        target_error = max(
            self._angle_distance(target[index], self.last_targets[index])
            for index in range(ACTION_DIM)
        )
        actual_error = max(errors)
        joint_speed = max(abs(value) for value in self.joint_vel)
        base_ang_speed = math.sqrt(
            self.base_ang_vel_world.x * self.base_ang_vel_world.x +
            self.base_ang_vel_world.y * self.base_ang_vel_world.y +
            self.base_ang_vel_world.z * self.base_ang_vel_world.z
        )

        settled = (
            target_error <= pos_tolerance and
            actual_error <= pos_tolerance and
            joint_speed <= vel_tolerance and
            base_ang_speed <= self.startup_base_ang_vel_tolerance
        )
        if not settled:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for A1 startup %s to settle: target_err=%.3f actual_err=%.3f (%s=%.3f) joint_vel=%.3f base_w=%.3f",
                label,
                target_error,
                actual_error,
                JOINT_ORDER[errors.index(actual_error)],
                actual_error,
                joint_speed,
                base_ang_speed,
            )
            return False, None

        if settle_start_time is None:
            return False, now
        return (now - settle_start_time).to_sec() >= settle_time, settle_start_time

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
        if len(obs) != OBS_DIM:
            raise rospy.ROSException("Internal observation has %d values, expected %d" % (len(obs), OBS_DIM))
        self._append_observation(obs)
        return self.torch.tensor(
            [value for history_obs in self.obs_history for value in history_obs],
            dtype=self.torch.float32,
        ).unsqueeze(0)

    def _append_observation(self, obs):
        if not self.obs_history:
            self.obs_history = [list(obs) for _ in range(HISTORY_LEN)]
            return
        self.obs_history = self.obs_history[-(HISTORY_LEN - 1):] + [list(obs)]

    def _active_cmd(self, now):
        if self.last_cmd_time == rospy.Time(0):
            return Twist()
        if (now - self.last_cmd_time).to_sec() > self.command_timeout:
            return Twist()
        return self.latest_cmd

    def _publish_policy_active(self, active):
        self.policy_active_pub.publish(Bool(data=active))

    def _publish_targets(self, targets, fixed_kp=None, fixed_kd=None):
        for index, joint_name in enumerate(JOINT_ORDER):
            leg, joint = self._split_joint_name(joint_name)
            if fixed_kp is None or fixed_kd is None:
                kp, kd = self._gains_for_joint(joint)
            else:
                kp, kd = fixed_kp, fixed_kd
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
    def _step_targets(now, last_step_time, current, target, rate):
        dt = (now - last_step_time).to_sec() if last_step_time else 0.0
        max_delta = max(0.0, rate) * max(0.0, dt)
        return [
            A1RlPolicyDriver._step_towards(current[index], target[index], max_delta)
            for index in range(ACTION_DIM)
        ]

    @staticmethod
    def _step_towards(value, target, max_delta):
        error = target - value
        if abs(error) <= max_delta:
            return target
        return value + math.copysign(max_delta, error)

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

    @staticmethod
    def _angle_distance(first, second):
        """Return the shortest distance between two revolute joint angles."""
        return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)

    @staticmethod
    def _nearest_equivalent_angle(value, reference):
        """Express a measured angle using the representation nearest reference."""
        return reference + (value - reference + math.pi) % (2.0 * math.pi) - math.pi


def main():
    rospy.init_node("a1_rl_policy_driver")
    A1RlPolicyDriver()
    rospy.spin()


if __name__ == "__main__":
    main()
