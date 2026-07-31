#!/usr/bin/env python3
"""Start another roslaunch file after an optional Bool topic and delay."""

import shlex
import signal
import subprocess

import rospy
from std_msgs.msg import Bool


class DelayedRoslaunch:
    def __init__(self):
        self.launch_file = rospy.get_param("~launch_file", "")
        self.launch_args = rospy.get_param("~launch_args", "")
        self.wait_topic = rospy.get_param("~wait_topic", "")
        self.wait_timeout = rospy.get_param("~wait_timeout", 0.0)
        self.delay = rospy.get_param("~delay", 0.0)
        self.process = None

        if not self.launch_file:
            raise rospy.ROSException("~launch_file is required")

    def run(self):
        if self.wait_topic:
            self._wait_for_bool_true()
        if self.delay > 0.0 and not rospy.is_shutdown():
            rospy.loginfo("Delaying %.2fs before starting %s", self.delay, self.launch_file)
            rospy.sleep(self.delay)
        if rospy.is_shutdown():
            return

        command = ["roslaunch", self.launch_file] + shlex.split(self.launch_args)
        rospy.loginfo("Starting delayed roslaunch: %s", " ".join(command))
        self.process = subprocess.Popen(command)
        rospy.on_shutdown(self._shutdown_child)

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown() and self.process.poll() is None:
            rate.sleep()

        if self.process.poll() is not None and self.process.returncode != 0:
            rospy.logwarn("Delayed roslaunch exited with code %s: %s",
                          self.process.returncode, self.launch_file)

    def _wait_for_bool_true(self):
        rospy.loginfo("Waiting for %s before starting %s", self.wait_topic, self.launch_file)
        deadline = None
        if self.wait_timeout > 0.0:
            deadline = rospy.Time.now() + rospy.Duration.from_sec(self.wait_timeout)

        while not rospy.is_shutdown():
            timeout = 1.0
            if deadline is not None:
                remaining = (deadline - rospy.Time.now()).to_sec()
                if remaining <= 0.0:
                    raise rospy.ROSException("Timed out waiting for %s" % self.wait_topic)
                timeout = min(timeout, remaining)
            try:
                msg = rospy.wait_for_message(self.wait_topic, Bool, timeout=timeout)
            except rospy.ROSException:
                rospy.loginfo_throttle(5.0, "Still waiting for %s", self.wait_topic)
                continue
            if msg.data:
                rospy.loginfo("Received true on %s", self.wait_topic)
                return

    def _shutdown_child(self):
        if self.process is None or self.process.poll() is not None:
            return
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self.process.terminate()


def main():
    rospy.init_node("delayed_roslaunch")
    DelayedRoslaunch().run()


if __name__ == "__main__":
    main()
