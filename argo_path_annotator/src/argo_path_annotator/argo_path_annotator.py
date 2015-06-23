#!/usr/bin/env python

import rospy
import math
import threading

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class ArgoPathAnnotator(object):

	def __init__(self):
		self._path_length = 0
		self._path_starting_time = None

		self._speed = 0
		self._min_speed = 0.05
		self._last_pose = None

		self._speed_smooth_factor = 0.05

		self._time_estimations = list()

		self._time_est_pub = rospy.Publisher("/time_estimate_raw", Float32, queue_size=10)
		self._est_pub_rate = rospy.Rate(1)

		self._path_sub = rospy.Subscriber("/drivepath", Path, self._path_callback)
		self._pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, self._pose_callback)

		thread = threading.Thread(target=self._time_est_aggregator)
		thread.daemon = True
		thread.start()


	def _path_callback(self, msg):
		if len(msg.poses) == 0:
			self._path_length = 0
			self._path_starting_time = None
		else:
			self._path_length = self.calc_path_length(list(ps.pose.position for ps in msg.poses))
			self._path_starting_time = rospy.Time.now()
		self.update_time_estimate()


	def _pose_callback(self, msg):
		if self._last_pose is None:
			self._last_pose = msg
			return
		time_diff = self.calc_time_dist(self._last_pose, msg)
		if time_diff > 0:
			new_speed = self.calc_waypoint_dist(self._last_pose.pose.position, msg.pose.position) / time_diff
			self._speed = max(self._min_speed, (1 - self._speed_smooth_factor) * new_speed + self._speed_smooth_factor * self._speed)
		self._last_pose = msg

		self.update_time_estimate()


	def update_time_estimate(self):
		time_estimate = 0
		if self._speed > 0 and self._path_length > 0:
			path_time_estimate = self._path_length / self._speed
			time_elapsed = (rospy.Time.now() - self._path_starting_time).to_sec()
			time_estimate = path_time_estimate - time_elapsed

		self._time_estimations.append(time_estimate)


	def calc_path_length(self, path):
		length = 0
		lastpoint = path[0]
		for point in path[1:]:
			length += self.calc_waypoint_dist(lastpoint, point)
			lastpoint = point
		return length

	def calc_waypoint_dist(self, wp1, wp2):
		return math.sqrt(math.pow(wp2.x - wp1.x, 2) + math.pow(wp2.y - wp1.y, 2) + math.pow(wp2.z - wp1.z, 2))

	def calc_time_dist(self, ps1, ps2):
		return (ps2.header.stamp - ps1.header.stamp).to_sec()

	def _time_est_aggregator(self):
		while True:
			time_estimate_agg = sum(self._time_estimations) / len(self._time_estimations) \
				if len(self._time_estimations) else 0
			self._time_estimations = list()
			self._time_est_pub.publish(Float32(time_estimate_agg))
			self._est_pub_rate.sleep()
