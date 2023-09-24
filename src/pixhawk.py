#!/usr/bin/env python3

import pymap3d as pm

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix

### main #####################################################################

def main():
	Pixhawk().loop()

class Pixhawk:
	def __init__(self):

		rospy.init_node("pixhawk")

		### local variables ##################################################

		self.local_startup: Point = None
		self.global_startup: GeoPoint = None
		self.global_origin_frequency = rospy.get_param("~global_origin_frequency")

		### connect to ROS ###################################################

		mavros_local_pose_topic = rospy.get_param("~mavros_local_pose_topic")
		mavros_global_pose_topic = rospy.get_param("~mavros_global_pose_topic")
		local_position_topic = rospy.get_param("~local_position_topic")
		global_position_topic = rospy.get_param("~global_position_topic")
		global_origin_topic = rospy.get_param("~global_origin_topic")

		self.local_pose_pub = rospy.Publisher(local_position_topic, Pose, queue_size=1)
		self.global_pose_pub = rospy.Publisher(global_position_topic, GeoPoint, queue_size=1)
		self.global_origin_pub = rospy.Publisher(global_origin_topic, GeoPoint, queue_size=1)

		self.mavros_local_pose_sub = rospy.Subscriber(mavros_local_pose_topic, PoseStamped, self.mavros_local_callback)
		self.mavros_global_pose_sub = rospy.Subscriber(mavros_global_pose_topic, NavSatFix, self.mavros_global_pose_callback)

		### end init #########################################################

	### callbacks ############################################################

	def mavros_local_callback(self, posestamped: PoseStamped):
		if not self.local_startup:
			self.local_startup = posestamped.pose.position
		self.local_pose_pub.publish(posestamped.pose)

	def mavros_global_pose_callback(self, gps_nsf: NavSatFix):
		gps_gp = GeoPoint(
			latitude=gps_nsf.latitude,
			longitude=gps_nsf.longitude,
			altitude=gps_nsf.altitude,
		)
		if not self.global_startup:
			self.global_startup = gps_gp
		self.global_pose_pub.publish(gps_gp)

	### loop #################################################################

	def loop(self):
		rate = rospy.Rate(self.global_origin_frequency)

		# calculate global origin as soon as local_startup and global_startup are set
		while not rospy.is_shutdown():
			if self.local_startup and self.global_startup:
				x, y, z = self.local_startup.x, self.local_startup.y, self.local_startup.z
				lat1, lon1, h1 = self.global_startup.latitude, self.global_startup.longitude, self.global_startup.altitude
				lat0, lon0, h0 = pm.enu2geodetic(-x, -y, -z, lat1, lon1, h1)
				global_origin = GeoPoint(latitude=lat0, longitude=lon0, altitude=h0)
				break
			rate.sleep()

		# publish global origin continuously
		while not rospy.is_shutdown():
			self.global_origin_pub.publish(global_origin)
			rate.sleep()

if __name__ == "__main__":
	main()
