#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import Pose, PoseArray, Vector3
# from std_msgs.msg import Int64MuliArray
from im_pickit_msgs.msg import Object, ObjectArray
from im_pickit_msgs.srv import CheckForObjects
from math import sin, cos
roslib.load_manifest('combined_tf')

global link0_link1
link0_link1 = 0.077
class import_frame:
	def __init__(self, wrt_robot_frame):
		self.wrt_robot_frame = wrt_robot_frame
		self.pose_camera = Pose()

		self.position_rb_base = Pose()

		self.pose_point_rb = Pose()
		# self.pose_pointlist_rb = PoseArray()

		self.pose_point_indy = Pose()
		# self.pose_pointlist_indy = PoseArray()

		self.camera_pub = rospy.Publisher('/camera_pose', Pose, queue_size = 1)
		self.rb_objects_pub = rospy.Publisher('/rb10/object_pose', Pose, queue_size=1)
		self.indy_objects_pub = rospy.Publisher('/indy7/object_pose', Pose, queue_size=1)


		# rospy.ServiceProxy('/pickit/check_for_objects', CheckForObjects)
		rospy.Subscriber('/pickit/objects_wrt_robot_frame', ObjectArray, self.subscribe_frame)
		# rospy.Subscriber('/rb_rel_indy', Int64MuliArray, self.rb_rel_indy)


	def subscribe_frame(self, wrt_robot_frame):
		"""
		Subscribe position of rb(/base_footprint) w.r.t indy(/world), objects w.r.t. indy(/link1) and rb(/base_footprint), then make tf tree.
		"""
		# subscribe frame of rb wrt indy.
		self.position_rb_base.position.x = 0.007
		self.position_rb_base.position.y = 1.405
		self.position_rb_base.position.z = 0.037
		self.position_rb_base.orientation.x = 0.010
		self.position_rb_base.orientation.y = 0.005
		self.position_rb_base.orientation.z = - 0.286
		self.position_rb_base.orientation.w = 0.958


		# # subscribe frame of camera.
		# self.pose_camera.position.x = wrt_robot_frame.robot_to_camera_tf.transform.translation.x
		# self.pose_camera.position.y = wrt_robot_frame.robot_to_camera_tf.transform.translation.y
		# self.pose_camera.position.z = wrt_robot_frame.robot_to_camera_tf.transform.translation.z
		# self.pose_camera.orientation.x = wrt_robot_frame.robot_to_camera_tf.transform.rotation.x
		# self.pose_camera.orientation.y = wrt_robot_frame.robot_to_camera_tf.transform.rotation.y
		# self.pose_camera.orientation.z = wrt_robot_frame.robot_to_camera_tf.transform.rotation.z
		# self.pose_camera.orientation.w = wrt_robot_frame.robot_to_camera_tf.transform.rotation.w

		# print(self.pose_camera)
		# self.camera_pub.publish(self.pose_camera)
		
		
		# subscribe frame of objects from rainbow robot.
		for i in range(wrt_robot_frame.n_valid_objects):
			# put object frame in poses.
			x0 = wrt_robot_frame.objects[i].object_tf.transform.translation.x - self.position_rb_base.position.x 
			y0 = wrt_robot_frame.objects[i].object_tf.transform.translation.y - self.position_rb_base.position.y 
			z0 = wrt_robot_frame.objects[i].object_tf.transform.translation.z - self.position_rb_base.position.z 
			# - self.position_rb_base.position.x
			
			rb_orientation_list = [self.position_rb_base.orientation.x, 
								self.position_rb_base.orientation.y, 
								self.position_rb_base.orientation.z, 
								self.position_rb_base.orientation.w]
			euler_rb = tf.transformations.euler_from_quaternion(rb_orientation_list)

			x1 = cos(euler_rb[2])*x0 + sin(euler_rb[2])*y0
			y1 = - sin(euler_rb[2])*x0 + cos(euler_rb[2])*y0
			z1 = z0

			x2 = x1
			y2 = cos(euler_rb[0])*y1 + sin(euler_rb[0])*z1
			z2 = - sin(euler_rb[0])*y1 + cos(euler_rb[0])*z1

			self.pose_point_rb.position.x = - sin(euler_rb[1])*z2 + cos(euler_rb[1])*x2
			self.pose_point_rb.position.y = y2
			self.pose_point_rb.position.z = cos(euler_rb[1])*z2 + sin(euler_rb[1])*x2
			

			object_orientation_list = [wrt_robot_frame.objects[i].object_tf.transform.rotation.x, 
								wrt_robot_frame.objects[i].object_tf.transform.rotation.y, 
								wrt_robot_frame.objects[i].object_tf.transform.rotation.z, 
								wrt_robot_frame.objects[i].object_tf.transform.rotation.w]

			euler_object = tf.transformations.euler_from_quaternion(object_orientation_list)

			quat = tf.transformations.quaternion_from_euler(euler_object[0] - euler_rb[1], 
															euler_object[1] - euler_rb[1], 
															euler_object[2] - euler_rb[2])
			self.pose_point_rb.orientation.x = quat[0]
			self.pose_point_rb.orientation.y = quat[1]
			self.pose_point_rb.orientation.z = quat[2]
			self.pose_point_rb.orientation.w = quat[3]
			# quat[0]
			# wrt_robot_frame.objects[i].object_tf.transform.rotation.w
			
			print("object w.r.t rainbow")
			print(self.pose_point_rb)
			self.rb_objects_pub.publish(self.pose_point_rb)
		

		

		# subscribe frame of objects from indy robot.
		global link0_link1
		link0_link1 = 0.077
		for i in range(wrt_robot_frame.n_valid_objects):
			# put object frame in poses.
			self.pose_point_indy.position.x = wrt_robot_frame.objects[i].object_tf.transform.translation.x
			self.pose_point_indy.position.y = wrt_robot_frame.objects[i].object_tf.transform.translation.y
			self.pose_point_indy.position.z = wrt_robot_frame.objects[i].object_tf.transform.translation.z - link0_link1
			self.pose_point_indy.orientation.x = wrt_robot_frame.objects[i].object_tf.transform.rotation.x
			self.pose_point_indy.orientation.y = wrt_robot_frame.objects[i].object_tf.transform.rotation.y 
			self.pose_point_indy.orientation.z = wrt_robot_frame.objects[i].object_tf.transform.rotation.z 
			self.pose_point_indy.orientation.w = wrt_robot_frame.objects[i].object_tf.transform.rotation.w
			# - world_link1
			print("object w.r.t indy7")
			print(self.pose_point_indy)
			self.indy_objects_pub.publish(self.pose_point_indy)
			# self.indy_object_to_tf(wrt_robot_frame)

		# listener = tf.TransformListener()
		# while not rospy.is_shutdown():
		# 	try:
		# 		rospy.sleep(0.1)
		# 		(trans,rot) = listener.lookupTransform('base_footprint', 'object', rospy.Time(0))
		# 		break
		# 	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# 		continue

		# print("good!")
		# self.pose_point_rb.position.x = round(trans[0], 3)
		# self.pose_point_rb.position.y = round(trans[1], 3)
		# self.pose_point_rb.position.z = round(trans[2], 3)
		# self.pose_point_rb.orientation.x = round(rot[0], 3)
		# self.pose_point_rb.orientation.y = round(rot[1], 3)
		# self.pose_point_rb.orientation.z = round(rot[2], 3)
		# self.pose_point_rb.orientation.w = round(rot[3], 3)

		# # print("trans: ", trans)
		# # print("rot: ", rot)
	
		# self.rb_objects_pub.publish(self.pose_point_rb)
		# self.rb_object_to_tf(wrt_robot_frame)


	def rb_wrt_indy_to_tf(self, wrt_robot_frame):
		"""
		calculate from frame data(translation and rotation) of 'base_footprint' frame to tf data
		"""
		pose_world = self.position_rb_base
		br = tf.TransformBroadcaster()
		br.sendTransform(
			(pose_world.position.x, pose_world.position.y, 
			pose_world.position.z), 
			(pose_world.orientation.x, pose_world.orientation.y, 
			pose_world.orientation.z, pose_world.orientation.w), 
			rospy.Time.now(), 'base_footprint', 'link0')


	def rb_object_to_tf(self, wrt_robot_frame):
		"""
		calculate from frame data(translation and rotation) of 'rb_object' frame to tf data.
		"""
		object_list = 'rb_object1'
		# for i in range(len(self.pose_point_rb.poses)):
		pose_object = self.pose_point_rb
		br = tf.TransformBroadcaster()
		br.sendTransform(
			(pose_object.position.x, pose_object.position.y, 
			pose_object.position.z), 
			(pose_object.orientation.x, pose_object.orientation.y, 
			pose_object.orientation.z, pose_object.orientation.w), 
			rospy.Time.now(), object_list, 'base_footprint')


	def indy_object_to_tf(self, wrt_robot_frame):
		"""
		calculate from frame data(translation and rotation) of 'indy_object' frame to tf data.
		"""
		global link0_link1
		object_list = 'object'
		listener = tf.TransformListener()
		# 'indy_object1'
		# for i in range(len(self.pose_pointlist_indy.poses)):
		pose_object = self.pose_point_indy
		br = tf.TransformBroadcaster()
		br.sendTransform(
			(pose_object.position.x, pose_object.position.y, 
			pose_object.position.z + link0_link1), # z-axis offset
			(pose_object.orientation.x, pose_object.orientation.y, 
			pose_object.orientation.z , pose_object.orientation.w), 
			rospy.Time.now(), object_list, 'link0')

		


	def pickit_world_to_tf(self, wrt_robot_frame):
		"""
		connect tf relationship between 'world' frame and 'pickit/world' frame to visualize pointcloud.
		"""
		br = tf.TransformBroadcaster()
		br.sendTransform(
			(0, 0, 0), 
			(0, 0, 0, 1), 
			rospy.Time.now(), 'pickit/world', 'link0')

	def camera_wrt_rb_to_tf(self):
		"""
		calculate from frame data(translation and rotation) of 'camera' frame to tf data
		and publish as topic 'frame/camera'.
		"""
		pose_camera = self.pose_camera
		br = tf.TransformBroadcaster()
		br.sendTransform(
			(0.835, 0.074, 0.954), 
			(0.435, 0.769, -0.415, 0.216), 
			rospy.Time.now(), 'base_footprint', 'camera')

	def camera_wrt_indy_to_tf(self):
		"""
		calculate from frame data(translation and rotation) of 'camera' frame to tf data
		and publish as topic 'frame/camera'.
		"""
		pose_camera = self.pose_camera
		br = tf.TransformBroadcaster()
		br.sendTransform(
			(0.7384, 0.5570, 0.6367), 
			(-0.6322, -0.6158, 0.3309, 0.3341), 
			rospy.Time.now(), 'camera', 'link0')

def main():
	rospy.init_node('tf_manager')
	wrt_robot_frame = ObjectArray()
	frame = import_frame(wrt_robot_frame)
	while not rospy.is_shutdown():
		frame.subscribe_frame(wrt_robot_frame)
		frame.pickit_world_to_tf(wrt_robot_frame)
		frame.rb_wrt_indy_to_tf(wrt_robot_frame)
		frame.indy_object_to_tf(wrt_robot_frame)
		# frame.rb_object_to_tf(wrt_robot_frame)
		
		# rospy.sleep(0.1)
		# frame.camera_wrt_rb_to_tf()
		# frame.camera_wrt_indy_to_tf()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass