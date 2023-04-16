#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import Pose, PoseArray
from im_pickit_msgs.msg import Object, ObjectArray

roslib.load_manifest('combined_tf')

"""
Subscribe position of objects and rb base, then make tf tree.
"""
class import_frame:
	def __init__(self, wrt_robot_frame):
		self.wrt_robot_frame = wrt_robot_frame
		self.pose_camera = Pose()
		self.pose_point = Pose()
		self.pose_pointlist = PoseArray()

		self.camera_pub = rospy.Publisher('frame/camera', Pose, queue_size = 1)
		self.objects_pub = rospy.Publisher('frame/objects', PoseArray, queue_size=1)

		rospy.Subscriber('/pickit/objects_wrt_robot_frame', ObjectArray, self.subscribe_frame)


	def subscribe_frame(self, wrt_robot_frame):
		# subscribe frame of camera.
		self.pose_camera.position.x = wrt_robot_frame.robot_to_camera_tf.transform.translation.x
		self.pose_camera.position.y = wrt_robot_frame.robot_to_camera_tf.transform.translation.y
		self.pose_camera.position.z = wrt_robot_frame.robot_to_camera_tf.transform.translation.z
		self.pose_camera.orientation.x = wrt_robot_frame.robot_to_camera_tf.transform.rotation.x
		self.pose_camera.orientation.y = wrt_robot_frame.robot_to_camera_tf.transform.rotation.y
		self.pose_camera.orientation.z = wrt_robot_frame.robot_to_camera_tf.transform.rotation.z
		self.pose_camera.orientation.w = wrt_robot_frame.robot_to_camera_tf.transform.rotation.w

		self.camera_pub.publish(self.pose_camera)
		print(self.pose_camera)

		# subscribe frame of objects.
		for i in range(wrt_robot_frame.n_valid_objects):
			# put object frame in poses.
			self.pose_point.position.x = wrt_robot_frame.objects[i].object_tf.transform.translation.x
			self.pose_point.position.y = wrt_robot_frame.objects[i].object_tf.transform.translation.y
			self.pose_point.position.z = wrt_robot_frame.objects[i].object_tf.transform.translation.z
			self.pose_point.orientation.x = wrt_robot_frame.objects[i].object_tf.transform.rotation.x
			self.pose_point.orientation.y = wrt_robot_frame.objects[i].object_tf.transform.rotation.y
			self.pose_point.orientation.z = wrt_robot_frame.objects[i].object_tf.transform.rotation.z
			self.pose_point.orientation.w = wrt_robot_frame.objects[i].object_tf.transform.rotation.w

			self.pose_pointlist.poses.append(self.pose_point)
			print(self.pose_pointlist.poses[-1])
		print(self.pose_pointlist.poses)
		self.objects_pub.publish(self.pose_pointlist)


	def object_to_tf(self):
		"""
		calculate from frame data(translation and rotation) of 'object' frame to tf data
		and publish as topic 'frame/objects'.
		"""
		object_list = ['object']
		for i in range(len(self.pose_pointlist.poses)):
			pose_object = self.pose_pointlist.poses[i]
			br = tf.TransformBroadcaster()
			br.sendTransform(
				(pose_object.position.x, pose_object.position.y, 
				pose_object.position.z), 
				(pose_object.orientation.x, pose_object.orientation.y, 
				pose_object.orientation.z, pose_object.orientation.w), 
				rospy.Time.now(), object_list[i], 'base_footprint')


	def rb_to_tf(self):
		"""
		calculate from frame data(translation and rotation) of 'camera' frame to tf data
		and publish as topic 'frame/camera'.
		"""
		pose_rb = self.pose_camera
		br = tf.TransformBroadcaster()
		br.sendTransform(
			(pose_rb.position.x, pose_rb.position.y, 
			pose_rb.position.z), 
			(pose_rb.orientation.x, pose_rb.orientation.y, 
			pose_rb.orientation.z, pose_rb.orientation.w), 
			rospy.Time.now(), 'camera', 'base_footprint')

def main():
	rospy.init_node('rb_tf_manager')
	wrt_robot_frame = ObjectArray()
	frame = import_frame(wrt_robot_frame)
	while not rospy.is_shutdown():
		frame.rb_to_tf()
		frame.object_to_tf()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass