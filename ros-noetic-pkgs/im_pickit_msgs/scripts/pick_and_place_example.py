#!/usr/bin/env python3
# Copyright (c) 2021, Pick-it NV.
# All rights reserved.
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose, Point
from im_pickit_msgs.msg import ObjectArray
from im_pickit_msgs.srv import LoadConfig, LoadConfigRequest, CaptureImage, ProcessImage
from multiprocessing.pool import ThreadPool

def trigger_detection(thread_pool):
    """Captures an image and triggers image processing."""
    response = capture_image_srv()
    assert (response.status == response.STATUS_SUCCESS), "Failed to capture image."

    # Trigger non-blocking image processing, so the robot can move while
    # the captured image is being processed.
    return thread_pool.apply_async(process_image_srv)

def collect_detection_results(detection_handle):
    """Waits for detection to finish and returns the results."""
    response = detection_handle.get()
    return response.objects

def retrieve_first_object_pose(detection_result):
    """Retrieves the first object pose and transforms it to the pickit/robot_base frame."""
    first_object = detection_result.objects[0]
    object_pick_pose = first_object.object_pick_tf
    pickit_ref_T_object = PoseStamped(
        header=object_pick_pose.header,
        pose=Pose(
            position=Point(
                x=object_pick_pose.transform.translation.x,
                y=object_pick_pose.transform.translation.y,
                z=object_pick_pose.transform.translation.z),
            orientation=object_pick_pose.transform.rotation
        )
    )
    # Transform object pose from pickit/reference frame to pickit/robot_base frame.
    robot_base_T_object = tf_buffer.transform(pickit_ref_T_object, "pickit/robot_base")
    return robot_base_T_object

def pick(object_pose):
    """Pick the object with the robot."""
    pass

def goto_detection():
    """Move robot in detection position."""
    pass

def place():
    """Place picked object."""
    pass

if __name__ == "__main__":
    """
    The following example implements the logic described in the simple pick and place program.
    Detections are performed asynchronously, so the robot can move while waiting for detection
    results. Furthermore, object poses are transformed into the robot base frame for easier
    usage with other libraries e.g. MoveIt.
    """
    rospy.init_node("pickit_client")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    thread_pool = ThreadPool(processes=1)

    # Initialize ROS services.
    load_product_srv = rospy.ServiceProxy("/pickit/configuration/product/load", LoadConfig)
    load_setup_srv = rospy.ServiceProxy("/pickit/configuration/setup/load", LoadConfig)
    capture_image_srv = rospy.ServiceProxy("/pickit/capture_image", CaptureImage)
    process_image_srv = rospy.ServiceProxy("/pickit/process_image", ProcessImage)
    load_product_srv.wait_for_service()
    load_setup_srv.wait_for_service()
    capture_image_srv.wait_for_service()
    process_image_srv.wait_for_service()

    # Load initial configuration by their product & setup name.
    product_config = "cylinder"
    setup_config = "application_01"
    response = load_product_srv(LoadConfigRequest.METHOD_LOAD_BY_CONFIG_NAME, product_config, 0, True)
    assert response.success, "Failed to load product config '{}'.".format(product_config)
    response = load_setup_srv(LoadConfigRequest.METHOD_LOAD_BY_CONFIG_NAME, setup_config, 0, True)
    assert response.success, "Failed to load setup config '{}'.".format(setup_config)

    # Trigger detection and wait for detection result
    goto_detection()
    detection_handle = trigger_detection(thread_pool)
    detection_result = collect_detection_results(detection_handle)

    # Main loop: Pick and place the first detected object until no more object is detected.
    while True:
        if detection_result.status != ObjectArray.STATUS_SUCCESS \
           or not detection_result.n_valid_objects:
            rospy.loginfo("No objects found.")
            break

        rospy.loginfo("Detected {} objects".format(detection_result.n_valid_objects))

        object_pose = retrieve_first_object_pose(detection_result)
        pick(object_pose)
        goto_detection()
        detection_handle = trigger_detection(thread_pool)
        place()

        detection_result = collect_detection_results(detection_handle)
