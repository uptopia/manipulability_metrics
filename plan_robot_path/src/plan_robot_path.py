#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import tf
# import tf.transformations as tr

import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg

# import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

import numpy as np

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker


import tf2_ros
import tf2_sensor_msgs      #do_transform_cloud
import tf2_geometry_msgs    #do_transform_pose

import message_filters

# import std_msgs.msg
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

# import yaml

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class PlanRobotPath(object):

    """PlanRobotPath"""

    def __init__(self):
        super(PlanRobotPath, self).__init__()

        print('=========PlanTraj_for_DLO_GraspPoseInBase=======')

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("PlanTraj_for_DLO_GraspPoseInBase", anonymous=True)
        
        self.listener = tf.TransformListener()

        self.robot = moveit_commander.RobotCommander()

        self.group_name = rospy.get_param('/plan_robot_path/group_name')#"blue_arm" #"panda_arm"
        self.base_frame = rospy.get_param('/plan_robot_path/base_frame')#"base_link"
        self.cam_frame = rospy.get_param('/plan_robot_path/cam_frame')#"camera_color_optical_frame"
        print(self.group_name, self.base_frame, self.cam_frame)
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        dlo_cloudInCam_sub = message_filters.Subscriber('/dlo_cloudInCam', PointCloud2)
        dlo_graspInCam_sub = message_filters.Subscriber('/dlo_graspInCam', PoseStamped)

        ts = message_filters.ApproximateTimeSynchronizer([dlo_cloudInCam_sub, dlo_graspInCam_sub], queue_size=10, slop=10, allow_headerless=False)
        ts.registerCallback(self.callback)

        self.dlo_cloudInBase_pub = rospy.Publisher("/dlo_cloudInBase", PointCloud2, queue_size=2)

        self.dlo_graspInBase_br = tf.TransformBroadcaster()


        ###############
        # fixed take pic pose
        # base_H_cam
        ###############
        not_arrived = True
        self.base_H_cam=[]
        while not_arrived:
            self.move_group.set_named_target("take_pic")
            success = self.move_group.go()

            if success:
                print("success")
                self.move_group.stop()
                self.base_H_cam = self.tf_buffer.lookup_transform(self.base_frame, self.cam_frame, rospy.Time(), rospy.Duration(0.1))
                print('base_H_cam:', self.base_H_cam)
                break
            else:
                print("failed")

        rospy.spin()

    def callback(self, dlo_cloudInCam_msg, dlo_graspInCam_msg):
        print('base_H_cam:', self.base_H_cam)

        # ###############
        # # base_H_cam
        # ###############
        # base_H_cam = self.tf_buffer.lookup_transform(self.base_frame, self.cam_frame, rospy.Time(), rospy.Duration(0.1))
        # print('base_H_cam:', base_H_cam)

        ###############
        # cloud_inCam -->
        # cloud_inBase
        ###############
        dlo_cloudInBase_msg = tf2_sensor_msgs.do_transform_cloud(dlo_cloudInCam_msg, self.base_H_cam)
        self.dlo_cloudInBase_pub.publish(dlo_cloudInBase_msg)

        ###############
        # grasp_inCam -->
        # grasp_inBase
        ###############
        dlo_graspInBase_msg = tf2_geometry_msgs.do_transform_pose(dlo_graspInCam_msg, self.base_H_cam)
        self.dlo_graspInBase_br.sendTransform((dlo_graspInBase_msg.pose.position.x, dlo_graspInBase_msg.pose.position.y, dlo_graspInBase_msg.pose.position.z),
                                            (dlo_graspInBase_msg.pose.orientation.x, dlo_graspInBase_msg.pose.orientation.y, dlo_graspInBase_msg.pose.orientation.z, dlo_graspInBase_msg.pose.orientation.w),
                                            dlo_graspInCam_msg.header.stamp,
                                            "dlo_graspInBase_frame", 
                                            self.base_frame)
        
        ###############
        # plan robot
        # grasp trajectory
        ###############

        self.move_group.set_pose_target(dlo_graspInBase_msg)
        plan = self.move_group.plan()
        print("plan: ", type(plan))
        print("trajectory points:", type(plan[0]), type(plan[1]), type(plan[2]))

        if plan[1].joint_trajectory.points:  # True if trajectory contains points
            print(len(plan[1].joint_trajectory.points))
            print('plan points:\n', type(plan[1].joint_trajectory.points[0].positions),plan[1].joint_trajectory.points[0].positions)
            move_success = self.move_group.execute(plan[1])#, wait=True)
            print('move_success:', move_success)

            # manip_list = []
            # for m in range(len(plan[1].joint_trajectory.points)):
            #     joint_angles = list(plan[1].joint_trajectory.points[m].positions)
            #     # print('robot jacobian: \n', self.move_group.get_jacobian_matrix(joint_angles))
            #     # https://github.com/robotlearn/pyrobolearn/blob/9cd7c060723fda7d2779fa255ac998c2c82b8436/pyrobolearn/robots/robot.py#L3302
            #     jacob = self.move_group.get_jacobian_matrix(joint_angles)
            #     manip = np.linalg.det(jacob.dot(jacob.T))**0.5
            #     manip_list.append(manip)
            #     print('manip:', m, manip)

            #     ## joint angle to end effector pose
            #     # pt = geometry_msgs.msg.Point()
            #     # pt.x = ?
            #     # pt.y = 
            #     # pt.z = 
            #     # traject_marker.points.ap？end(pt)

            # x_axis = [val for val in range(len(plan[1].joint_trajectory.points))]
            # plt.title("manipulability")
            # plt.xlabel("robot pose num")
            # plt.ylabel("manip value")
            # plt.plot(x_axis, manip_list, 'b-o')
            # plt.show()
        else:
            rospy.logerr("Trajectory is empty. Planning was unsuccessful.")

        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose

        #==================#
        #  Jacobian beform 
        #  move to target
        #==================#
        # robot_current_state = self.robot.get_current_state()
        # joint_values_new = self.move_group.get_current_joint_values()
        # # print('type(joint_values_new):', type(joint_values_new))
        # # print("robot current joints: \n", joint_values_new)
        # # print('robot jacobian: \n', self.move_group.get_jacobian_matrix(joint_values_new))
        # # print("robot current pose: \n", self.move_group.get_current_pose().pose)

        return all_close(dlo_graspInBase_msg.pose, current_pose, 0.01)

if __name__ == "__main__":
    try:
        PlanRobotPath()
    except rospy.ROSInterruptException:
        pass