#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import open3d as o3d

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

import std_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ctypes import * # convert float to uint32

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
## END_SUB_TUTORIAL
# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

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

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
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

        print('=========plan_robot_path=======')

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("plan_robot_path", anonymous=True)
        
        self.listener = tf.TransformListener()

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        rospy.Subscriber("/dlo_cloud", PointCloud2, self.dlo_cloud_cb)
        rospy.Subscriber("/dlo_grasp", geometry_msgs.msg.Pose, self.plan_robot_path_cb)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.dlo_cloud_inRobot_pub = rospy.Publisher("/dlo_cloud_inRobot", PointCloud2, queue_size=2)

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        print("============ Planning frame: %s" % planning_frame)
        print("============ End effector link: %s" % eef_link)
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.base_H_DLOobj = np.identity(4)

        rospy.spin()

    def convertCloudFromRosToOpen3d(self, ros_cloud):
        #https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/master/lib_cloud_conversion_between_Open3D_and_ROS.py
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(point_cloud2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        # Check empty
        open3d_cloud = o3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # combine
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud
    
    # Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
    def convertCloudFromOpen3dToRos(self, open3d_cloud, frame_id="odom"):
        # Set "header"
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        # Set "fields" and "cloud_data"
        points=np.asarray(open3d_cloud.points)

        #testing
        # print("cloud field: ", FIELDS_XYZ)
        fields=FIELDS_XYZ
        cloud_data=points
        # print("cloud_data:\n", cloud_data)

        ## original
        # if not open3d_cloud.colors: # XYZ only
        #     print("cloud field: ", FIELDS_XYZ)
        #     fields=FIELDS_XYZ
        #     cloud_data=points
        # else: # XYZ + RGB
        #     print("cloud field: ", FIELDS_XYZRGB)
        #     fields=FIELDS_XYZRGB
        #     # -- Change rgb color from "three float" to "one 24-byte int"
        #     # 0x00FFFFFF is white, 0x00000000 is black.
        #     colors = np.floor(np.asarray(open3d_cloud.colors)*255)                        # nx3 matrix (3 float)
        #     colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]    # 1 24-byte int
        #     cloud_data=np.c_[points, colors] #concatenate by columns
        #     print("cloud_data:", cloud_data)

        # create ros_cloud
        return point_cloud2.create_cloud(header, fields, cloud_data)

    def dlo_cloud_cb(self, dlo_cloud_msg):
        # assert isinstance(dlo_cloud_msg, PointCloud2)
        # print("self.base_H_DLOobj == np.identity(4):", np.array_equal(self.base_H_DLOobj, np.identity(4)))
        if np.array_equal(self.base_H_DLOobj, np.identity(4)):
            print("base_H_DLOobj is identity matrix")
        else:
            print(" dlo_cloud to dlo_cloud_inRobot")
            # print("base_H_DLOobj:\n", self.base_H_DLOobj)
            # dlo_cloud_gen = point_cloud2.read_points_list(dlo_cloud_msg, skip_nans=False)
            o3d_dlo_cloud = self.convertCloudFromRosToOpen3d(dlo_cloud_msg)
            o3d_dlo_cloud_inRobot = o3d_dlo_cloud.transform(np.identity(4))#self.base_H_DLOobj)
            # print(o3d_dlo_cloud.colors)
            # print(np.asarray(o3d_dlo_cloud_inRobot.points))
            ros_dlo_cloud_inRobot = self.convertCloudFromOpen3dToRos(o3d_dlo_cloud_inRobot, frame_id='panda_link0')
            # print(type(dlo_cloud_msg),type(o3d_dlo_cloud),type(self.base_H_DLOobj),type(o3d_dlo_cloud_inRobot), type(ros_dlo_cloud_inRobot))
            self.dlo_cloud_inRobot_pub.publish(ros_dlo_cloud_inRobot)

    def plan_robot_path_cb(self, pose_msg):
        print('=================')
        #quaternion rotation
        # https://wiki.ros.org/tf2/Tutorials/Quaternions
        # https://blog.csdn.net/wjrzm2001/article/details/129160906

        #==================#
        # base_H_endeffect
        # base_H_cam
        #==================#
        #https://stackoverflow.com/questions/54384021/transforming-pose-with-tf-listener-is-not-working-in-rviz
        # rosrun rqt_tf_tree rqt_tf_tree
        # rosrun tf view_frames
        try:
            (trans, quaternion) = self.listener.lookupTransform('/panda_link0', '/panda_link7', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("not working")
            pass

        # print('trans:\n', trans)
        # print('quaternion:\n', quaternion, type(quaternion))

        base_H_cam = tf.transformations.quaternion_matrix(quaternion)#tf.fromTranslationRotation(trans, quaternion)
        base_H_cam[0][3] = trans[0]
        base_H_cam[1][3] = trans[1]
        base_H_cam[2][3] = trans[2]
        # print("base_H_cam:\n", base_H_cam)

        #https://robotics.stackexchange.com/questions/99136/transformation-matrices-to-geometry-msgs-pose
        # cur_matrix = matrix.reshape(3,4)
        # cur_matrix_homo = np.vstack((cur_matrix, np.array([0, 0, 0, 1]))) # to homogenous coordinates

        # q = tf.transformations.quaternion_from_matrix(cur_matrix_homo)

        # p = Pose()
        # p.position.x = matrix[0][3]
        # p.position.y = matrix[1][3]
        # p.position.z = matrix[2][3]
        # p.orientation.x = q[0]
        # p.orientation.y = q[1]
        # p.orientation.z = q[2]
        # p.orientation.w = q[3]

        #==================#
        #   cam_H_DLOobj
        #==================#
        # print("cam_H_DLOobj:\n")
        print(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z)
        print(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w)
        print(type(pose_msg.orientation))
    
        cam_H_DLOobj = tf.transformations.quaternion_matrix([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])#tf.transformations.fromTranslationRotation(pose_msg.position, pose_msg.orientation)
        cam_H_DLOobj[0][3] = pose_msg.position.x
        cam_H_DLOobj[1][3] = pose_msg.position.y
        cam_H_DLOobj[2][3] = pose_msg.position.z
        # print("cam_H_DLOobj:\n", cam_H_DLOobj)

        #==================#
        #   base_H_DLOobj
        #==================#
        # https://gist.github.com/lucascoelhof/b40c3f56080d789bf0623843af10f752
        # https://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
        # https://wiki.ros.org/tf/TfUsingPython

        base_H_DLOobj = base_H_cam*cam_H_DLOobj
        # print("base_H_DLOobj:\n", base_H_DLOobj)
        self.base_H_DLOobj = base_H_DLOobj

        dlo_x = base_H_DLOobj[0][3]
        dlo_y = base_H_DLOobj[1][3]
        dlo_z = base_H_DLOobj[2][3]
        dlo_rot_quat = tf.transformations.quaternion_from_matrix(base_H_DLOobj)
        # print('dlo_rot_quat:', dlo_rot_quat)

        dlo_rot_quat_norm = dlo_rot_quat/np.sqrt(np.dot(dlo_rot_quat,dlo_rot_quat))
        # print('dlo_rot_quat normalized:', dlo_rot_quat_norm)
        dlo_rot_x = dlo_rot_quat_norm[0]
        dlo_rot_y = dlo_rot_quat_norm[1]
        dlo_rot_z = dlo_rot_quat_norm[2]
        dlo_rot_w = dlo_rot_quat_norm[3]

        # add a dlo_obj frame
        # https://blog.csdn.net/Will_Ye/article/details/123035136
        # invalid quaternion in the transform (0.481047 0.508212 0.241462 0.237747)
        br = tf.TransformBroadcaster()
        br.sendTransform((dlo_x, dlo_y, dlo_z), 
                         (dlo_rot_x, dlo_rot_y, dlo_rot_z, dlo_rot_w),
                                     rospy.Time.now(),
                                     'dlo_obj',
                                     'panda_link8')
        
        tmp_trans, tmp_rot = self.listener.lookupTransform("/dlo_obj", "/panda_link8", rospy.Time(0))
        # print('tmp_trans, tmp_rot:\n', tmp_trans, tmp_rot)


        #==================#
        #  Jacobian beform 
        #  move to target
        #==================#
        joint_values = self.move_group.get_current_joint_values()
        # print("robot current joints: \n", joint_values)
        # print('robot jacobian: \n', self.move_group.get_jacobian_matrix(joint_values))
        # print("robot current pose: \n", self.move_group.get_current_pose().pose)

        print('move to grasp pose...\n')
        print(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z)
        print(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w)
        self.move_group.set_pose_target(pose_msg)
        plan = self.move_group.plan()
        print("plan: ", type(plan))
        print("trajectory points:", type(plan[0]), type(plan[1]), type(plan[2]))

        if plan[1].joint_trajectory.points:  # True if trajectory contains points
            print(len(plan[1].joint_trajectory.points))
            print('plan points:\n', type(plan[1].joint_trajectory.points[0].positions),plan[1].joint_trajectory.points[0].positions)
            move_success = self.move_group.execute(plan[1])
            print('move_success:', move_success)
            for m in range(len(plan[1].joint_trajectory.points)):
                joint_angles = list(plan[1].joint_trajectory.points[m].positions)
                # print('robot jacobian: \n', self.move_group.get_jacobian_matrix(joint_angles))
                # https://github.com/robotlearn/pyrobolearn/blob/9cd7c060723fda7d2779fa255ac998c2c82b8436/pyrobolearn/robots/robot.py#L3302
                jacob = self.move_group.get_jacobian_matrix(joint_angles)
                manip = np.linalg.det(jacob.dot(jacob.T))**0.5
                print('manip:', m, manip)

        else:
            rospy.logerr("Trajectory is empty. Planning was unsuccessful.")

        # ## Now, we call the planner to compute the plan and execute it.
        # # `go()` returns a boolean indicating whether the planning and execution was successful.
        # success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose

        #==================#
        #  Jacobian beform 
        #  move to target
        #==================#
        joint_values_new = self.move_group.get_current_joint_values()
        print('type(joint_values_new):', type(joint_values_new))
        # print("robot current joints: \n", joint_values_new)
        # print('robot jacobian: \n', self.move_group.get_jacobian_matrix(joint_values_new))
        # print("robot current pose: \n", self.move_group.get_current_pose().pose)

        return all_close(pose_msg, current_pose, 0.01)


    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose

        print('finish plan_robot_path')
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

# def main():
#     try:
#         print("")
#         print("----------------------------------------------------------")
#         print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
#         print("----------------------------------------------------------")
#         print("Press Ctrl-D to exit at any time")
#         print("")
#         input(
#             "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
#         )
#         tutorial = MoveGroupPythonInterfaceTutorial()

#         input(
#             "============ Press `Enter` to execute a movement using a joint state goal ..."
#         )
#         tutorial.go_to_joint_state()

#         input("============ Press `Enter` to execute a movement using a pose goal ...")
#         tutorial.go_to_pose_goal()

#         input("============ Press `Enter` to plan and display a Cartesian path ...")
#         cartesian_plan, fraction = tutorial.plan_cartesian_path()

#         input(
#             "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
#         )
#         tutorial.display_trajectory(cartesian_plan)

#         input("============ Press `Enter` to execute a saved path ...")
#         tutorial.execute_plan(cartesian_plan)


#     except rospy.ROSInterruptException:
#         return
#     except KeyboardInterrupt:
#         return


if __name__ == "__main__":
    try:
        PlanRobotPath()
    except rospy.ROSInterruptException:
        pass