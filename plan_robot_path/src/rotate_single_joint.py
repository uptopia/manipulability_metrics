#!/usr/bin/env python3

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from math import pi
import math
import PyKDL
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
# import geometry_msgs
import tf

epsilon = sys.float_info.epsilon
# print("\033[33mThe value of epsilon is:", epsilon)

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class RotateSingleJoint():
    """RotateSingleJoint"""

    def __init__(self):
        super(RotateSingleJoint, self).__init__()

        print('\033[33m=========RotateSingleJoint=======')
        rospy.init_node("RotateSingleJoint", anonymous=True)

        try:
            rospy.loginfo("waiting for move_group action server")
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot = moveit_commander.RobotCommander()
        except RuntimeError:
            rospy.loginfo("could not connect to MoveIt in time, exiting")
            rospy.signal_shutdown('fatal error')        

        self.group_name = rospy.get_param('/rotate_single_joint/group_name')#"manipulator"
        self.base_frame = rospy.get_param('/rotate_single_joint/base_frame')#"base_link"
        print("\033[33mself.group_name, self.base_frame:", self.group_name, self.base_frame)

        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # set parameters
        self.move_group.set_pose_reference_frame(self.base_frame)
        self.ee_link = self.move_group.get_end_effector_link()
        self.move_group.set_end_effector_link(self.ee_link)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        self.move_group.set_max_velocity_scaling_factor(0.1)

        # self.joint_traj_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
        
        # print("\033[33m============ Planning frame: {}".format(self.move_group.get_planning_frame()))       #world
        # print("\033[33m============ End effector link: {}".format(self.move_group.get_end_effector_link())) #tool0
        # print("\033[33m============ Available Planning Groups: {}".format(self.robot.get_group_names()))    #manipulator
        # print("\033[33m============ Printing robot state: {}".format(self.robot.get_current_state()))

        # # self.send_traj()

        self.robot_chain = self.create_ur5e_chain()

        # self.manip_ellipsoid_list = []
        # self.manip_score_list = []

        # self.go_to_joint_state()
        # # mscore_name ='m1'
        # # mscore_list = [1,2,3,4,5,6,7,8]
        # # self.plot_mscore(mscore_name, mscore_list)
        self.run_pose()
        
    def run_pose(self):
        self.back_up()
        # self.back_home()
        # self.rotate_joint(4,-pi/4, pi/4)
        # self.pose()
        # self.pose1()
        # self.rotate_joint(4,-pi/4, pi/4)
        # self.pose2()
        # print("\033[33m~~~~~~~~~~~~~~~~~~~", type(self.move_group.get_remembered_joint_values), dir(self.move_group.get_remembered_joint_values))
    def pose(self):
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = -pi * 0.5
        joint_goal[1] = -pi * 0.5
        joint_goal[2] = -pi * 0.5
        joint_goal[3] = -pi * 0.5
        joint_goal[4] = pi * 0.5
        joint_goal[5] = pi * 0.5    
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_joints = self.move_group.get_current_joint_values()
        origin_orientation =  self.move_group.get_current_pose().pose.orientation
        origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

        # self.origin_degree[0] = origindegree[0]/3.14*180.0
        # self.origin_degree[1] = origindegree[1]/3.14*180.0
        # self.origin_degree[2] = origindegree[2]/3.14*180.0
        return all_close(joint_goal, current_joints, 0.01)
    
    def pose1(self):
        pose_goal = Pose()
        pose_goal.position.x = 0.11096
        pose_goal.position.y = 0.51508   
        pose_goal.position.z = 0.5002
        pose_goal.orientation.x = -0.06103
        pose_goal.orientation.y = 0.70449
        pose_goal.orientation.z = -0.06157
        pose_goal.orientation.w = 0.70439
        self.move_group.set_pose_target(pose_goal, self.ee_link)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def pose2(self):
        pose_goal = Pose()
        pose_goal.position.x = 0.11096
        pose_goal.position.y = 0.46508
        pose_goal.position.z = 0.5002
        
        quaternion = quaternion_from_euler(np.radians(0),np.radians(90.), np.radians(0))   #roll_angle, pitch_angle, yaw_angle  
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        self.move_group.set_pose_target(pose_goal, self.ee_link)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def rotate_joint(self, axis_num, angle_min, angle_max):
        # curr_joints = self.move_group.get_current_joint_values()
        start_joints = self.move_group.get_current_joint_values()
        end_joints = self.move_group.get_current_joint_values()
        start_joints[4] = angle_min
        end_joints[4] = angle_max

        print("\033[33mstart_joints:", start_joints)
        print("\033[33mend_joints:", end_joints)
        # self.move_group.set_pose_target(start_joints)
        self.move_group.set_named_target("home") #home, up
        traj1 = self.move_group.plan()

        print("\033[33m============ Visualizing traj1")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(traj1)
        self.display_trajectory_publisher.publish(display_trajectory)

        print("\033[33m============ Waiting while plan1 is visualized (again)...")
        rospy.sleep(5)
        self.move_group.go()
        # self.move_group.go(start_joints, wait=True)
        self.move_group.stop()

        # self.move_group.go(end_joints, wait=True)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()

    # def pose(self):
    #     group = self.group
    #     group.set_max_acceleration_scaling_factor(0.1)
    #     group.set_max_velocity_scaling_factor(0.1)
    #     joint_goal = group.get_current_joint_values()
    #     joint_goal[0] = -pi * 0.5
    #     joint_goal[1] = -pi * 0.5
    #     joint_goal[2] = -pi * 0.5
    #     joint_goal[3] = -pi * 0.5
    #     joint_goal[4] = pi * 0.5
    #     joint_goal[5] = pi * 0.5    
    #     group.go(joint_goal, wait=True)
    #     group.stop()
    #     group.clear_pose_targets()
    #     current_joints = group.get_current_joint_values()
    #     origin_orientation =  group.get_current_pose().pose.orientation
    #     origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

        
    #     self.origin_degree[0] = origindegree[0]/3.14*180.0
    #     self.origin_degree[1] = origindegree[1]/3.14*180.0
    #     self.origin_degree[2] = origindegree[2]/3.14*180.0
    #     return all_close(joint_goal, current_joints, 0.01)
        
    def back_up(self):
        # https://robot.czxy.com/docs/kinematics/moveit/goal/move/
        prev_joints_radian = self.move_group.get_current_joint_values()
        prev_joints_degree = self.move_group.get_current_joint_values()
        # self.move_group.set_named_target("home") #home, up
        # self.move_group.go()

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = -pi * 0.5
        joint_goal[1] = -pi * 0.5
        joint_goal[2] = -pi * 0.5
        joint_goal[3] = -pi * 0.5
        joint_goal[4] = pi * 0.5
        joint_goal[5] = pi * 0.5    
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        #====================
        #  Jacobian Matrix
        #====================
        curr_joints_radian = self.move_group.get_current_joint_values()
        curr_joints_degree = self.move_group.get_current_joint_values()
        for id in range(len(curr_joints_radian)):
            curr_joints_degree[id]=curr_joints_radian[id]*(180.0/pi)
            prev_joints_degree[id]=prev_joints_radian[id]*(180.0/pi)
        # # print("\033[33mprev_joints: {:8.2f}, \ncurr_joints: {:8.2f}".format(prev_joints, curr_joints))
            
        print("\033[33mprev_joints_degree:",', '.join('{:.2f}'.format(f) for f in prev_joints_degree))
        print("\033[33mcurr_joints_degree:",', '.join('{:.2f}'.format(f) for f in curr_joints_degree))
        print("\033[33mjoint_goal:",', '.join('{:.2f}'.format(f*(180.0/pi)) for f in joint_goal))

        matrix = np.array(self.move_group.get_jacobian_matrix(curr_joints_radian)) #use radian
        print("\033[33mjacobian matrix:\n", matrix)
                
        num_joints =6
        joint_angles_array = PyKDL.JntArray(num_joints)
        for idx in range(num_joints):
            joint_angles_array[idx] = curr_joints_radian[idx]       #use radian
        jac = self.calculateJacobian(self.robot_chain, joint_angles_array) #np.matrix #use degree
        jac_mat = self.kdl_to_mat(jac)
        print("\033[33mjacobian matrix PyKDL:\n", jac_mat)

        ellipsoid_1 = math.sqrt(np.linalg.det((matrix * matrix.transpose())))
        print("\033[33mellipsoid 1:", ellipsoid_1)
        ellipsoid_2 = math.sqrt(np.linalg.det((jac_mat * jac_mat.transpose())))
        print("\033[33mellipsoid 1, 2: ", ellipsoid_1, ellipsoid_2)

        #====================
        #  Position
        #====================
        pp = self.move_group.get_current_pose().pose.position
        rr = self.move_group.get_current_pose().pose.orientation
        print("\033[33mposition:", pp.x, pp.y, pp.z)#type(pp), pp)
        print("\033[33morientation:",rr.x, rr.y, rr.z, rr.w)#type(rr), rr)
        print("\033[33mframd_id:", self.move_group.get_current_pose().header.frame_id)

        # now = rospy.Time.now() #specific time, rospy.Time(0) recent time
        # https://blog.csdn.net/qq_39779233/article/details/105478695
        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform("/base_link", "/tool0", rospy.Time(0), rospy.Duration(4.0))
        (trans,rot) = tf_listener.lookupTransform("/base_link", "/tool0", rospy.Time(0))
        print("\033[33mtrans: {},\nrot:: {}".format(trans, rot))

        # pp.x, pp.y, pp.z = trans[0], trans[1], trans[2]
        # rr.x, rr.y, rr.z, rr.w = rot[0], rot[1], rot[2], rot[3]
        # print("\033[33mposition:", pp.x, pp.y, pp.z)#type(pp), pp)
        # print("\033[33morientation:",rr.x, rr.y, rr.z, rr.w)#type(rr), rr)

        euler = tf.transformations.euler_from_quaternion((rr.x, rr.y, rr.z, rr.w))
        roll, pitch, yaw = euler[0], euler[1], euler[2]
        
        joint_angles1 = self.compute_IK_from_rpy(self.robot_chain, pp, euler)
        print("\033[33mjoint_angles1:",', '.join('{:.2f}'.format(180.0*(f/pi)) for f in joint_angles1))

        joint_angles2 = self.compute_IK_from_quat(self.robot_chain, pp, rr)
        print("\033[33mjoint_angles2:",', '.join('{:.2f}'.format(180.0*(f/pi)) for f in joint_angles2))

        # origin_degree =  euler_from_quaternion([rr.x, rr.y, rr.z, rr.w]) 
        # # print("\033[33morigindegree:", origin_degree, roll, pitch, yaw)
        # joint_angles3 = self.compute_IK_from_rpy(self.robot_chain, pp, origin_degree)
        # print("\033[33mjoint_angles3:",', '.join('{:.2f}'.format(180.0*(f/pi)) for f in joint_angles3))

        #=====
        curr_joints_radian = self.move_group.get_current_joint_values()
        curr_joints_degree = self.move_group.get_current_joint_values()
        for id in range(len(curr_joints_radian)):
            curr_joints_degree[id]=curr_joints_radian[id]*(180.0/pi)
            prev_joints_degree[id]=prev_joints_radian[id]*(180.0/pi)
        # # print("\033[33mprev_joints: {:8.2f}, \ncurr_joints: {:8.2f}".format(prev_joints, curr_joints))
            
        print("\033[33mprev_joints_degree:",', '.join('{:.2f}'.format(f) for f in prev_joints_degree))
        print("\033[33mcurr_joints_degree:",', '.join('{:.2f}'.format(f) for f in curr_joints_degree))


    def back_home(self):
        # https://robot.czxy.com/docs/kinematics/moveit/goal/move/
        prev_joints = self.move_group.get_current_joint_values()
        self.move_group.set_named_target("home") #home, up
        success = self.move_group.go()
        # print("\033[33msuccess:",success)
        # if success:
        #     print("\033[33msuccess")
        # else:
        #     print("\033[33mfailed")
        self.move_group.stop()

        curr_joints = self.move_group.get_current_joint_values()
        print("\033[33mprev_joints, curr_joints:", prev_joints, curr_joints)

    def plot_mscore(self, mscore_name, mscore_list):

        xpoints = np.arange(0, len(mscore_list),1)
        ypoints = np.array(mscore_list)

        plt.title("mscore: {}".format(mscore_name))
        plt.xlabel('time',{'fontsize':10,'color':'red'})
        plt.ylabel('mscore',{'fontsize':10,'color':'green'})
        plt.plot(xpoints, ypoints)
        plt.show()

    def create_ur5_chain(self):
        # dh_parameters:
        # d1: 0.089159
        # a2: -0.42500
        # a3: -0.39225
        # d4: 0.10915
        # d5: 0.09465
        # d6: 0.0823

        # # DH parameters
        # a = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
        # alpha = [math.pi/2, 0.0, 0.0, math.pi/2, -math.pi/2, 0.0]
        # d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
        # theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # chain = PyKDL.Chain()

        # for i in range(6):
        #     chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ),\
        #                     PyKDL.Frame(PyKDL.Rotation.RotZ(theta[i])*PyKDL.Rotation.RotX(alpha[i]), \
        #                                 PyKDL.Vector(a[i], -d[i]*math.sin(alpha[i]), d[i]*math.cos(alpha[i])))))
        
        #================#
        #    method2
        #================#
        chain = PyKDL.Chain()
        dh_params = [
            # a,       alpha,        d,     theta
            (0.0,      0.5*math.pi,  0.089159, 0.0),
            (-0.42500, 0.0,          0.0,      0.0),
            (-0.39225, 0.0,          0.0,      0.0),
            (0.0,      0.5*math.pi,  0.10915,  0.0),
            (0.0,      -0.5*math.pi, 0.09465,  0.0),
            (0.0,      0.0,          0.0823,   0.0),
        ]
        for a, alpha, d, theta in dh_params:
            # frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, alpha, theta),
            #                     PyKDL.Vector(a, 0, d))
            print(a, alpha, d, theta)
            
            frame = PyKDL.Frame()
            frame = frame.DH(a, alpha, d, theta)
            joint = PyKDL.Joint(PyKDL.Joint(PyKDL.Joint.RotZ))
            segment= PyKDL.Segment(joint, frame)
            chain.addSegment(segment)
        
        
        return chain

    def create_ur5e_chain(self):
        # dh_parameters:
        # d1: 0.163
        # a2: -0.42500
        # a3: -0.39225
        # d4: 0.134         # wrist1_length = d4 - elbow_offset - shoulder_offset 
        # d5: 0.100
        # d6: 0.100
        # DH parameters
        # a = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
        # alpha = [math.pi/2, 0.0, 0.0, math.pi/2, -math.pi/2, 0.0]
        # d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
        # theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # #================#
        # #    method1
        # #================#
        # chain = PyKDL.Chain()

        # # DH Parameters
        # a = [0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0]
        # alpha = [math.pi/2, 0.0, 0.0, math.pi/2, -math.pi/2, 0.0]
        # d = [0.163, 0.0, 0.0, 0.134, 0.100, 0.100]
        # theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # for i in range(6):
        #     chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ),\
        #                     PyKDL.Frame(PyKDL.Rotation.RotZ(theta[i])*PyKDL.Rotation.RotX(alpha[i]), \
        #                                 PyKDL.Vector(a[i], -d[i]*math.sin(alpha[i]), d[i]*math.cos(alpha[i])))))
        
        #================#
        #    method2
        #================#
        chain = PyKDL.Chain()
        dh_params = [
            # a,       alpha,        d,     theta
            (0.0,      0.5*math.pi,  0.163, math.pi),
            (-0.425,   0.0,          0.0,   0.0),
            (-0.39225, 0.0,          0.0,   0.0),
            (0.0,      0.5*math.pi,  0.134, 0.0),
            (0.0,      -0.5*math.pi, 0.100, 0.0),
            (0.0,      0.0,          0.100, 0.0),
        ]
        for a, alpha, d, theta in dh_params:
            # frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, alpha, theta),
            #                     PyKDL.Vector(a, 0, d))
            print(a, alpha, d, theta)
            
            frame = PyKDL.Frame()
            frame = frame.DH(a, alpha, d, theta)
            joint = PyKDL.Joint(PyKDL.Joint(PyKDL.Joint.RotZ))
            segment= PyKDL.Segment(joint, frame)
            chain.addSegment(segment)
        
        return chain

    def joint_list_to_kdl(self, q):
        if q is None:
            return None
        if type(q) == np.matrix and q.shape[1] == 0:
            q = q.T.tolist()[0]
        q_kdl = PyKDL.JntArray(len(q))
        for i, q_i in enumerate(q):
            q_kdl[i] = q_i
        return q_kdl

    def kdl_to_mat(self, m):
        mat =  np.mat(np.zeros((m.rows(), m.columns())))
        for i in range(m.rows()):
            for j in range(m.columns()):
                mat[i,j] = m[i,j]
        return mat
    
    # def compute_FK_from_joint(self, chain, joints):
    #     '''Forward Kinematics'''
    #     fk = PyKDL.ChainFkSolverPos_recursive(chain)

    #     result = fk.JntToCart()


    #     endeffec_frame = PyKDL.Frame()
    #     kinematics_status = self._fk_kdl.JntToCart(joint_list_to_kdl(q),
    #                                                endeffec_frame,
    #                                                link_number)
    #     if kinematics_status >= 0:
    #         p = endeffec_frame.p
    #         M = endeffec_frame.M
    #         return np.mat([[M[0,0], M[0,1], M[0,2], p.x()], 
    #                        [M[1,0], M[1,1], M[1,2], p.y()], 
    #                        [M[2,0], M[2,1], M[2,2], p.z()],
    #                        [     0,      0,      0,     1]])
    #     else:
    #         return None

    def compute_IK_from_rpy(self, chain, position, rpy):
        '''Inverse Kinematics'''
        target_frame = PyKDL.Frame(PyKDL.Rotation.RPY(rpy[0],rpy[1], rpy[2]),\
                                    PyKDL.Vector(position.x, position.y, position.z))
        
        joint_angles = self.compute_IK_from_frame(chain, target_frame)

        return joint_angles

    def compute_IK_from_quat(self, chain, position, quat):
        '''Inverse Kinematics'''

        target_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w),\
                                    PyKDL.Vector(position.x, position.y, position.z))

        joint_angles = self.compute_IK_from_frame(chain, target_frame)

        return joint_angles
    
    def compute_IK_from_frame(self, chain, target_frame):
        '''Inverse Kinematics'''
        # fk = PyKDL.ChainFkSolverPos_recursive(chain)
        # ikv = PyKDL.ChainIkSolverVel_pinv(chain)
        # ik = PyKDL.ChainIkSolverPos_NR(chain, fk, ikv) #ok
        ik = PyKDL.ChainIkSolverPos_LMA(chain) #ok
        # ik = PyKDL.ChainIkSolverPos(chain)  #X

        joint_angles = PyKDL.JntArray(chain.getNrOfJoints())
        result = ik.CartToJnt(PyKDL.JntArray(chain.getNrOfJoints()), target_frame, joint_angles)
        # print("compute_IK_from_frame result:", result)

        # initial_joint_angles = PyKDL.JntArray(chain.getNrOfJoints())
        # joint_angles = PyKDL.JntArray(chain.getNrOfJoints())
        # ik.CartToJnt(initial_joint_angles, target_frame, joint_angles)
        # print("\033[33mjoint_angles: ", joint_angles, type(joint_angles), list(joint_angles), type(list(joint_angles)))
        return joint_angles
    
    def calculateJacobian(self, chain, joint_angles):
        # https://github.com/wuphilipp/sawyer_kdl/blob/master/launch/sawyer_kdl_test.launch
        # https://github.com/RethinkRobotics/baxter_pykdl/blob/master/src/baxter_kdl/kdl_kinematics.py#L226
        # http://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1ChainJntToJacSolver.html#a352c173cdc62320164a1e69486591b9f/
        jac_kdl = PyKDL.ChainJntToJacSolver(chain)

        # j_kdl = PyKDL.Jacobian(joint_angles.rows())
        j_kdl = PyKDL.Jacobian(chain.getNrOfJoints())
        q_kdl = self.joint_list_to_kdl(list(joint_angles))
        jac_kdl.JntToJac(q_kdl, j_kdl)

        return j_kdl 

    def cal_joint_lmt_val(self, theta, min_deg, max_deg):

        #設定此單軸旋轉的正負極限
        tot  = max_deg-min_deg
        half = max_deg-(tot/2)
        tmp_w = (np.sign(theta-min_deg)*np.sign(max_deg-theta))-1.0
        #單位步階函數(unit step function)
        if tmp_w >= 0:
            w = 1.0
        else:
            w = 0.0
        # w = hardlim((sign(n-min_deg)*sign(max_deg-n))-1); #單位步階函數(unit step function)
        
        # 【Method 3】JL eval method another view (with abs)
        tmp = (theta-half) / (max_deg-half)
        pow = 4
        score = (1.0-tmp**pow)*w
        return score

    def m_score(self, joint_angles):

        JL_score = []
        Sshoulder_score = []
        Selbow_score = []
        Swrist_score = []
        for cnt in range(len(list(joint_angles))):
            print("\033[33mjoint #:", cnt, list(joint_angles)[cnt])
            ang = list(joint_angles)[cnt]
            
            # Compute Joint-Limits Score (JL-score)
            JL_score.append(self.cal_joint_lmt_val(ang, -360, 360))
            
            # Compute Singularity Score (S-score) Singularity Shoulder
            if cnt ==2 or cnt ==3 or cnt ==4:
                if (ang<0 and ang >= -360):      
                    min_angle, max_angle = -360, 0
                else: 
                    min_angle, max_angle = 0, 360
                val_shoulder =self.cal_joint_lmt_val(ang, min_angle, max_angle)
            else:
                val_shoulder =100
            Sshoulder_score.append(val_shoulder)

            # Compute Singularity Score (S-score) Singularity Elbow
            if cnt ==3:
                if (ang<360 and ang >= 180):      
                    min_angle, max_angle = 180, 360
                elif (ang<180 and ang >= 0):
                    min_angle, max_angle = 0, 180
                elif (ang<0 and ang >= -180):
                    min_angle, max_angle = -180, 0
                else:
                    min_angle, max_angle = -180, -360
                val_elbow =self.cal_joint_lmt_val(ang, min_angle, max_angle)
            else:
                val_elbow =100
            Selbow_score.append(val_elbow)

            # Compute Singularity Score (S-score) Singularity Wrist
            if cnt ==5:
                if (ang<360 and ang >= 180):      
                    min_angle, max_angle = 180, 360
                elif (ang<180 and ang >= 0):
                    min_angle, max_angle = 0, 180
                elif (ang<0 and ang >= -180):
                    min_angle, max_angle = -180, 0
                else:
                    min_angle, max_angle = -180, -360
                val_wrist =self.cal_joint_lmt_val(ang, min_angle, max_angle)
            else:
                val_wrist =100
            Swrist_score.append(val_wrist)
        
        # Compute Manipulability Score (M-score)
        M_score = min(min(JL_score), min(Sshoulder_score), min(Selbow_score), min(Swrist_score))
        print("\033[33mJL_score:", JL_score, min(JL_score))
        print("\033[33mSshoulder_score:", Sshoulder_score, min(Sshoulder_score))
        print("\033[33mSelbow_score:", Selbow_score, min(Selbow_score))
        print("\033[33mSwrist_score:", Swrist_score, min(Swrist_score))
        print("\033[33mM_score:", M_score)
        
        return M_score

    def calculate_mscore_from_joints(self, joint_angles):

        # # Set Target Pose
        # target_pose = [pp[0], pp[1], pp[2], rr[0], rr[1], rr[2]]

        # # Compute IK
        # joint_angles = self.compute_IK(self.robot_chain, target_pose)
        # # print("\033[33mjoint angles:", joint_angles)

        joint_angles_array = PyKDL.JntArray(len(joint_angles))
        # joint_angles_array.rows = joint_angles
        jac = self.calculateJacobian(self.robot_chain, joint_angles_array) #np.matrix

        manip_ellipsoid = math.sqrt(np.linalg.det((jac * jac.transpose())))
        # m2 = 1/(m1+epsilon)
        # m3 = - math.log(m1)

        manip_score = self.m_score(joint_angles)
        # m4 = 1-mm
        # m5 = -math.log(mm+epsilon)
        # m6 = 1/(mm+epsilon)
        # m9 = 1/((mm+0.0001)**0.5)

        return manip_ellipsoid, manip_score

    def send_traj(self):
        print("\033[33msend_traj")
        # Create the topic message
        traj = JointTrajectory()
        traj.header = Header()
        # Joint names for UR5
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                            'wrist_3_joint']

        rate = rospy.Rate(1)
        cnt = 0
        pts = JointTrajectoryPoint()
        traj.header.stamp = rospy.Time.now()

        while not rospy.is_shutdown():
            print("\033[33mcnt:", cnt)
            cnt += 1

            if cnt%2 == 1:
                pts.positions = waypoints[0]
            else:
                pts.positions = waypoints[1]

            pts.time_from_start = rospy.Duration(1.0)

            # Set the points to the trajectory
            traj.points = []
            traj.points.append(pts)
            # Publish the message
            self.joint_traj_pub.publish(traj)
            rate.sleep()

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the self.move_group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        # print("\033[33mjoint_goal:", joint_goal, type(joint_goal))
        # manip1, manip2 = self.calculate_mscore_from_joints(joint_goal)
        # print("\033[33mmanip1, manip2:", manip1, manip2)
        # self.manip_ellipsoid_list.append(manip1)
        # self.manip_score_list.append(manip2)

        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        # joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the self.move_group
        move_group.go(joint_goal, wait=True)
        print("\033[33mgogogo======================")

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()
        print("\033[33mstopstop======================")
        # self.plot_mscore('m1', self.manip_ellipsoid_list)
        # self.plot_mscore('m2', self.manip_score_list)

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

if __name__ == "__main__":
    try:
        RotateSingleJoint()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")



                # print("\033[33mee_link:", self.ee_link)
        # tf_listener = tf.TransformListener()
        # # print(tf_listener.allFramesAsDot())
        # # print(tf_listener.allFramesAsString())
        # # print(tf_listener.getFrameStrings())
        # # print(tf_listener.frameExists("world"))
        # print(tf_listener.frameExists("upper_arm_link"))
        # print(tf_listener.frameExists("wrist_3_link"))
        # # t = tf_listener.getLatestCommonTime("/upper_arm_link", "/wrist_3_link")
        # # position, quaternion = tf_listener.lookupTransform("/upper_arm_link", "/wrist_3_link", rospy.Time(0))
        # # print("\033[33mbask_link-tool0 (position, quaternion):", position, quaternion)
        # # if tf_listener.frameExists("/upper_arm_link") and tf_listener.frameExists("/wrist_3_link"):
        # #     t = self.tf.getLatestCommonTime("/upper_arm_link", "/wrist_3_link")
        # #     position, quaternion = tf_listener.lookupTransform("/upper_arm_link", "/wrist_3_link", t)#rospy.Time(0))
        # #     print("\033[33mbask_link-tool0 (position, quaternion):", position, quaternion)
        # # else:
        # #     print("\033[33mNOnononono")
        # (base_trans_tool, base_rot_tool) = tf_listener.lookupTransform("world", "world", rospy.Time(0))
        # print("\033[33mbask_link-tool0 (position, quaternion):", base_trans_tool, base_rot_tool)

        # # try:
		# # 	(base_trans_tool, base_rot_tool) = tf_listener.lookupTransform(self.base_link, self.tip_link, rospy.Time(0))
        # #     print("\033[33mbask_link-tool0 (position, quaternion):", base_trans_tool, base_rot_tool)
		# # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# # 	rospy.logwarn('lookupTransform for robot failed!, ' + self.base_link + ', ' + self.tip_link)


        # base2tool_pose = tf_listener.lookupTransform("/world", "/tool0", rospy.Time(0))
        # print("\033[33mbask_link-tool0 pose:", base2tool_pose)
   
        # # tf_listener = tf.TransformListener()
        # # print(tf_listener.allFramesAsString())
        # # (trans, rot) = tf_listener.lookupTransform("fr3_link0", "fr3_link1", rospy.Time())
        # # rospy.loginfo('Translation of obj', trans)