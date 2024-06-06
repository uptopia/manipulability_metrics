https://github.com/tku-iarc/UR5_control
# UR手臂控制
```bash
UR手臂 package來源參考https://github.com/fmauch/universal_robot.git ＆ https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
安裝教學參考https://blog.csdn.net/zxxxiazai/article/details/103568577
```
## 1.install moveIt ＆ trac_ik
  ```bash
  $ sudo apt-get install ros-melodic-moveit
  $ sudo apt-get install ros-melodic-trac-ik-kinematics-plugin
  ```
  
## 2. gazebo test
  ### 請參考補充
  ```bash
 1. $ roslaunch ur_gazebo ur5.launch
 2. $ roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
  
 3. 控制二則一
    rviz
   $ roslaunch ur5_moveit_config moveit_rviz.launch config:=true
    python
  $ rosrun ur_move_test ur_move_test_node.py
  ```
## 3. real robot control
  ### 注意執行順序不能改變
  ```bash
  1.設定電腦ip 為192.168.0.100 （可以更改,如果更改電腦ip,手臂external_control.urp程序的ip也要做調整）
  2.開起手臂電源
  4.執行 roslaunch ur_robot_driver ur5_bringup.launch limited:=true r[![GitHub Workflow Status](https://github.com/tecnalia-medical-robotics/manipulability_metrics/workflows/CI/badge.svg?branch=melodic-devel)](https://github.com/tecnalia-medical-robotics/manipulability_metrics/actions)
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# Manipulability Metrics

<p align="center">
  <img width="640" src="https://user-images.githubusercontent.com/678580/96242832-3731dd00-0fa4-11eb-990d-495cb4b44640.png">
</p>


This repository provides a library to easily compute multiple manipulability metrics for a single or dual-arm manipulator.

The list of metrics that can be computed for a single-arm manipulator are:

- **Inverse condition number:**
Computes the quotient between the smaller and the greater singular values of the Jacobian.
The inverse of the condition number is used so that a bigger value represents better manipulability, which is the case with the rest of the metrics.
- **Manipulability Measure:**
Computes the square root of the determinant of the product of the Jacobian and the Jacobian transpose. It's equivalent to the product of all singular values of the Jacobian.
- **Minimum Singular Value:**
Returns the value of the minimum singular value.
- **Task-Oriented Manipulability Measure (TOMM):**
Computes how well the manipulability ellipsoid of the manipulator matches the provided _desired_ manipulability ellipsoid.

In addition to the above metrics that can be used for an individual arm in a dual-arm setup, the following dual-arm-specific metrics are available:

- **Dual-Arm Manipulability Measure (DAMM)<sup>[1](#damm-note)</sup>:**
Approximates the combined manipulability ellipsoid of both manipulators as the intersection of the manipulability matrices of each manipulator and returns a value proportional to its volume.
- **Task-Oriented Dual-Arm Manipulability Measure (TODAMM):**
Computes how well the combined manipulability ellipsoid of both manipulator matches the provided _desired_ manipulability ellipsoid.

For more information about the TOMM, DAMM and TODAMM metrics, refer to: S. Lee, "Dual redundant arm configuration optimization with task-oriented dual arm manipulability," in IEEE Transactions on Robotics and Automation, vol. 5, no. 1, pp. 78-97, Feb. 1989, doi: 10.1109/70.88020. ([link](https://ieeexplore.ieee.org/document/88020)).

## Installation/setup

This library is provided by the `manipulability_metrics` ROS package.
The accompanying `manipulability_metrics_examples` package contains two usage examples, for the single and dual-arm use-cases.

Both packages require C++14 to be built, and therefore only support ROS Melodic or newer distros.

This packages can be built and used following the usual ROS workflow:
- Add the sources of this repository to a ROS workspace
- Use `rosdep` to install any missing dependency
- Build with your prefered tool: `catkin_tools` (tested), `catkin_make` or `catkin_make_isolated`.

Any package in the same or overlying workspaces can then use the library in the usual way.

In order to run the examples, issue any of the two commands below after having built the packages and sourced the workspace's setup script:
- `roslaunch manipulability_metrics_examples panda_manipulability_demo.launch`
- `roslaunch manipulability_metrics_examples dual_panda_manipulability_demo.launch`

## Usage

The following snippet illustrates the simplest possible use case.

```cpp
#include <manipulability_metrics/manipulability_metrics.h>
#include <iostream>

int main(int argc, char** argv)
{
  const auto model = getModel();  // Parse or build a urdf::ModelInterface instance

  const auto chain = manipulability_metrics::Chain{ model, "chain_root", "chain_tip" };

  const auto jntarray = getJointPositions();  // Fill KDL::JntArray with joint positions

  std::cout << "Inverse Condition Number: " << manipulability_metrics::inverseConditionNumber(chain, jntarray) << '\n';

  return 0;
}
```

More detailed use cases can be found in the examples package.

## Acknowledgements

This development is supported by the European Union's Horizon 2020 project RobotUnion.
This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 779967.

The opinions and arguments expressed reflect only the author's view and reflect in no way the European Commission's opinions. The European Commission is not responsible for any use that may be made of the information it contains.

[![RobotUnion logo](https://user-images.githubusercontent.com/678580/96243105-92fc6600-0fa4-11eb-87ee-0e41ae040c55.png)](https://robotunion.eu/)

---

<a name="damm-note">[1]</a>:
the paper proposes methods to compute the DAMM for two manipulators in two configurations: _tight cooperation_, where no freedom of motion exists between both arms, e.g. when carrying a bulky load bi-manually; and _loose cooperation_, where partial relative motion between both arms is permitted, e.g. when assembling parts held by each arm.
However, only the _tight cooperation_ scenario is supported by this library at the moment.
obot_ip:=192.168.0.12(robot_ip)
  
  5.開起手臂程序
    示教器，運行程序 —> 文件 —> 加载程序 —> external_control.urp程序，打開—>運行
    可以看到终端显示：
	[ INFO]: Robot mode is now POWER_ON
	[ INFO]: Robot mode is now IDLE
	[ INFO]: Robot mode is now RUNNING
	[ INFO]: Robot requested program
	[ INFO]: Sent program to robot
	[ INFO]: Robot ready to receive control commands.
	
  6.roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
  7.roslaunch ur5_moveit_config moveit_rviz.launch config:=true 
  如果想用python控制手臂控制步驟7可以更換成python檔
    rosrun ur_move_test ur_move_test_node.py
  ```  


## 補充：
   ```bash
   1.真實手臂與gazebo間的切換
     控制真實手臂與gazebo必須要更改 /universal_robot/ur5_moveit_config/config/controllers.yaml 文件
     真實手臂必須在檔案內的  name: 後添加 scaled_pos_joint_traj_controller
     gazebo必須在檔案內的  name: 後添加 ""
     
     否則會報錯 [ERROR] : Action client not connected: /follow_joint_trajectory

   2.如何修改運動學演算法：
     原本官方是KDL演算法,此package為trak_ik演算法
     想換演算法則必須更改 /universal_robot/ur5_moveit_config/config/kinematics.yaml 文件
     kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin 取代成
     kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin

   3.限制關節運動角度：
     修改 /universal_robot/ur_description/urdf/ur5_joint_limited_robot.urdf.xacro文件
     shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}"
     shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}"
     elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}"
     wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}"
     wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}"
     wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
     預設是-pi to pi 可以去設定想要的角度
     如果想要讓gazebo初始狀態顯示為有限制過的角度 更改 ur5.launch文件 將limited的default設為true
   ```



