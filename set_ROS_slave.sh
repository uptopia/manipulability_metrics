# echo "export ROS_MASTER_URI=http://172.18.0.2:11311" >> ~/.bashrc
# echo "export ROS_IP=172.18.0.1" >> ~/.bashrc
# source ~/.bashrc

master_ip=$1
ros_ip=$2

echo "master_ip: ${master_ip}"
echo "ros_ip:    ${ros_ip}"

echo "export ROS_MASTER_URI=http://${master_ip}:11311" >> ~/.bashrc
echo "export ROS_IP=${ros_ip}" >> ~/.bashrc
source ~/.bashrc