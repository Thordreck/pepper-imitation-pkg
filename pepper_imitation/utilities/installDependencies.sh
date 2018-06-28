set -e

echo -e "Adding ROS repos..."

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

echo -e "Installing ROS dependencies..."
sudo apt update
sudo apt -y install ros-kinetic-catkin ros-kinetic-desktop-full ros-kinetic-pepper-bringup \
                    ros-kinetic-pepper-control ros-kinetic-pepper-bringup ros-kinetic-pepper-description \
                    ros-kinetic-pepper-gazebo-plugin ros-kinetic-pepper-robot ros-kinetic-pepper-sensors-py \
                    ros-kinetic-executive-smach ros-kinetic-smach ros-kinetic-smach-msgs \
                    ros-kinetic-smach-ros ros-kinetic-smach-viewer ros-kinetic-naoqi-driver

echo -e "Initializing rosdep..."
sudo rosdep init
rosdep update

