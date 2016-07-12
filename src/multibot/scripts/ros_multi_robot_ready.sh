#Add repo to list
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
fi
#Add key and update package list
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update -q -y

#fix deps and install ros
sudo apt-get install -q -y libgl1-mesa-dev-lts-utopic
sudo apt-get install -q -y ros-jade-desktop-full

#init ros
sudo rosdep init
rosdep update
echo ". /opt/ros/jade/setup.bash" >> ~/.bashrc
. /opt/ros/jade/setup.bash

#install Git to get packages later
sudo apt-get install -q -y git

#install available prebuilt packages
sudo apt-get install -q -y ros-jade-navigation ros-jade-gmapping

#create empty workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/src

#get needed packages
git clone https://github.com/OSLL/aau_multi_robot.git
git clone https://github.com/LeoSko/teleop_twist_keyboard.git

cd ~/catkin_ws/
catkin_make

#source environment
echo ". ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
. ~/catkin_ws/devel/setup.bash

#configure adhoc_communication package to be run as root by default
sudo chown root ~/catkin_ws/devel/lib/adhoc_communication/adhoc_communication
sudo chmod +s ~/catkin_ws/devel/lib/adhoc_communication/adhoc_communication

echo "We are launching exploration for one robot as an example."
xterm -e roslaunch explorer explore_one.launch
