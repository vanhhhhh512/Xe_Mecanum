Update fix lỗi không chạy được Gazebo ( cài thêm map của TurtleBot3 ) . Em chỉ sửa file Readme thêm một số lệnh để thầy dễ clone về lại không bị lỗi :

Bước 1: Dọn rác

killall -9 gzserver

killall -9 gzclient

killall -9 rviz2

pkill -9 -f mecanum_numpad.py

ros2 daemon stop

Bước 2: Tải map và thư viện

sudo apt update

sudo apt install -y ros-humble-turtlebot3-gazebo

sudo apt install -y ros-humble-joint-state-publisher

sudo apt install -y ros-humble-robot-state-publisher

sudo apt install -y ros-humble-xacro

sudo apt install -y ros-humble-gazebo-ros-pkgs

Bước 3: Clone code về và build lại

mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src

git clone https://github.com/vanhhhhh512/Xe_Mecanum.git Xe

colcon build --packages-select Xe

Bước 4: Chạy file launch hiển thị gazebo , rviz , terminal điều khiển xe

source install/setup.bash

ros2 launch Xe master_launch.py
