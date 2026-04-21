mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src

git clone https://github.com/vanhhhhh512/Xe_Mecanum.git Xe

colcon build --packages-select Xe

source install/setup.bash

ros2 launch Xe master_launch.py
