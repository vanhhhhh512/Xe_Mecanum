Chạy từng dòng lệnh sau

mkdir -p ~/r_ws/src

cd ~/r_ws/src

git clone https://github.com/vanhhhhh512/Xe_Mecanum.git Xe

colcon build --packages-select Xe

source install/setup.bash

ros2 launch Xe master_launch.py
