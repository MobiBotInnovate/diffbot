## Robot Package Template
### Install ros2 humble
- Follow these instructions https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
### Dependencies
```bash
sudo apt update
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controller
```

Setting up workspace
```bash
mkdir ~/ros2_ws && cd ~/ros2_ws
mkdir src && cd src
git clone https://github.com/MobiBotInnovate/diffbot.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
Running the progam
```bash
ros2 launch diffbot launch_sim.launch.py
```
Start the lidar
```
ros2 launch diffbot rplidar.launch.py
```
Start slam
```
ros2 launch diffbot online_async_launch.py
```
Start rviz
```
rviz2 -d ~/ros2_ws/src/config/view_bot.rviz
```
