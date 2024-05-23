## Robot Package Template
### Install ros2 humble
- Follow these instructions https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
### Dependencies
Install on both the dev machine and raspberry pi
```bash
sudo apt update
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-twist-mux
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-joint-state-publisher
```

Install only on the dev machine 
```bash
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-rviz2
```


Add a standoff for the caster wheels 32mm 

encoder turns per revolution 2497.12

set value to 83 to turn 1 revolution per second

Setting up workspace for simulation
```bash
mkdir ~/ros2_ws && cd ~/ros2_ws
mkdir src && cd src
git clone https://github.com/MobiBotInnovate/diffbot.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
### Real robot
Next ssh into the raspberry pi 
```bash
mkdir ~/robot_ws && cd ~/robot_ws
mkdir src && cd src
git clone https://github.com/MobiBotInnovate/diffbot.git
cd ~/robot_ws
git clone -b humble git@github.com:Buzzology/diffdrive_arduino.git
cd diffdrive_arduino/
git checkout 3883c00
cd ..
git clone git@github.com:joshnewans/serial.git
git clone git@github.com:hiwad-aziz/ros2_mpu6050_driver.git
cd ~/robot_ws/ && colcon build --symlink-install 
source install/setup.bash
```
Running the simulated robot
```bash
ros2 launch diffbot launch_sim.launch.py
```
To run the real robot run the following command on the raspberry pi
```bash
ros2 launch diffbot robot_pi.launch.py
```
On the dev machine run the following command to start slam, nav2 and rviz2
```bash
ros2 launch diffbot robot_laptop.launch.py
```
To steer the robot you can either use the nav2 stack or with the keyboard using the following command
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=key_cmd_vel
```
