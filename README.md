# vehicle-routing-ros2
Implementation of dynamic vehicle routing (DVR) algorithms in a truely distributed fashion based on ROS 2.

For a simplified version, please check out https://github.com/lb-robotics/python-vehicle-routing.

## Software Requirements
- Ubuntu 20.04 LTS (Focal)
- ROS 2 Galactic

## Usage
1. Create ROS workspace.
```bash
mkdir -p ~/dvr_ws/src
```

2. Clone this repo into `src` folder.
```bash
cd ~/dvr_ws/src
git clone https://github.com/shineyruan/vehicle-routing-ros2.git
```

3. Install dependencies.
```bash
cd ~/dvr_ws
source /opt/ros/galactic/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -i -y
```

4. Build the workspace. The entire workspace is composed in `dvr_launch` package.
```bash
cd ~/dvr_ws
colcon build --symlink-install --packages-up-to dvr_launch --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

5. Run the demo. `number_vehicles` controls the number of vehicles generated. (Default is 10)
```bash
ros2 launch dvr_launch dvr.launch.py number_vehicles:=10
```

6. Try more examples by modifying the parameters in `dvr_launch/config`!
