# vehicle-routing-ros2

Implementation of dynamic vehicle routing (DVR) algorithms in a truly distributed fashion based on ROS 2.

For a simplified version, please check out <https://github.com/lb-robotics/python-vehicle-routing>.

## Software Requirements

- Ubuntu 22.04 LTS (Jammy)
- ROS 2 Humble

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
source /opt/ros/humble/setup.bash
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

## References

- F. Bullo, E. Frazzoli, M. Pavone, K. Savla and S. L. Smith, "[Dynamic Vehicle Routing for Robotic Systems](https://ieeexplore.ieee.org/abstract/document/5954127?casa_token=sAaSTkWYbO8AAAAA:eE9HJHY242a0InCpEhtyF0-iPnP2DSIq73AVHbDkbQVy-yuM4i_RGsC-RiwneH00c-z6EfxoNdU)," in Proceedings of the IEEE, vol. 99, no. 9, pp. 1482-1504, Sept. 2011, doi: 10.1109/JPROC.2011.2158181.
- M. Pavone, A. Arsie, E. Frazzoli and F. Bullo, "Distributed Algorithms for Environment Partitioning in Mobile Robotic Networks," in IEEE Transactions on Automatic Control, vol. 56, no. 8, pp. 1834-1848, Aug. 2011, doi: 10.1109/TAC.2011.2112410.
- Vardi, Yehuda, and Cun-Hui Zhang. "The multivariate L1-median and associated data depth." Proceedings of the National Academy of Sciences 97.4 (2000): 1423-1426.

## Acknowledgements

- CI set up is borrowed from [autoware.universe](https://github.com/autowarefoundation/autoware.universe) and [autoware-github-actions](https://github.com/autowarefoundation/autoware-github-actions).
