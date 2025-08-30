# ros2_jazzy_lidar_ball_follower

A ROS 2 Jazzy package that enables TortoiseBot to follow a ball-shaped object in Gazebo using only LIDAR data. The node processes `/scan` data to detect the closest object (e.g., a red sphere) in front of the robot, estimates its direction and distance, and publishes velocity commands to `/cmd_vel` to align and follow it. Includes teleoperation support for manual control.

---

This project extends TortoiseBot’s LIDAR capabilities by enabling it to **follow a ball-shaped object** (e.g., a red sphere) in a Gazebo world using only `/scan` data. The robot:  

- Detects the **closest object** in its field of view  
- Calculates **direction and distance** of the object  
- Publishes velocity commands to `/cmd_vel` to slowly follow the ball  
- Aligns itself by turning if the object is off-center  
- Moves forward when the ball is centered
  
  Click [here](https://youtu.be/KUTT6o2aveo)to watch the output video.
  
---

## System Requirements
- **OS**: Ubuntu 24.04  
- **ROS 2**: Jazzy Jalisco  
- **Simulator**: Gazebo Harmonic  

---

## Installation

```bash
# Create workspace if not already created
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/your_username/ros2_jazzy_lidar_ball_follower.git

# Install Python dependencies
pip install -r ros2_jazzy_lidar_ball_follower/requirements.txt

# Build the workspace
cd ~/ros2_ws
colcon build

# Source environments
source /opt/ros/jazzy/setup.bash
source install/setup.bash
````

---

## Simulation

### 1. Launch Simulation

Start TortoiseBot in Gazebo with LIDAR:

```bash
ros2 launch tortoisebot_gazebo tortoisebot_world.launch.py
```
![](https://github.com/Sivapriya083/ros2_jazzy_lidar_ball_follower/blob/main/ball_follower.png?raw=true)



Spawn a red ball in front of the robot (URDF/SDF sphere with color property).

---

### 2. Run Ball Follower Node

```bash
ros2 run ros2_jazzy_lidar_ball_follower ball_follower
```

The node will:

* Subscribe to `/scan` (`sensor_msgs/msg/LaserScan`)
* Detect the closest object (the ball)
* Estimate angle and distance
* Publish velocity commands to `/cmd_vel` to follow it

---

### 3. Teleop Control (Manual Override)

If you want to manually drive the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Control keys:

* `w` → forward
* `s` → backward
* `a` → turn left
* `d` → turn right
* `space` → stop

---

## Example Output

```bash
Ball detected at distance: 1.20 m, angle: -5.0°
Aligning left...
Ball centered, moving forward...
```

---

## Simulation Notes

* Ensure the LIDAR plugin is active in TortoiseBot URDF.
* Adjust FOV filtering (e.g., ±20°) in the code if needed.
* The ball color is for **visual effect only**; detection uses **LIDAR ranges**.

---

## License

MIT License

