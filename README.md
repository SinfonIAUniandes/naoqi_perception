# NAOqi Perception for ROS 2

This ROS 2 package provides a bridge to the perception functionalities of SoftBank Robotics' Pepper and NAO robots, specifically interfacing with the `ALTracker` module from the NAOqi OS. It allows a ROS 2 system to control the robot's tracking and pointing behaviors.

## Features

*   **Pointing**: Command the robot to point at a specific 3D coordinate in space using one of its effectors (e.g., an arm).
*   **Face Tracking**: Enable different modes for tracking faces:
    *   **Head Tracking**: The robot uses only its head to follow a detected face.
    *   **Full Body Tracking**: The robot can move its entire body to keep a detected face in view.
*   **Tracker Control**: Start and stop tracking modes on demand.

## Dependencies

*   `rclpy`
*   `naoqi_utilities_msgs`: Contains the custom service definitions (`PointAt`, `SetTrackerMode`) used by this node.

## How to Run the Node

To start the node, you need to provide the IP address and port of the robot.

```bash
ros2 run naoqi_perception naoqi_perception_node --ros-args -p ip:=<robot_ip> -p port:=<robot_port>
```

For example:
```bash
ros2 run naoqi_perception naoqi_perception_node --ros-args -p ip:=192.168.1.101 -p port:=9559
```

## ROS 2 API

All services are exposed under the node's namespace (`/naoqi_perception_node/` by default).

### Services

*   **`~/point_at`** ([naoqi_utilities_msgs/srv/PointAt](naoqi_utilities_msgs/srv/PointAt.srv))  
    Makes the robot point an effector (e.g., "LArm", "RArm") at a specified 3D point. You can define the point, frame, and speed.

*   **`~/set_tracker_mode`** ([naoqi_utilities_msgs/srv/SetTrackerMode](naoqi_utilities_msgs/srv/SetTrackerMode.srv))  
    Sets the current tracking mode. The available modes are:
    *   `"start_head"`: Starts face tracking using only the head.
    *   `"start_move"`: Starts face tracking using the whole body for movement.
    *   `"stop"`: Stops the tracker and returns the head to a neutral position.

## Usage Example

To make the robot start tracking a face with its head:

```bash
ros2 service call /naoqi_perception_node/set_tracker_mode naoqi_utilities_msgs/srv/SetTrackerMode "{mode: 'start_head'}"
```

To make the robot point its left arm at the coordinate (1.0, 0.5, 0.0) in its own frame:

```bash
ros2 service call /naoqi_perception_node/point_at naoqi_utilities_msgs/srv/PointAt "{effector_name: 'LArm', point: {x: 1.0, y: 0.5, z: 0.0}, frame: 2, speed: 0.5}"
```
*Note: `frame: 2` corresponds to `FRAME_ROBOT` in the NAOqi API.*

To stop all tracking:

```bash
ros2 service call /naoqi_perception_node/set_tracker_mode naoqi_utilities_msgs/srv/SetTrackerMode "{mode: 'stop'}"
```