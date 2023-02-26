## Rosbridge

Rosbridge allows for communication between ROS2 and non-ROS programs by creating a WebSocket connection that can convert between ROS messages and JSON objects. We use this to remotely publish and subscribe to topics for our teleoperation.

Launch the ROS2 WebSocket with `ros2 launch urc_teleop websocket.launch.py`
