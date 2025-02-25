# AUSPEX-KNOW

A knowledge base for all AUSPEX modules.

# Build

Build AUSPEX-KNOW using colcon:
```
colcon build
```

# Run AUSPEX-KNOW

To run ***AUSPEX-KNOW***, first run valkey server via predefined alias:
```
start_valkey
```
Then run knowledge main via:
```
ros2 run auspex_knowledge knowledge_main
```
This runs all necessary ROS2 nodes for a working knowledge base.

# Connect to GUI

For connecting ***AUSPEX-KNOW*** to [AUGUR](https://github.com/UniBwM-IFS-AILab/AUGUR) the [Rosbridge Suite](https://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge) is necessary.<br>

For installation run:
```
sudo apt-get install ros-humble-rosbridge-suite
```
To start the rosbridge server run:
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
For seeing example UAVs in the GUI, you can run a mock publisher:
```
ros2 run auspex_knowledge drone_state_mock_publisher
```
