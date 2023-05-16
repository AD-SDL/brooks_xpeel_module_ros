# Brooks XPeel Module

A repository for Brooks XPeel, including user manuals and remote control interfaces.

This package guides a user to install python and ROS2 packages that remotely controls and receives feedback from the XPeel.


## ROS2 Launch

### Peeler Client

 - `ros2 launch peeler_module_client peeler_module.launch.py`
 - Port name can be specified as `ros2 launch peeler_module_client peeler_module.launch.py peeler_port:=/dev/ttyUSB0`
