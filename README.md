# Brooks XPeel Module

A repository for Brooks XPeel, including user manuals and remote control interfaces.

This package guides a user to remotely control and receive feedback from the XPeel.

Peeler is the main object responsible for removing seals off of microplates

## ROS2 Launch

### Sealer Client
 - `ros2 launch sealer_module_client sealer_module.launch.py`
 - Port name can be specified as `ros2 launch sealer_module_client sealer_module.launch.py sealer_port:=/dev/ttyUSB1`
### Peeler Client

 - `ros2 launch peeler_module_client peeler_module.launch.py`
 - Port name can be specified as `ros2 launch peeler_module_client peeler_module.launch.py peeler_port:=/dev/ttyUSB0`
