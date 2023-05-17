# Brooks XPeel Module

A repository for Brooks XPeel, including user manuals and remote control interfaces.

This package guides a user to install python and ROS2 packages that remotely controls and receives feedback from the XPeel.

## Current Features
* Peeler initialization
* Peeler information (version number, current status, tape remaining, etc.)
* Basic movements (move conveyor, spool, reset microplate, etc.)
* Execute peeling 

## User Guide
1. ### Create a workspace
- `cd ~`
<!-- - `source /opt/ros/foxy/setup.bash` -->
- Create a WEI workspace it doesn't exist already
- `mkdir -p ~/wei_ws/src`

2. ### Git Clone brooks_xpeel_module repository
- `cd ~/wei_ws/src`
- `git clone https://github.com/AD-SDL/brooks_xpeel_module.git`

3. ### Install Brooks Peeler Driver
    #### Creating a conda/venv environment.
    - `conda create -n peeler-driver python=3.9`
    - `conda activate peeler-driver`

    #### Python Install 
    - `cd ~/wei_ws/src/brooks_xpeel_module/brooks_peeler_driver`
    - `pip install -r requirements.txt`
    - `pip install -e .`

    This installs brooks_peeler_driver as a package
4. ### Install Brooks Peeler Client
- `cd ~/wei_ws`
- `colcon build`
- `source install/setep.bash`


3. ### Sending commands to XPeel:
	* Connect XPeel Driver to device with a serial to usb cable.
	* Find XPeel port ex: "/dev/ttyUSB0" 

   	In Python:
    
		from azenta_driver.peeler_client import BROOKS_PEELER_CLIENT
		peeler = BROOKS_PEELER_CLIENT(port)

	* Send commands (provided below)

### ROS2 Launch

Launching the peeler client

 - `ros2 launch peeler_module_client peeler_module.launch.py`
 - Port name can be specified as `ros2 launch peeler_module_client peeler_module.launch.py peeler_port:=/dev/ttyUSB0`
