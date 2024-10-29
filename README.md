# Follow The Gap
Basic Follow The Gap algorithm implementation for F1TENTH, tested with the AutoDRIVE F1TENTH simulator. 

# Installation
```bash
cd ros2_ws/src
git clone https://github.com/SZE-F1TENTH/followthegap.git
cd ..
colcon build
```

## AutoDRIVE simulator

### Using WSL (Windows Subsystem for Linux)

Windows part
1. Download the simulator for Windows: https://github.com/Tinker-Twins/AutoDRIVE/releases/download/Simulator-0.3.0/AutoDRIVE_Simulator_Windows.zip
2. Unzip the downloaded folder.
3. Run the AutoDRIVE Simulator Application located inside the previously unzipped folder.
WSL Ubuntu 20.04 part
1. Install dependencies. Keeping the versions is important:
```bash
pip3 install eventlet==0.33.3
pip3 install Flask==1.1.1
pip3 install Flask-SocketIO==4.1.0
pip3 install python-socketio==4.2.0
pip3 install python-engineio==3.13.0
pip3 install greenlet==1.0.0
pip3 install gevent==21.1.2
pip3 install gevent-websocket==0.10.1
pip3 install Jinja2==3.0.3
pip3 install itsdangerous==2.0.1
pip3 install werkzeug==2.0.3
pip3 install attrdict
pip3 install numpy
pip3 install pillow
pip3 install opencv-contrib-python
```
2. Install the AutoDRIVE-Devkit package
```bash
cd ros2_ws/src
git clone https://github.com/rudolfkrecht/AutoDRIVE.git
cd AutoDRIVE
git checkout AutoDRIVE-Devkit
cd ../.. #go back to the ros2_ws folder
colcon build
cd ..
source /opt/ros/foxy/setup.bash && source ros2_ws/install/setup.bash #Don't forget to source
```

# Usage
Make sure to run AutoDRIVE simulator on Windows. After that, run the ROS 2 bridge and other tools. In a terminal, launch:
```bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch
```
In a different terminal, run:
```bash
ros2 run follow_the_gap follow_the_gap_node
```
### Note
To connect the Windows simulator with the ROS 2 bridge, open a WSL terminal, and type ```ifconfig```. Look for the ```eth0``` block. Type the ```inet``` IP address to the IP address box of the simulator. The port number is 4567. 
