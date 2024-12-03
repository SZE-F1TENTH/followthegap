# Follow The Gap
Basic Follow The Gap algorithm implementation for F1TENTH, tested with F1TENTH racecar. 

# Installation
```bash
cd ros2_ws/src
git clone https://github.com/SZE-F1TENTH/followthegap.git
cd ..
colcon build
```
# Parameters
safety radius - minimum safe distance from obstacles
max throttle - no longitudinal controller is implemented, this currently serves as constant throttle value
steering sensitivity - proportional value for steering input, it can be used with sign to change steering direction
max steering angle - defined by hardware
wheelbase - defined by hardware, only used for steering marker radius calculation
