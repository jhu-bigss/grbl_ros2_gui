A ROS2 GUI for interfacing with grbl device



## Dependencies
### Xacro
```
sudo apt install ros-foxy-xacro
```
### pyserial
```
pip3 install pyserial
```

## Instructions
### Changing permissions on serial port
To change the permissions for `/dev/ttyACM0` by adding yourself to the dialout group:
```
sudo usermod -a -G dialout $USER
```
Logout and then log back in for the group changes to take effect.


### Labjack
See labjack/[README](labjack/README.md)