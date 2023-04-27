A ROS2 GUI for interfacing with grbl device.
A project built on [cn5X](https://github.com/fra589/cn5X).

## Dependencies
### pyserial
```
pip3 install pyserial
```

## Build

Clone the repository into your ROS2 workspace and build it:

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules git@github.com:jhu-bigss/grbl_ros2_gui.git
cd ..
colcon build --symlink-install
```

## Instructions

Source the workspace and run the launch file:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch grbl_ros2_gui grbl_gui.launch.py
```

## Troubleshooting

### unrecognized arguments: `--ros-args`

You need to add the `--ros-args` argument (following line) to the `cn5X.py` file in the submodule [cn5X](https://github.com/fra589/cn5X):

```py
parser.add_argument("--ros-args", action="store_true", help=self.tr("ROS arguments flag"))
```

https://github.com/fra589/cn5X/blob/master/cn5X.py#L77-L83

### Changing permissions on serial port
To change the permissions for `/dev/ttyACM0` by adding yourself to the dialout group:
```
sudo usermod -a -G dialout $USER
```
Logout and then log back in for the group changes to take effect.


### Labjack
See labjack/[README](labjack/README.md)