# ros2_facelook_node

A Python ROS2 node that receives ```std_msg/Int32MultifactorArray``` that contain face bounding
box information and then outputs PWM commands to stepping motors that control the pan and
tilt of the camera. The goal is to keep the seen faces as close to the center if the image as possible.

This is designed to work in conjunction with [ros2_adafruit_pwmhat_node], [ros2_raspicam_node], and
[ros2_facefinder_node]. These work together to demonstrate ROS2 and distributed control.


### Building

The node is built using ROS2 on Raspbian. I have built the latest ROS2 sources on the Raspberry Pi 3 using a script at [ROS2OnPiTools] that fetches and builds the latest ROS2 on Raspbian. Once ROS2 is available, presuming we're accessing the console on the Pi, the build instructions are:

```
source /opt/ros2/setup.bash    # set ROS2 paths into environment variables
cd
git clone https://github.com/Misterblue/ros2_facelook_node.git
cd ros2_facelook_node
colcon build --symlink-install
```

Since the code is only Python (none of that C++ stuff), there are libraries.

### Running

The above build instructions build this package in its own directory so running requires setting up the environment for both ROS2 and this package:

```
source /opt/ros2/setup.bash        # set ROS2 paths into environment variables
cd $HOME/ros2_facelook_node
source install/local-setup.bash    # set local package links into environment variables
ros2 run ros2_facelook_node service
```

As of October 12, 2018, building Python only modules with ```colcon``` doesn't add the path to the install directory to ADMENT_PATH so the above ```ros2 run``` command will say "program not found". This should be fixed in a later release but, as of now, you will have to manually add the path to the search path.

### Notes

This programs doesn't use ROS2 parameters because, as of October 11, 2018, the Python API for parameters is not complete. Parameters will be added when available (hopefully in the Dec 2018 release).

[ROS2OnPiTools]: https://github.com/Misterblue/ROS2OnPiTools
[ros2_adafruit_pwmhat_node]: https://github.com/Misterblue/ros2_adafruit_pwmhat_node
[ros2_raspicam_node]: https://github.com/Misterblue/ros2_raspicam_node
[ros2_facefinder_node]: https://github.com/Misterblue/ros2_facefinder_node
