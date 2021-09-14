# ros2_bag
## Description of the package
Ros2 package used to simplify rosbag recording and playback from a turtlebot3 running Nav2 and slam-toolbox. It has a qos override to allow rviz2 to playback the laser scan data.

## Usage
**Basic usage:** *ros2 launch ros2_bag record_bag.launch.py*

It will record a bag called test, in the folder /root/rosbags that is created if not present.

**Complete usage with default arguments:** *ros2 launch ros2_bag record_bag.launch.py folder:=/root/rosbags file:=test overwrite:=false*

It works as above.

If overwrite is not set to true, it will create a new file called <file_name> followed by a number from 1 to 100, if the others are already present
