# ros2_bag
## Description of the package
Ros2 package used to simplify rosbag recording and playback from a turtlebot3 running Nav2 and slam-toolbox. It has a qos override to allow rviz2 to playback the laser scan data.

## Record
**Basic usage:** *ros2 launch ros2_bag record_bag.launch.py*

It will record a bag called test, in the folder /root/rosbags that is created if not present.

**Complete usage with default arguments:** *ros2 launch ros2_bag record_bag.launch.py folder:=/root/rosbags file:=test overwrite:=false*

It works as above.

If overwrite is not set to true, it will create a new file called <file_name> followed by a number from 1 to 100, if the others are already present

## Play

**Basic usage:** *ros2 launch ros2_bag play_bag.launch.py*

It will play the latest file on Windows and the latest file on Linux only if none of them has been played since the last one was created, from the folder /root/rosbags. Otherwise on Linux it will play the last one that was played, since it had the latest metadata change.
This is caused by the inability to store and/or acces file creation time on unix with filesystem ext4, for now.

**Complete usage with default arguments:** *ros2 launch ros2_bag record_bag.launch.py folder:=/root/rosbags file:=test
