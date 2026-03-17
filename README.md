# Minibot Scan Fusion

This is a ROS2 jazzy package for joining the lidar data from two ydlidar T-mini plus lidars into one combined lidar output.

For this the scan data of one lidar is transformed into the coordinate frame of the second lidar. A section of the scan data can be cut out for each lidar to prevent a robot from scanning itself. The trimmed and transformed scans from both lidars are then published as a new topic with the coordinate frame of one of the lidars.

The node is not intended to join more than two lidars into one, though you could easily add that into the source code.
* * *
To use this package it is recommended to follow the steps in the ydlidar installation section of the EduArt documentation for the minibot project, as the ydlidar SKD and the ydlidar_ros2_driver as well as port remapping are needed for the T-mini plus lidars.

After building the package, run the node with the launch file:
`ros2 launch minibot_scan_fusion minibot_scan_fusion.launch.py`
* * *
For the purpose of transparency: AI was heavily used to create this package but it was tested and used with our minibot robot and has worked flawlessly at the Robocup German Open 2026.
