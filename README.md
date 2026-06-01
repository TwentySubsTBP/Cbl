# Cbl
Create the package as instructed in the setting up workspace
paste the tb3world folder and use that.
when launching gazebo use that package
use  ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=src/../resource/map.yaml to get the correct map in nav2
get the pose 
make the python nav_test executeable by chomd -x ...
run the script but be careful with selecting hte goal
In theory if both robots are in the correct spot navigation should be smooth and fast
