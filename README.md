# a_better_star
A ROS global planner plugin. Maybe it's A better A*.  
The plugin is a modification of A* global planner from official [global_planner](http://wiki.ros.org/global_planner) package. We use Euclidean distance as heuristics and make it faster to compute. And we alse update the open list of A* for attempting to get a more optimal path.  
## Setup plugin
Modify your launch file of move_base.Set the value of rosparam /move_base/base_global_planner to a_better_star/GlobalPlanner. Make sure the name space is correct. There is a brief yaml file as an example in the param folder.  
## Parameters
The parameters are compatible with the official global_planner package.  
