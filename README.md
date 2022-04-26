# angle_planner
Global planner for ROS navigation stack with using orientation constraints

Installation

1. Copy files in your workspace_dir/src using
git clone git@github.com:grand-yarl/angle_planner.git

2. Add following to move_base.launch
```no-highlight
<param name="base_global_planner" value="angle_planner/AnglePlanner"/>
```
3. Add orientation publisher node by adding to navigation.launch
```no-highlight
<!-- Orientation publisher -->
<arg name="orient_file" default="$(find navigation_directory)/orientation/orientation_file.txt"/>
<node pkg="orientation_pub" name="orientation_pub" type="orientation_pub" args="$(arg orient_file)"/>
```
4. Create "orientation" directory in navigation and put orientation_file.txt in it

5. Data format described in orientation_file.txt. Look examples there!

Orientation restrictions in area are defined by 12 or 20
double numbers (12 numbers for 8 neighbours, 20 numbers for 16 neighbours)
in one line, that are separated by space 
(other separation types will cause an error!).
First 4 numbers define rectangle area bounds in global coordinate frame
Coordinates of first point should be less than coordinates of second
(min_x < max_x, min_y < max_y).
Following 8 or 16 numbers define restriction array.
All numbers in array should be belong interval [0, 1].
Subsequence of numbers in array is determined by the rule:
First number reflects the assessment of the transition 
when the robot moves along the x-axis, next angle evaluations
go by roundabout counterclockwise.
```no-highlight
              x                                           x
              ^                         3   2   1  16  15 ^
    2   1   8 |                         4              14 |
    3  i,j  7 |                         5      i,j     13 |
    4   5   6 |                         6              12 |
  y<----------|                         7   8   9  10  11 |
                                      y<------------------|
```
Data line:
```no-highlight
min_x min_y max_x max_y r[1] r[2] ... r[8] ... r[16].
```
Combining lines with 12 and 20 numbers is not allowed.
Restriction areas can overlap, but restriction array for the intersection
will be taken from the upper lines. 
Symbol # is used for comments.

6. Parameters in move_base.yaml:
```no-highlight
AnglePlanner:

  costmap_critical - Critical cost for costmap cell, default = 128
  
  orientation_critical - Critical cost for orientation restriction, default = 1
  
  orientation_coeff - Coefficient of orientation impact, default = 255
  
  use_16 - Use 16 neighbours instead of 8, default = False
```
7. Build your workspace

8. For adding orientation file from another direction add parameter
orient_file = "path to file"
in your launch command
