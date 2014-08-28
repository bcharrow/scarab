#!/bin/bash

set -o nounset
set -o errexit

S1=scarab42
S2=scarab44

S1_POS_X=$(rosparam get /$S1/amcl/initial_pose_x)
S1_POS_Y=$(rosparam get /$S1/amcl/initial_pose_y)
S1_XY="{x: ${S1_POS_X}, y: ${S1_POS_Y}}"

S2_POS_X=$(rosparam get /$S2/amcl/initial_pose_x)
S2_POS_Y=$(rosparam get /$S2/amcl/initial_pose_y)
S2_XY="{x: ${S2_POS_X}, y: ${S2_POS_Y}}"

rostopic pub -1 /$S1/goal geometry_msgs/PoseStamped "{pose: {position: $S2_XY}}" &
rostopic pub -1 /$S2/goal geometry_msgs/PoseStamped "{pose: {position: $S1_XY}}" &

wait
