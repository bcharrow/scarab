#!/bin/bash

set -o nounset
set -o errexit

S1=scarab42
S2=scarab44
ORIENTATION="orientation: {w: 1.0}"
rostopic pub -1 /$S1/goal geometry_msgs/PoseStamped "{pose: {position: {x: 5.0}, $ORIENTATION}}" &
rostopic pub -1 /$S2/goal geometry_msgs/PoseStamped "{pose: {position: {y: 5.0}, $ORIENTATION}}" &

wait
