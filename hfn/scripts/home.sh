#!/bin/bash

set -o nounset

S0=scarab40
S1=scarab41
S2=scarab42
S3=scarab43
S4=scarab44
S5=scarab45

DEST="{pose: {position: {x: 0.0, y: 0.0}}}"
rostopic pub -1 /$S0/goal geometry_msgs/PoseStamped "$DEST" &
rostopic pub -1 /$S1/goal geometry_msgs/PoseStamped "$DEST" &
rostopic pub -1 /$S2/goal geometry_msgs/PoseStamped "$DEST" &
rostopic pub -1 /$S3/goal geometry_msgs/PoseStamped "$DEST" &
rostopic pub -1 /$S4/goal geometry_msgs/PoseStamped "$DEST" &
rostopic pub -1 /$S5/goal geometry_msgs/PoseStamped "$DEST" &

wait
