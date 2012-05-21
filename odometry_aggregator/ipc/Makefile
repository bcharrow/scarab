PKG= odometry_aggregator
MEX_MSGS= OdometryArray
ROS_MSGS= OdometryArray

MEX_FILES := $(foreach msg, $(MEX_MSGS), $(PKG)_$(msg))
ROS_FILES := $(foreach msg, $(ROS_MSGS), $(PKG)_$(msg)_publisher)
ROS_FILES += $(foreach msg, $(ROS_MSGS), $(PKG)_$(msg)_subscriber)

include $(shell rospack find ipc_roslib)/ipc_roslib.mk
include $(shell rospack find ipc_geometry_msgs)/ipc_geometry_msgs.mk
include $(shell rospack find ipc_nav_msgs)/ipc_nav_msgs.mk
include $(shell rospack find ipc_bridge_local)/ipc_bridge_local.mk
