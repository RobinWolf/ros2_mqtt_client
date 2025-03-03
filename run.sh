#!/bin/bash
##############################################################################
##                   Build the image, using Dockerfile                  ##
##############################################################################
ROS_DISTRO=humble

uid=$(eval "id -u")
gid=$(eval "id -g")


##############################################################################
##                            Run the container                             ##
##############################################################################
SRC_CONTAINER=/home/visionai/ros2_ws/src
SRC_HOST="$(pwd)"/src


docker run \
  --name visionai_ros_mqtt_client \
  --rm \
  -it \
  --net=host \
  -e DISPLAY="$DISPLAY" \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  --env-file .env\
  visionai_ros/mqtt_client/"$ROS_DISTRO"