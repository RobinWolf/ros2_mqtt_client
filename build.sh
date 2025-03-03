#!/bin/bash
##############################################################################
##                   Build the image, using Dockerfile                  ##
##############################################################################
ROS_DISTRO=humble

uid=$(eval "id -u")
gid=$(eval "id -g")

#--no-cache \
docker build \
  --build-arg ROS_DISTRO="$ROS_DISTRO" \
  --build-arg UID="$uid" \
  --build-arg GID="$gid" \
  -f Dockerfile \
  -t visionai_ros/mqtt_client/"$ROS_DISTRO" .