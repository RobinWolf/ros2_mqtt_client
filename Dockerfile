##############################################################################
##                           1. stage: Base Image                           ##
##############################################################################
ARG ROS_DISTRO=humble

# For PC with amd64: (https://hub.docker.com/r/osrf/ros/tags?page=1&page_size=&name=&ordering=?
#FROM osrf/ros:$ROS_DISTRO-desktop AS base

#For Jetson with arm64: (https://hub.docker.com/r/arm64v8/ros/tags)
FROM arm64v8/ros:$ROS_DISTRO AS base

# Configure DDS for node communication
COPY dds_profile.xml /opt/misc/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/misc/dds_profile.xml

# Create user with root privilege
ARG USER=mqtt_client
ARG UID=1000
ARG GID=1000
ENV USER=$USER
RUN groupadd -g $GID $USER \
    && useradd -m -u $UID -g $GID --shell $(which bash) $USER 

# Setup workpace
USER $USER
RUN mkdir -p /home/$USER/ros2_ws
WORKDIR /home/$USER/ros2_ws


##############################################################################
##                     2. stage: install needed MQTT Packages               ##
##############################################################################
FROM base AS mqtt

# install pip
USER root
RUN apt-get update && apt-get install -y pip
USER $USER

# install mqtt packages and libaries
USER root
RUN apt-get update && apt-get install -y mosquitto-clients
RUN pip install paho-mqtt
USER $USER

# Append the ROS 2 source command to .bashrc and build
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
 
# Copy src into src folder to build the workspace initially --> mounting overwrites this
COPY ./src /home/$USER/ros2_ws/src

# Build the workspace packages
RUN cd /home/$USER/ros2_ws && \
     . /opt/ros/$ROS_DISTRO/setup.sh && \
     colcon build

# Add built package to entrypoint by calling install/setup.bash
USER root
RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/ros2_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER

# start the Node inside the Container
CMD ["bash"]
#CMD ["ros2", "launch", "ros2mqtt_bridge", "mqtt_client.launch.py"]