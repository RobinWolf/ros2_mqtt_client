# ROS2 Node to Bridge ROS2 Topics to MQTT Topics

This Node acts as a Client which Publishes Messages from ROS2 Topics as JSON-Strings on MQTT Topics

### Informations:

- The Node is launched by default, when the Container starts.
- This Container and the one which runs the Robot must have the same $ROS_DOMAIN_ID (currently = 10) specified in their .env-File and they have to run in the same Network (or on the same device)
    --> ROS2 Connection between the Containers over DDS-Backbone
- manual Launch File: ros2 launch ros2mqtt_bridge mqtt_client.launch.py

### Supported ROS2 Topics:
#### Joint States 
- ROS2 Topic: /joint_states (without effort) https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html

- MQTT Topic: robot/joint_states  
*{"name": ["shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "shoulder_pan_joint"], "position": [-1.5975495492817535, 1.5992276698352537, -1.5739582385953117, -1.5702114546204142, -2.489144220795557, 0.6527386745224636], "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}*

### Development Hints:
- Test-Broker (mqtt.eclipseprojects.io) not reachable on unauthentificated Port (1883) from FFT-HOT-SPOT WiFi (Firewall ???)
    --> works with iPhone HotSpot

- Verify Messages are send correctly to the Broker (inside the Container)
    1) open another Terminal and connect to the Container: ```docker exec -it visionai_ros_mqtt_client bash```
    2) ```mosquitto_sub -h mqtt.eclipseprojects.io -p 1883 -t "robot/joint_states"```


### On the Broker Side:
- define IP and Port in Mosquitto Conftg File *mosquitto.conf*
- enable unauthentificated Client Connection in *mosquitto.conf*: listener 1883, allow_anonymous true (outcoment !)
- start the broker with: mosquitto -v -c mosquitto.conf (you have to be in the mosquitto install folder or provide the whole path to the config !)
