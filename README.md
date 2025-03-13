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
*{'Timestamp': '2025-03-13T10:19:27.045', 'joint_a1': {'Value': -89.95271498890692, 'Unit': 'degree'}, 'joint_a2': {'Value': -89.95665240237182, 'Unit': 'degree'}, 'joint_a3': {'Value': 89.95489088007768, 'Unit': 'degree'}, 'joint_a4': {'Value': 0.0026753081328678204, 'Unit': 'degree'}, 'joint_a5': {'Value': 89.95476902175223, 'Unit': 'degree'}, 'joint_a6': {'Value': 89.95558374117988, 'Unit': 'degree'}}*

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
