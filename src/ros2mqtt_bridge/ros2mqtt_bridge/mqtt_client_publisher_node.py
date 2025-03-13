'''
    author: @rw39401
'''

##ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


##Python
import json
import math
from datetime import datetime
import paho.mqtt.client as mqtt

class MQTTPublisherClient(Node):
    def __init__(self):
        super().__init__('mqtt_publisher_client_node')

        # subscribe to joint states
        self.ros_topic_subscription = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

        # load params
        self.declare_parameter('mqtt_broker', 'localhost')
        self._mqtt_broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.declare_parameter('mqtt_port', 1883)
        self._mqtt_port = self.get_parameter('mqtt_port').value
        self.declare_parameter('mqtt_qos', 0)
        self._mqtt_qos = self.get_parameter('mqtt_qos').value
        self.declare_parameter('mqtt_topic', 'default_topic')
        self._mqtt_topic = self.get_parameter('mqtt_topic').get_parameter_value().string_value

        # connect to mqtt broker
        self.is_connected = False
        self.get_logger().info(f'Connecting to MQTT Broker: {self._mqtt_broker}:{self._mqtt_port} ')
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect

        try:
            self.mqtt_client.connect(self._mqtt_broker, self._mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT Broker: {self._mqtt_broker}:{self._mqtt_port} - {e}')


    def on_connect(self, client, userdata, flags, rc):
        '''
        Callback function when Clinet connects to the Broker

        '''

        if rc == 0:
            self.get_logger().info(f'Connected to MQTT Broker successfully: {self._mqtt_broker}:{self._mqtt_port}')
            self.is_connected = True
        else:
            self.get_logger().error(f'Failed to connect to MQTT Broker: {self._mqtt_broker}:{self._mqtt_port}')
            self.is_connected = False


    def on_disconnect(self, client, userdata, rc):
        '''
        Callback function when Clinet disconnects from the Broker

        '''
                
        self.get_logger().warn(f'Disconnected from MQTT Broker: {self._mqtt_broker}:{self._mqtt_port}')
        self.is_connected = False


    def joint_states_callback(self, msg):
        '''
        Callback function for ROS topic subscription
        Publishes the received joint states message to the MQTT broker

        https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html

        '''

        #self.get_logger().info('Received Joint States Message over ROS2 Topic')
        
        timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] # -3 because milliseconds instead of microseconds
        translated_dict = {
            'Timestamp': timestamp,
            msg.name[0] : {'Value':msg.position[0]*(180/math.pi), 'Unit':'degree'},
            msg.name[1] : {'Value':msg.position[1]*(180/math.pi), 'Unit':'degree'},
            msg.name[2] : {'Value':msg.position[2]*(180/math.pi), 'Unit':'degree'},
            msg.name[3] : {'Value':msg.position[3]*(180/math.pi), 'Unit':'degree'},
            msg.name[4] : {'Value':msg.position[4]*(180/math.pi), 'Unit':'degree'},
            msg.name[5] : {'Value':msg.position[5]*(180/math.pi), 'Unit':'degree'},
        }

        # publish JSON String to MQTT broker
        if self.is_connected:
            self.mqtt_client.publish(self._mqtt_topic, json.dumps(translated_dict), qos=self._mqtt_qos)
            self.get_logger().info(f'Published Joint States Message to MQTT Broker {translated_dict}')
        #else:
            #self.get_logger().info(f'Dont publish Joint States Message to MQTT Broker, because no Connection')




def main(args=None):
    rclpy.init(args=None)
    my_mqtt_publisher_client = MQTTPublisherClient()
    rclpy.spin(my_mqtt_publisher_client)
    rclpy.shutdown()




if __name__ == '__main__':
    main()