import rospy

from std_msgs.msg import String
from rospy_message_converter import json_message_converter
from talon_state_controller import TalonState.h

input_data = "".join([chr(i) for i in [97, 98, 99, 100]])






message = String(data = 'Hello')
json_str = json_message_converter.convert_ros_message_to_json(message)

