# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#This node is a listener, receive raw CAN message and can be used to compare encoding precision

import rclpy
from rclpy.node import Node
from moa_msgs.msg import CAN
from moa_msgs.msg import CANStamped


class CanListener(Node):

    def __init__(self):
        super().__init__('CAN_Listener')
        self.subscription = self.create_subscription(
            CANStamped,
            'pub_raw_can',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #Print the received and decoded message
        my_list = [0, 0, 0, 0, 0]
        my_list[0] = (msg.can.data[0]-127.5)/182.665
        my_list[1] = msg.can.data[1]/14.61
        my_list[2] = msg.can.data[2]/6.1151
        my_list[3] = msg.can.data[3]/12.75
        my_list[4] = msg.can.data[4]/20.4
        my_string = ', '.join(str(x) for x in my_list)
        self.get_logger().info(my_string)


def main(args=None):
    rclpy.init(args=args)

    CAN_listener = CanListener()

    rclpy.spin(CAN_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    CAN_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
