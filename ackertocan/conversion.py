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

import rclpy
from rclpy.node import Node

from ackermann_msg.msg import AckermannDrive  
from moa_msgs.msg import CAN

class conversion(Node):

    def __init__(self):
        super().__init__('converter')
        self.publisher_ = self.create_publisher(CAN, 'pub_raw_can', 10)
        self.subscription = self.create_subscription(
            AckermannDrive,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0
        

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.speed)
        
        #Initiate data
        msgB = CAN()
        msgB.id = self.i
        msgB.is_rtr = False
        msgB.data = [0,0,0,0,0,0,0,0]
        
        msgB.data[0] = round(msg.steering_angle*182.665+127.5) #Scale with 182.665 with offset of 127.5
        msgB.data[1] = round(msg.steering_angle_velocity*14.61) #scale with factor of 14.61
        msgB.data[2] = round(msg.speed*6.1151)#scale with factor of 6.1151
        msgB.data[3] = round(msg.acceleration*12.75) #scale with factor 12.75
        msgB.data[4] = round(msg.jerk*20.4) #scale with factor 20.4
        
        my_string = ', '.join(str(x) for x in msgB.data)
        self.publisher_.publish(msgB)
        self.get_logger().info(my_string)
        self.i += 1
        


def main(args=None):
    rclpy.init(args=args)

    converter = conversion()

    rclpy.spin(converter)
    rclpy.spin(converter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
