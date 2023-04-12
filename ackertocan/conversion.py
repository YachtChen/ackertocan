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

#This the main node that encode ackermann drive message to can message

import rclpy
from rclpy.node import Node
from ackermann_msg.msg import AckermannDrive
from ackermann_msg.msg import AckermannDriveStamped
from moa_msgs.msg import CAN
from moa_msgs.msg import CANStamped

class conversion(Node):

    def __init__(self):
        super().__init__('converter')
        self.publisher_ = self.create_publisher(CANStamped, 'pub_raw_can', 10)
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0
        

    def listener_callback(self, msg):
        #Show that it receives drive message
        self.get_logger().info('I heard: "%s"' % msg.drive.speed)
        
        #Initiate vairables
        msgB = CANStamped()
        msgB.header.frame_id = 'can_link'
        msgB.can.id = self.i
        msgB.can.is_rtr = False
        msgB.can.data = [0,0,0,0,0,0,0,0]
        
        #Use scaling and rounding method to encode
        msgB.can.data[0] = round(msg.drive.steering_angle*182.665+127.5) #Scale with 182.665 with offset of 127.5
        msgB.can.data[1] = round(msg.drive.steering_angle_velocity*14.61) #scale with factor of 14.61
        msgB.can.data[2] = round(msg.drive.speed*6.1151)#scale with factor of 6.1151
        msgB.can.data[3] = round(msg.drive.acceleration*12.75) #scale with factor 12.75
        msgB.can.data[4] = round(msg.drive.jerk*20.4) #scale with factor 20.4
        self.publisher_.publish(msgB)
        
        #Print the array in the log
        my_string = ', '.join(str(x) for x in msgB.can.data)
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
