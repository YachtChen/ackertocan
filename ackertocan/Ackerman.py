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
import random
import numpy 
from rclpy.node import Node
from ackermann_msg.msg import AckermannDrive  


class AckerPublisher(Node):

    def __init__(self):
        super().__init__('Acker_publisher')
        self.publisher_ = self.create_publisher(AckermannDrive, 'cmd_vel', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = AckermannDrive()
        #Assumptions made on values
        msg.steering_angle = random.uniform(-0.698, 0.698) #Maximum +-0.698 radians
        msg.steering_angle_velocity = random.uniform(0.0,17.4533)#0 to 17.4533 radians/second
        msg.speed = random.uniform(0, 41.7) #0-41.7m/s
        msg.acceleration = random.uniform(0, 20)#0-20 m/s^2
        msg.jerk = random.uniform(0, 12.5) #0-12.5 m/s^2
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: sa:%f sav:%f speed:%f acce:%f jerk:%f' % (msg.steering_angle,msg.steering_angle_velocity,msg.speed,msg.acceleration,msg.jerk))

def main(args=None):
    rclpy.init(args=args)

    Acker_publisher = AckerPublisher()

    rclpy.spin(Acker_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Acker_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
