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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np


class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        # replace 0's with nan
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        lr2i = np.nanargmin(laser_range)

        # log the info
        self.get_logger().info('Shortest distance at %i degrees' % lr2i)
	
        # angle_min_deg = msg.angle_min * 180.0 / 3.14159 
        # angle_max_deg = msg.angle_max * 180.0 / 3.14159
        # scan_range = angle_max_deg - angle_min_deg
        
        # self.get_logger().info(f"Scan Range: {scan_range}")
        # self.get_logger().info(f"angle_min: {angle_min_deg}, angle_max: {angle_max_deg}")
        # self.get_logger().info(f"Number of readings: {len(msg.ranges)}")
        
        angle_increment_rad = msg.angle_increment
        angle_increment_deg = angle_increment_rad * 180.0 / 3.14159  # Convert to degrees

        self.get_logger().info(f"Angle Increment: {angle_increment_rad} rad ({angle_increment_deg}°)")
        self.get_logger().info(f"Total Scan Range: {(msg.angle_max - msg.angle_min) * 180.0 / 3.14159}°")
        self.get_logger().info(f"Number of Readings: {len(msg.ranges)}")

def main(args=None):
    rclpy.init(args=args)

    scanner = Scanner()

    rclpy.spin(scanner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
