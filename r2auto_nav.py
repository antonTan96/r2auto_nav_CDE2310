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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
import cmath
import time
from mapNode import MapNode
import matplotlib.pyplot as plt

# constants
rotatechange = 0.1
speedchange = 0.2
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 10
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.coords = (0,0)
        self.path = [] 
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        # Create a tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #interactive image
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.image = self.ax.imshow(self.occdata, cmap='gray', vmin=0, vmax=1)
        plt.colorbar(self.image, ax=self.ax)
        plt.title("Occupancy Grid with Path")


    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        
        # Extract x and y coordinates
        self.coords = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Transform odometry coordinates to map frame
        # try:
        #     transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        #     map_x, map_y = self.transform_coordinates(self.x, self.y, transform)
        #     self.get_logger().info(f'Map coordinates: x={map_x}, y={map_y}')
        # except tf2_ros.LookupException as e:
        #     self.get_logger().error(f'Transform lookup failed: {e}')

    def transform_coordinates(self, x, y, transform):
        # Apply the transformation to the coordinates
        transformed_x = transform.transform.translation.x + x * math.cos(transform.transform.rotation.z) - y * math.sin(transform.transform.rotation.z)
        transformed_y = transform.transform.translation.y + x * math.sin(transform.transform.rotation.z) + y * math.cos(transform.transform.rotation.z)
        return transformed_x, transformed_y

    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
    
    def max_pooling_2d(arr, pool_size=(5, 5), stride=(5, 5)):
    # Create a view of the array with sliding windows
        windows = np.lib.stride_tricks.sliding_window_view(arr, pool_size)
        
        # Apply max pooling over the windows
        pooled = windows.max(axis=(2, 3))
        
        # Stride the result if necessary
        if stride != pool_size:
            pooled = pooled[::stride[0], ::stride[1]]
    
        return pooled

    def plan_route(self):
        # Get the occupancy grid
        occ_grid = self.occdata
        # Apply max pooling to the occupancy grid
        occ_grid_pooled = self.max_pooling_2d(occ_grid, pool_size=(5, 5), stride=(5, 5))
        # find where the turtlebot lies in the pooled occupancy_grid
        pooled_x, pooled_y 
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            map_x, map_y = self.transform_coordinates(self.x, self.y, transform)
            pooled_x, pooled_y = int(map_x / 5), int(map_y / 5)
            self.get_logger().info(f'Pooled coordinates: x={pooled_x}, y={pooled_y}')
            self.get_logger().info(f'Map coordinates: x={map_x}, y={map_y}')
        except tf2_ros.LookupException as e:
            self.get_logger().error(f'Transform lookup failed: {e}')

        # Create a start node
        start = MapNode(pooled_x, pooled_y)
        frontier = [start]
        visited = set()
        current_node = start
        while len(frontier) > 0:
            current_node = frontier.pop(0)
            if current_node in visited:
                continue
            visited.add(current_node)
            if occ_grid_pooled[current_node.x, current_node.y] == -1:
                break
            neighbors = current_node.generate_neighbors(occ_grid_pooled.shape[0], occ_grid_pooled.shape[1])
            for neighbor in neighbors:
                if neighbor not in visited:
                    frontier.append(neighbor)
                    neighbor.parent = current_node
        path = []
        while current_node is not None:
            path.append(current_node)
            current_node = current_node.parent
        path.reverse()
        #self.visualize_path(path, pooled_occ_grid)
        return path
    
    def visualize_path(self, path, pooled_occ_grid):
        # Create a copy of the occupancy grid
        occ_grid_copy = np.copy(pooled_occ_grid)
        # Mark the path on the occupancy grid
        for node in path:
            occ_grid_copy[node.x, node.y] = 2

        self.image.set_data(occ_grid_copy)
        # Redraw the plot
        plt.draw()
        plt.pause(0.01)  # Allow the GUI event loop to update




    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving

            #create route
            


            while rclpy.ok():
                
                self.plan_route()

                    
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
