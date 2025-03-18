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
from auto_nav.mapNode import MapNode
import matplotlib.pyplot as plt
import torch
import torch.nn.functional as F

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
        self.nextcoords = MapNode(0,0)
        self.visited = set()
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = torch.empty((0,0))
        self.map_res = 0
        self.map_origin=0
        
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
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)

        #interactive image
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.image = self.ax.imshow(np.zeros((1,1)), cmap='gray')
        plt.colorbar(self.image, ax=self.ax)
        plt.title("Occupancy Grid with Path")


    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        
        # Extract x and y coordinates
        # self.coords = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # self.get_logger().info('In odom_callback')
        # self.get_logger().info('x: %f y: %f' % (self.coords[0], self.coords[1]))

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

        # get map resolution
        self.map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        self.map_origin = msg.info.origin.position

        #reshape into 2D
        oc2 = msgdata 
        # reshape to 2D array using column order
        
        self.occdata = torch.tensor(oc2).reshape(msg.info.height, msg.info.width)

        # print to file
        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
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
    

    def plan_route(self):
        # Get the occupancy grid
        occ_grid = self.occdata
        #return if empty
        if occ_grid.shape[0] == 0:
            return []
        # Apply max pooling to the occupancy grid
        occ_grid = occ_grid.reshape(1,1,occ_grid.shape[0], occ_grid.shape[1])
        occ_grid_pooled = F.max_pool2d(occ_grid, 5,5).reshape(occ_grid.shape[2]//5, occ_grid.shape[3]//5)
        # find where the turtlebot lies in the pooled occupancy_grid
        pooled_x = -1
        pooled_y = -1
        try:
            timeout = rclpy.time.Duration(seconds=2.0)  # Wait up to 2 seconds
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout):
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                map_x= (transform.transform.translation.x - self.map_origin.x)/self.map_res
                map_y= (transform.transform.translation.y - self.map_origin.y)/self.map_res
                pooled_x = int(map_x / 5)
                pooled_y = int(map_y / 5)
                self.coords = (pooled_x, pooled_y)
                self.get_logger().info(f'Map coordinates: x={map_x}, y={map_y}')
            else:
                self.get_logger().warn('Transform from "map" to "base_link" not available yet. Retrying...')
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Transform lookup failed: {e}')

        # Create a start node
        if pooled_x == -1 or pooled_y == -1:
            return []
        start = MapNode(pooled_x, pooled_y)
        frontier = [start]
        visited = set()
        current_node = start
        while len(frontier) > 0:
            current_node = frontier.pop(0)
            if current_node in visited:
                continue
            if occ_grid_pooled[current_node.x, current_node.y] > 90:
                continue
            visited.add(current_node)
            #self.get_logger().info(f'Current node: x={current_node.x}, y={current_node.y}, map_value={occ_grid_pooled[current_node.x, current_node.y]}')
            if occ_grid_pooled[current_node.x, current_node.y] == -1:
                break
            neighbors = current_node.generate_neighbors(occ_grid_pooled.shape[0], occ_grid_pooled.shape[1])
            for neighbor in neighbors:
                if neighbor not in visited:
                    frontier.append(neighbor)
                    neighbor.parent = current_node
        path = []
        visualize_path = []
        while current_node is not None:
            
            path.append(current_node)
            visualize_path.append(MapNode(current_node.x, current_node.y))
            current_node = current_node.parent
        path.reverse()
        self.visualize_path(visualize_path, occ_grid_pooled)
        return path
    
    def visualize_path(self, path, pooled_occ_grid):
        # Create a copy of the occupancy grid
        occ_grid_copy = np.copy(pooled_occ_grid)
        # Mark the path on the occupancy grid
        for node in path:
            occ_grid_copy[node.x, node.y] = 500
        #save the copy
        np.savetxt('pooled_map.txt', occ_grid_copy)
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
            self.path = self.plan_route()
            if len(self.path) != 0:
                self.nextcoords = self.path.pop()
            self.get_logger().info('going into loop')
            while rclpy.ok():
                
                if len(self.path) == 0:
                    self.path = self.plan_route()
                    if len(self.path) != 0:
                        self.nextcoords = self.path.pop()
                    
                else:
                    self.get_logger().info('Current position: x=%f, y=%f' % (self.coords[0], self.coords[1]))
                    #calculate distance from current to next
                    distance = math.sqrt((self.nextcoords.x - self.coords[0])**2 + (self.nextcoords.y - self.coords[1])**2)
                    self.get_logger().info('Distance to next node: %f' % distance)
                    if distance < 0.4:
                        self.get_logger().info('going to next node')
                        if len(self.path) > 0:
                            self.nextcoords = self.path.pop()
                            self.get_logger().info('Next node is: x=%f, y=%f'% (self.nextcoords.x, self.nextcoords.y))
                        else:
                            self.get_logger().info('Destination reached')
                    
                # allow the callback functions to run
                rclpy.spin_once(self)

        # except Exception as e:
        #    print(e)
        
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
