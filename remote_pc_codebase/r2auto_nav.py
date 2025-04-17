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
from std_msgs.msg import Int8

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformException
import numpy as np
import math
import cmath
import time
from auto_nav.mapNode import MapNode
import auto_nav.tests as robot_tests
import matplotlib.pyplot as plt
import torch
import torch.nn.functional as F
import threading
import random


# constants
heat_source_radius = 0.4
heat_sources = 2
rotatechange = 0.3
speedchange = 0.17
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.3
side_distance = 0.3
edge_distance = 0.3
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
        
        # self.get_logger().info('Created subscriber')
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.path = [] 
        self.next_coords = MapNode(0,0)
        self.bonk_count = 0
        self.spin_coords = []
        
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

        self.back_range = range(len(self.laser_range)//2 - 3, len(self.laser_range)//2 + 3)
        self.default_front_range = range(len(self.laser_range)//2 - 15, len(self.laser_range)//2 + 15)

        # Create a tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)

        #heat sources
        self.visited_heat_sources = []
        self.ir_index=-1
        self.heat_map_subscriber = self.create_subscription(
            Int8,
            'heatdir',
            self.heat_map_callback,
            10
        )
        self.heat_map_subscriber  # prevent unused variable warning

        #movement control to rpi
        self.launching = False
        self.rpi_publisher = self.create_publisher(Int8,'launch',10)
        #take back control from rpi
        self.rpi_subscriber = self.create_subscription(
            Int8,
            'move',
            self.rpi_callback,
            10)


        #interactive image
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.image = self.ax.imshow(np.zeros((1,1)), vmin=-1, vmax=100)
        plt.colorbar(self.image, ax=self.ax)
        plt.title("Occupancy Grid with Path")

        #spin node in separate thread
        self.spin_thread = threading.Thread(target=self.spin_node, daemon=True)
        self.spin_thread.start()

    def spin_node(self):
        rclpy.spin(self)

    def rpi_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info('Taking back control from RPI')
            self.launching = False
            

    def heat_map_callback(self, msg):
        self.ir_index = msg.data
        #self.get_logger().info(f'IR index: {self.ir_index}')

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

        # reshape to 2D array using column order        
        self.occdata = torch.tensor(msgdata).reshape(msg.info.height, msg.info.width)

        # print to file
        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        #np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def get_orientation(self):
        transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        _, _, current_angle = euler_from_quaternion(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        )
        return transform.transform.translation.x, transform.transform.translation.y, current_angle
        


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle, speed = 0.0):
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
        twist.linear.x = speed
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
    
    
    def plan_route(self, visualize=False, goal=None):
        # Get the occupancy grid
        occ_grid = self.occdata
        if visualize:
            np.savetxt(mapfile, occ_grid)
        found_path = False
        #return if empty
        if occ_grid.shape[0] == 0:
            return []
        #pad the occupancy grid to have size divisible by 3
        occ_grid = F.pad(occ_grid, (0, 3 - occ_grid.shape[1] % 3, 0, 3 - occ_grid.shape[0] % 3), value=-1) 
        # Apply max pooling to the occupancy grid
        occ_grid = occ_grid.reshape(1,1,occ_grid.shape[0], occ_grid.shape[1])
        occ_grid_pooled = F.max_pool2d(occ_grid, 3,3).reshape(occ_grid.shape[2]//3, occ_grid.shape[3]//3)
        
        # Create a distance-to-wall grid for path optimization
        wall_distance = np.zeros_like(occ_grid_pooled, dtype=float)
        WALL_THRESHOLD = 70  # Cells with values >= 70 are considered walls
        
        # Find cells that are walls
        wall_cells = np.where(occ_grid_pooled >= WALL_THRESHOLD)
        
        # Find where the turtlebot lies in the pooled occupancy_grid
        pooled_x = -1
        pooled_y = -1

        try:
            timeout = rclpy.time.Duration(seconds=2.0)  # Wait up to 2 seconds
            goal_pooled_x = -1
            goal_pooled_y = -1
            
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout):
                current_x, current_y, _ = self.get_orientation()
                map_x = (current_x - self.map_origin.x)/self.map_res
                map_y = (current_y - self.map_origin.y)/self.map_res
                if goal is not None:
                    goal_x, goal_y = goal.x, goal.y
                    goal_pooled_x = min(round((goal_x - self.map_origin.x) / self.map_res / 3), occ_grid_pooled.shape[1] - 1)
                    goal_pooled_y = min(round((goal_y - self.map_origin.y) / self.map_res / 3), occ_grid_pooled.shape[0] - 1)
                pooled_x = min(round(map_x / 3), occ_grid_pooled.shape[1] - 1)
                pooled_y = min(round(map_y / 3), occ_grid_pooled.shape[0] - 1)
                potential_cells = [(pooled_y, pooled_x)]
                if occ_grid_pooled[pooled_y, pooled_x] >= WALL_THRESHOLD:
                    potential_cells = []
                    for i in range(-1,2):
                        for j in range(-1,2):
                            ny, nx = pooled_y + i, pooled_x + j
                            if 0 <= ny < occ_grid_pooled.shape[0] and 0 <= nx < occ_grid_pooled.shape[1]:
                                if occ_grid_pooled[ny, nx] < WALL_THRESHOLD:
                                    potential_cells.append((ny, nx))
                    #find closest potential cell to robot
                    min_dist = float('inf')
                    for cell in potential_cells:
                        dist = math.sqrt((cell[0] - pooled_y)**2 + (cell[1] - pooled_x)**2)
                        if dist < min_dist:
                            min_dist = dist
                            pooled_y, pooled_x = cell
                    
                self.get_logger().info(f'Map coordinates: x={map_x}, y={map_y}')
            else:
                self.get_logger().warn('Transform from "map" to "base_link" not available yet. Retrying...')
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Transform lookup failed: {e}')

        # Create a start node
        if pooled_x == -1 or pooled_y == -1:
            return []
        
        # Calculate distance to nearest wall for each cell
        for y in range(occ_grid_pooled.shape[0]):
            for x in range(occ_grid_pooled.shape[1]):
                if occ_grid_pooled[y, x] >= WALL_THRESHOLD:
                    wall_distance[y, x] = 0  # This is a wall
                else:
                    # Find minimum distance to any wall
                    min_dist = float('inf')
                    for wy, wx in zip(wall_cells[0], wall_cells[1]):
                        dist = math.sqrt((y - wy)**2 + (x - wx)**2)
                        min_dist = min(min_dist, dist)
                    wall_distance[y, x] = min_dist
        
        #BFS setup
        self.get_logger().info(f'Starting BFS from x={pooled_x}, y={pooled_y}')
        start = MapNode(pooled_x, pooled_y)
        frontier = [start]
        visited = set()
        visited.add(start)
        if len(frontier) == 0:
            return []
        
        
        current_node = None
        while len(frontier) > 0:
            current_node = frontier.pop(0)
            
            visited.add(current_node)
            #self.get_logger().info(f'Current node: x={current_node.x}, y={current_node.y}, map_value={occ_grid_pooled[current_node.x, current_node.y]}')
            if goal is None:
                if occ_grid_pooled[int(current_node.y), int(current_node.x)] == -1:
                    self.get_logger().info(f'Found unknown space at x={int(current_node.x)}, y={int(current_node.y)}')
                    found_path = True
                    break
            else:
                if current_node.x == goal_pooled_x and current_node.y == goal_pooled_y:
                    self.get_logger().info(f'Found goal')
                    found_path = True
                    break

            if occ_grid_pooled[int(current_node.y), int(current_node.x)] >= WALL_THRESHOLD:
                #skip occupied nodes
                continue

            neighbors = current_node.generate_neighbors(occ_grid_pooled.shape[1], occ_grid_pooled.shape[0])
            
            # Sort neighbors by distance to walls (prefer cells farther from walls)
            neighbors.sort(key=lambda n: -wall_distance[int(n.y), int(n.x)])
            
            for neighbor in neighbors:
                #prepare to explore neighboring nodes
                if neighbor in visited:
                    continue
                frontier.append(neighbor)
                visited.add(neighbor)
                neighbor.parent = current_node
        
        path = []
        visualize_path = []
        if not found_path or current_node is None:
            self.get_logger().info('No path found')
            return []
        
        # Reconstruct path and shift nodes away from walls
        while current_node is not None:
            y, x = int(current_node.y), int(current_node.x)
            
            # Shift node away from nearby walls
            shift_x, shift_y = 0, 0
            max_shift = 1  # Maximum shift amount
            wall_influence_distance = 2  # How far walls influence the path
            
            # Check surrounding cells and compute shift vector
            for dy in range(-wall_influence_distance, wall_influence_distance + 1):
                for dx in range(-wall_influence_distance, wall_influence_distance + 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < occ_grid_pooled.shape[0] and 0 <= nx < occ_grid_pooled.shape[1]:
                        if occ_grid_pooled[ny, nx] >= WALL_THRESHOLD:
                            # This is a wall, calculate repulsive force
                            distance = max(0.1, math.sqrt(dy**2 + dx**2))  # Avoid division by zero
                            # Normalize direction vector
                            direction_x = -dx / distance  # Negative to move away
                            direction_y = -dy / distance
                            
                            # Scale by inverse distance (closer walls push harder)
                            magnitude = max_shift * (1.0 / distance)
                            
                            shift_x += direction_x * magnitude
                            shift_y += direction_y * magnitude
            
            # Apply the shift with limits
            shifted_x = x + shift_x
            shifted_y = y + shift_y
            
            # Ensure we don't shift into walls or out of bounds
            if (0 <= shifted_y < occ_grid_pooled.shape[0] and 
                0 <= shifted_x < occ_grid_pooled.shape[1] ):
                if occ_grid_pooled[int(shifted_y), int(shifted_x)] < WALL_THRESHOLD:
                    current_node.x = shifted_x
                    current_node.y = shifted_y
                else:
                    current_node.x = x + 0.25 * shift_x
                    current_node.y = y + 0.25 * shift_y
                
            # Convert pooled grid coordinates back to map coordinates 
            map_x = (current_node.x * 3 + 1.5) * self.map_res + self.map_origin.x
            map_y = (current_node.y * 3 + 1.5) * self.map_res + self.map_origin.y
            
            toAppend = MapNode(map_x, map_y)
            self.get_logger().info(f'Path node: x={toAppend.x}, y={toAppend.y}')
            path.append(toAppend)
            visualize_path.append(MapNode(current_node.x, current_node.y))
            current_node = current_node.parent
        
        path.reverse()
        visualize_path.reverse()
        
        # Smooth the path to remove jagged movements
        if len(path) > 2:
            smoothed_path = [path[0]]  # Keep the first point
            smoothing_window = 3
            for i in range(1, len(path) - 1):
                # Simple moving average for smoothing
                window_start = max(0, i - smoothing_window // 2)
                window_end = min(len(path), i + smoothing_window // 2 + 1)
                window = path[window_start:window_end]
                
                avg_x = sum(node.x for node in window) / len(window)
                avg_y = sum(node.y for node in window) / len(window)
                
                smoothed_path.append(MapNode(avg_x, avg_y))
            
            smoothed_path.append(path[-1])  # Keep the last point
            path = smoothed_path
        
        visited_grid = np.zeros_like(occ_grid_pooled)
        for node in visited:
            visited_grid[int(node.y), int(node.x)] = 100
        
        if visualize:
            np.savetxt('visited_map.txt', visited_grid)
            
            self.visualize_path(visualize_path, occ_grid_pooled)
        
        return path
    
    def visualize_path(self, path, pooled_occ_grid):
        self.get_logger().info('Visualizing path')
        # Create a copy of the occupancy grid
        occ_grid_copy = np.copy(pooled_occ_grid)
        # Mark the path on the occupancy grid
        path_grid = np.copy(occ_grid_copy)
        path_grid[np.where(occ_grid_copy >= 70)] = 100
        start = path.pop(0)
        #self.get_logger().info(f'Path node: x={start.x}, y={start.y}')
        
        for node in path:
            #occ_grid_copy[node.y, node.x] = 200
            #self.get_logger().info(f'Path node: x={node.x}, y={node.y}')
            path_grid[int(node.y), int(node.x)] = 50
        #mark start
        path_grid[int(start.y), int(start.x)] = 75
        #save the copy
        
        np.savetxt('pooled_map.txt', occ_grid_copy)
        np.savetxt('path_map.txt', path_grid)
        
        self.image.set_data(path_grid)
        occ_grid_copy[occ_grid_copy != -1] = 0
        occ_grid_copy[occ_grid_copy == -1] = 1
        np.savetxt('unknown_map.txt', occ_grid_copy)
        
        # Redraw the plot
        plt.draw()
        plt.pause(0.01)  # Allow the GUI event loop to update
    
    def travel_to_node(self, next_node):
        """Traverse the path planned by plan_route()"""
        self.get_logger().info(f'Approaching {next_node.x}, {next_node.y}')
                
        # Get current position in the map frame
        start_x, start_y , current_angle = self.get_orientation()
        distance = math.sqrt((next_node.x - start_x)**2 + (next_node.y - start_y)**2)
        
        # Calculate distance to target
        self.get_logger().info(f'Distance to waypoint: {distance}m')
        
        
        # Calculate angle to target in map frame
        target_angle = math.atan2(next_node.y - start_y, next_node.x - start_x)
        
        # Calculate the angle difference
        angle_diff = math.degrees(target_angle - current_angle)
        # Normalize to [-180, 180]
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        
        if distance < 0.3 and abs(angle_diff) > 130:
            self.get_logger().info('Already at waypoint')
            return
            
        self.get_logger().info(f'Rotating {angle_diff} degrees to face waypoint')
        
        # Rotate to face the target
        # segregate into 10 degree increments
        if abs(angle_diff) < 10: 
            self.rotatebot(angle_diff)
        else:
            times = abs(angle_diff // 10)
            self.get_logger().info(f'Rotating {times} times')
            direction = -1 if angle_diff < 0 else 1
            while(times > 0):
                if self.verify_heat_source(heat_source_radius):
                    self.get_logger().info('Heat source found')
                    return
                self.rotatebot(direction * 10)
                times -= 1
                self.stopbot()

                if self.laser_range[self.back_range].min() < stop_distance and self.laser_range[self.default_front_range].min() > stop_distance:
                    self.get_logger().info('going forward')
                    self.stopbot()
                    twist = Twist()
        
                    # Set linear speed (meters per second)
                    twist.linear.x = speedchange
                    time.sleep(0.2)

                    
                
            self.rotatebot(angle_diff - (direction * 10 * abs(angle_diff // 10)))
        
        # Move towards the target
        twist = Twist()
        
        # Set linear speed (meters per second)
        twist.linear.x = speedchange  # Using the constant defined at the top
        
        # Keep track of distance moved
        distance_moved = 0.0
        
        # Start moving
        while distance_moved < distance:
            # Check for obstacles in front
            # if self.scan_front_obstacle() != None:
            #     self.get_logger().warn('Obstacle detected! Stopping movement')
            #     self.stopbot()
            #     break
                
            # Publish the twist message
            self.publisher_.publish(twist)
            
            # Allow callbacks to execute
            obstacle_direction = self.scan_front_obstacle()
            if obstacle_direction != None:
                if obstacle_direction == "front":
                    self.get_logger().warn('Obstacle detected in front! Stopping movement')
                    #move back for 1 second
                    self.stopbot()
                    twist.linear.x = -speedchange
                    self.publisher_.publish(twist)
                    self.bonk_count += 1
                    time.sleep(0.5)
                    self.stopbot()
                    break
                elif obstacle_direction == "left":
                    self.get_logger().warn('Obstacle detected on the left! Rotating right')
                    self.rotatebot(20, -speedchange/4)
                elif obstacle_direction == "right":
                    self.get_logger().warn('Obstacle detected on the right! Rotating left')
                    self.rotatebot(-20, -speedchange/4)
                elif obstacle_direction == "right edge":
                    self.get_logger().warn('Obstacle detected on the right edge! Rotating left')
                    self.rotatebot(-20, speedchange/10)
                    
                elif obstacle_direction == "left edge":
                    self.get_logger().warn('Obstacle detected on the left edge! Rotating right')
                    self.rotatebot(20, -speedchange/10)
                    
            
            # Get current position in map frame
            try:
                current_x, current_y, _ = self.get_orientation()
                
                # Calculate distance moved
                distance_moved += math.sqrt(
                    (current_x - start_x)**2 + (current_y - start_y)**2
                )
                start_x = current_x
                start_y = current_y
                
            except (LookupException, ConnectivityException, ExtrapolationException, TransformException):
                # If transform fails, just continue with the time-based approach
                pass
            
            # Small delay to prevent flooding
            time.sleep(0.01)

    def get_distance_to_next_node(self, next_node):
        """Calculate the distance to the next node in the path"""
        # Get current position in the map frame
        current_x, current_y, _ = self.get_orientation()
        self.get_logger().info(f'Current position: x={current_x}, y={current_y}')
        
        # Calculate distance to target
        distance = math.sqrt((next_node.x - current_x)**2 + (next_node.y - current_y)**2)
        return distance
    
    def find_path_to_heat_source(self):
        current_x, current_y, angle = self.get_orientation()

        estimated_heat_source = (current_x + self.laser_range[0] * math.cos(angle),
                                current_y + self.laser_range[0] * math.sin(angle))
        self.get_logger().info(f'Estimated heat source: x={estimated_heat_source[0]}, y={estimated_heat_source[1]}')

        path = self.plan_route(visualize=True, goal=MapNode(estimated_heat_source[0], estimated_heat_source[1]))

        return path

    
    def verify_heat_source(self, proximity_threshold=1.2):
        """
        Verify and log heat source detection with robust location tracking
        
        Args:
            proximity_threshold (float): Maximum distance (meters) to consider 
                                        as the same heat source location
        
        Returns:
            bool: True if a new heat source is detected, False otherwise
        """
        try:
            # Check if IR columns have meaningful data
            if self.ir_index == -1:
                return False            
            self.stopbot()
            current_x, current_y, angle = self.get_orientation()

            estimated_heat_source = (current_x + self.laser_range[0] * math.cos(angle),
                                    current_y + self.laser_range[0] * math.sin(angle))
            self.get_logger().info(f'Estimated heat source: x={estimated_heat_source[0]}, y={estimated_heat_source[1]}')

            # Check if this heat source is sufficiently far from previously visited sources
            is_new_source = True
            for visited_source in self.visited_heat_sources:
                # Calculate Euclidean distance between current location and visited sources
                distance = np.sqrt(
                    (estimated_heat_source[0] - visited_source[0])**2 + 
                    (estimated_heat_source[1] - visited_source[1])**2
                )
                
                # If within proximity threshold, consider it the same source
                if distance < proximity_threshold:
                    is_new_source = False
                    break
            
            # If truly a new heat source
            return is_new_source
        
        except Exception as e:
            self.get_logger().error(f'Error in heat source verification: {str(e)}')
            return False
      
    def approach_heat_source(self):
        """Approach a heat source for further investigation"""
        try:
            
            # Slowly Move towards the target
            twist = Twist()
            
            # Set linear speed (meters per second)
            twist.linear.x = speedchange  # Using the constant defined at the top
            self.publisher_.publish(twist)
            start = time.time()
            # Start moving
            while True:
                
                self.get_logger().info('Moving towards heat source of %f speed' % twist.linear.x)
                self.publisher_.publish(twist)
                #align with heat source
                
                #TODO: see which column is on right and left
                
                
                obstacle_direction = self.scan_front_obstacle()
                if obstacle_direction == "front":
                    self.get_logger().warn('Obstacle detected! Stopping movement')
                    self.stopbot()
                    if self.ir_index != -1:
                        self.get_logger().info('Heat source found?')
                        break
                if self.ir_index == -1:
                    #look for heat source
                    self.stopbot()
                    self.rotatebot(2)
                    if time.time() - start > 5:
                        self.get_logger().info('Heat source lost')
                        break
                elif self.ir_index < 3:
                    start = time.time()
                    self.get_logger().info('Heat source on the right')
                    self.rotatebot(2,speedchange/2)
                    continue
                elif self.ir_index > 4:
                    start = time.time()
                    self.get_logger().info('Heat source on the left')
                    self.rotatebot(-2, speedchange/2)
                    continue
                
                obstacle_direction = self.scan_front_obstacle()
                # Check for obstacles in front
                
                if obstacle_direction == "left":
                    
                    self.get_logger().warn('Obstacle detected on the left! Rotating right')
                    self.rotatebot(20, speedchange/6)
                elif obstacle_direction == "right":
                    self.get_logger().warn('Obstacle detected on the right! Rotating left')
                    self.rotatebot(-20, speedchange/6)
                
                
                
            
            if self.verify_heat_source(heat_source_radius) == False:
                self.get_logger().info('Heat source is visited')
                return
            
            #shoot balls
            self.get_logger().info('Heat source is significant')
            distance_from_object = self.laser_range[0]
            
            # Get current position in map frame
            current_x, current_y, current_angle = self.get_orientation()

            #get find coordinates of object in front
            object_x = current_x + distance_from_object * math.cos(current_angle)
            object_y = current_y + distance_from_object * math.sin(current_angle)
            #shoot balls
            
            self.visited_heat_sources.append((object_x, object_y))

            self.save_heat_source_map()

            #signal to rpi to shoot
            self.get_logger().info('Shooting balls')
            msg = Int8()
            msg.data = 1
            self.launching = True
            self.rpi_publisher.publish(msg)
            
            # Stop the TurtleBot
            self.stopbot()
            
        except Exception as e:
            self.get_logger().error(f'Error in approaching heat source: {str(e)}')

    def save_heat_source_map(self):
        """Save the map with visited heat sources"""
        try:
            # Get the occupancy grid
            occ_grid = self.occdata
            
            # Create a copy of the occupancy grid
            occ_grid_copy = np.copy(occ_grid)
            
            # Mark the visited heat sources on the occupancy grid
            for source in self.visited_heat_sources:
                # Convert heat source coordinates to grid coordinates
                grid_x = int((source[0] - self.map_origin.x) / self.map_res)
                grid_y = int((source[1] - self.map_origin.y) / self.map_res)
                
                # Mark the heat source on the grid
                occ_grid_copy[grid_y, grid_x] = 100
            
            # Save the map
            np.savetxt('heat_source_map.txt', occ_grid_copy)
            
        except Exception as e:
            self.get_logger().error(f'Error in saving heat source map: {str(e)}')
    
    def scan_front_obstacle(self):
        """Check for obstacles in front of the TurtleBot"""
        if self.laser_range.size == 0:
            return None
        # Check for obstacles in front
        front_angle = int(65.0/360 * len(self.laser_range))
        total_angle = front_angle * 2
        left_border = int(-front_angle + total_angle/5)
        mid_left_border = int(left_border + total_angle/5 + 5)
        right_border = int(front_angle - total_angle/5)
        mid_right_border = int(right_border - total_angle/5 - 5)

        left_edge_angles = range(-front_angle, left_border)
        left_angles = range(left_border,mid_left_border)
        middle_angles = range(mid_left_border,mid_right_border)
        right_angles = range(mid_right_border, right_border)
        right_edge_angles = range(right_border, front_angle)
        # Get the front angles
        left_edge_ranges = self.laser_range[left_edge_angles]
        left_ranges = self.laser_range[left_angles]
        middle_ranges = self.laser_range[middle_angles]
        right_ranges = self.laser_range[right_angles]
        right_edge_ranges = self.laser_range[right_edge_angles]

        # Check for obstacles in front
        if np.nanmin(middle_ranges) < stop_distance:
            self.get_logger().warn('Obstacle detected in front')
            return "front"
        

        # Check for obstacles on the left
        if np.nanmin(left_ranges) < side_distance:
            if np.nanmin(right_ranges) < np.nanmin(left_ranges):
                self.get_logger().warn('Obstacle detected on the left but right is closer')
                return "right"
            self.get_logger().warn('Obstacle detected on the left')
            return "left"
        # Check for obstacles on the right
        if np.nanmin(right_ranges) < side_distance:
            self.get_logger().warn('Obstacle detected on the right')
            return "right"
        
        if np.nanmin(left_edge_ranges) < side_distance:
            if np.nanmin(right_edge_ranges) < np.nanmin(left_edge_ranges):
                self.get_logger().warn('Obstacle detected on the left edge but right edge is closer')
                return "right edge"
            self.get_logger().warn('Obstacle detected on the left edge')
            return "left edge"
        
        if np.nanmin(right_edge_ranges) < side_distance:
            self.get_logger().warn('Obstacle detected on the right edge')
            return "right edge"

        return None
    
    def return_all_nodes_in_map(self):
        '''Get all nodes in the map and returns a list of all nodes'''
        # Get the occupancy grid
        occ_grid = self.occdata
        # Create a list to store the nodes
        nodes = []
        # Iterate through the occupancy grid
        #pad the occupancy grid to have size divisible by 3
        occ_grid = F.pad(occ_grid, (0, 3 - occ_grid.shape[1] % 3, 0, 3 - occ_grid.shape[0] % 3), value=-1) 
        # Apply max pooling to the occupancy grid
        occ_grid = occ_grid.reshape(1,1,occ_grid.shape[0], occ_grid.shape[1])
        occ_grid_pooled = F.max_pool2d(occ_grid, 3,3).reshape(occ_grid.shape[2]//3, occ_grid.shape[3]//3)
        
        # Iterate through the occupancy grid
        for y in range(occ_grid_pooled.shape[0]):
            for x in range(occ_grid_pooled.shape[1]):
                # Check if the cell is wall
                if occ_grid_pooled[y, x] <= 70:
                    # Convert pooled grid coordinates back to map coordinates 
                    map_x = (x * 3 + 1.5) * self.map_res + self.map_origin.x
                    map_y = (y * 3 + 1.5) * self.map_res + self.map_origin.y
                    # Create a MapNode object and add it to the list
                    node = MapNode(map_x, map_y)
                    nodes.append(node)
        return nodes
    
    def mover(self):


        '''traversal code'''
        start_time = time.time()
        self.path = self.plan_route(True)
        
        while len(self.path) == 0:
            self.path = self.plan_route(True)
            if time.time() - start_time > 10:
                
                self.get_logger().info('No path found from start') 
                return
            
        final_list = []
        final_gambit = False

        while rclpy.ok():
            self.get_logger().info('not 2 heatsources : %s\n heat source detected:%s\n new heat source:%s\n' % 
                                   (len(self.visited_heat_sources)!=1, self.ir_index != -1, self.verify_heat_source(heat_source_radius)))
            
            while self.launching == True:
                self.get_logger().info('Waiting for launch to end')
                time.sleep(0.5)
            
            
            
            
            #renews path normally
            while len(self.path) == 0 and not final_gambit:
                tries = 10
                while tries > 0:
                    self.get_logger().info('No path found, trying again')
                    self.stopbot()
                    self.path = self.plan_route(True)
                    if len(self.path) != 0:
                        break
                    tries -= 1

                #map should be fully explored but heat sources are not found
                if tries == 0 and len(self.visited_heat_sources) != 2:
                    self.get_logger().warn('No path found, trying final gambit')
                    final_list = self.return_all_nodes_in_map()
                    final_gambit = True

            #normally get next node
            if len(self.path) != 0:                
                self.next_coords = self.path.pop(0)
                self.get_logger().info('Next node is: x=%f, y=%f'% (self.next_coords.x, self.next_coords.y))

            if len(self.visited_heat_sources) != heat_sources and self.verify_heat_source(heat_source_radius):
                heat_source_path = self.find_path_to_heat_source()
                while len(heat_source_path) != 0:
                    self.get_logger().info('Found heat source, approaching')
                    self.next_coords = heat_source_path.pop(0)
                    self.get_logger().info('Next node is: x=%f, y=%f'% (self.next_coords.x, self.next_coords.y))
                    self.travel_to_node(self.next_coords)

                if np.min(self.laser_range) < 0.2:
                    self.get_logger().info('too close to wall')
                    min_angle = np.argmin(self.laser_range)/len(self.laser_range) * 360
                    self.rotatebot(min_angle)
                    to_publish = Twist()
                    to_publish.linear.x = -speedchange
                    self.publisher_.publish(to_publish)
                    time.sleep(0.2)
                    self.stopbot()
                    
                
                if self.ir_index != -1:
                    self.get_logger().info('Heat source found, approaching')
                    self.approach_heat_source()
                else:
                    for _ in range(36):
                        self.rotatebot(10)
                        if self.verify_heat_source(heat_source_radius):
                            self.get_logger().info('Heat source found')
                            self.approach_heat_source()
                            self.path = self.plan_route(True)
                            break

            
            elif not final_gambit:

                if random.random() < 0.05:
                    self.get_logger().info('Replanning route due to random chance')
                    self.path = self.plan_route(True)
                    continue

                if random.random() < 0.007 and not np.any(self.laser_range < 0.15):

                    toSpin = True
                    current_x, current_y, _ = self.get_orientation()
                    for spins in self.spin_coords:
                        dist = math.sqrt((spins[0] - current_x)**2 + (spins[1] - current_y)**2)
                        if dist < 0.5:
                            self.get_logger().info('Already spun here')
                            toSpin = False
                            break
                    if toSpin:
                        self.get_logger().warn('autistic spin start')
                        found = False
                        for _ in range(0,36):
                            self.rotatebot(10)
                            if self.ir_index != -1:
                                found = True
                                self.get_logger().info('Heat source detected maybe')
                                break
                        self.get_logger().warn('autistic spin end')
                        self.spin_coords.append((current_x, current_y))
                        if found:
                            continue
                
                self.travel_to_node(self.next_coords)
                
                
            else:
                if len(final_list) == 0 and len(self.path) == 0:
                    self.get_logger().warn('Self destructing')
                    break
                if len(self.path) == 0:
                    new_dest = final_list.pop(0)
                    self.path = self.plan_route(True, new_dest)
                    if len(self.path) == 0:
                        continue
                self.travel_to_node(self.next_coords)

            if self.bonk_count > 4:
                self.get_logger().warn('Bonked too many times, replanning')
                self.path = self.plan_route(True)
                self.bonk_count = 0
            
            if len(self.visited_heat_sources) == heat_sources:
                if len(self.plan_route(True)) == 0:
                    self.stopbot()
                    self.get_logger().info('Mission Success!')
                    return
                
        
        self.stopbot()
        

def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    try:
        auto_nav.mover()
    except KeyboardInterrupt:
        auto_nav.get_logger().info('Keyboard interrupt, shutting down')

    finally:
        auto_nav.stopbot()

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
