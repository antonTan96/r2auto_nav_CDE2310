# Developer Guide for Turtlebot ICBM Navigation
This document describes the high-level design of the navigation constructs of the Turtlebot ICBM.

## Map Node
API: `mapNode.py`

A helper class to encapsulate coordinates during the BFS process in `plan route`

### Attributes:
* `x` : The x coordinates of the Node in the map published by map **or** with respect to map_frame
* `y` : the y coordinates of the Node in the map published by map **or** with respect to map_frame
* `parent` : The parent of the Node, used in search.

### Methods:
####  get_neighbors():
* Arguments:
    * `max_x` : the maximum value along the x_axis of the map
    * `max_y` : the maximum value along the y_axis of the map
* Returns: 
    * A list of neighboring nodes in the map.

## AutoNav Node
API: `r2auto_nav.py`

The main navigation control node for the Turtlebot ICBM.

### Tunable parameters
* `heat_source_radius` : The minimum distance (meters) between heat sources, used to verify whether a heat source has been detected or not.
* `heat_sources` : The number of heat sources to detect.
* `rotatechange` : The angular velocity (radians/second) of the Turtlebot ICBM when rotating.
* `speedchange` : The linear velocity (meters/second) of the Turtlebot ICBM when going in a straight line
* `stop_distance`, `side_distance`, `edge_distance` : The threshold distances of whether there is an object in front/side/edge of the Turtlebot ICBM.

### Attributes
* `publisher_` : A `Twist` publisher that controls movement of the Turtlebot ICBM. 
* `path` : A list of nodes that the Turtlebot ICBM should navigate to.
* `next_coords` : The next node that the Turtlebot ICBM will navigate to.
* `bonk_count` : A counter recording the amount of times the Turtlebot ICBM has almost ran into a wall. `path` is recalculated if a certain count is exceeded.
* `spin_coords` : A list of nodes that the Turtlebot ICBM has spun at.
* `occ_subscription` : A subscriber that subscribes to the `map` topic.
* `occ_data` : Represents the map occupancy from data provided by `occ_subscription`.
* `map_res`, `map_origin` : map data used to determine the location of the Turtlebot ICBM in `occ_data`.
* `scan_subscription` : A subscriber that subscribes to the `scan` topic.
* `laser_range` : A list of distances from each angle of the Turtlebot ICBM.
* `back_range` : A list of angles representing the back side of the Turtlebot ICBM.
* `default_front_range` : A list of angles representing the front side of the Turtlebot ICBM, only used when the Turtlebot ICBM is potentially stuck.
* `tf_buffer`, `tf_listener` : Used to determine the coordinates of the Turtlebot3 ICBM with respect to the `map` frame.
* `visited_heat_sources` : A list of coordinates representing the location of heat sources already identified.
* `ir_index` : An integer representing the column of the highest temperature that exceeds the threshold detected in the heat map. The value is -1 if there is no heat source identified.
* `heat_map_subscriber` : A subscriber that subscribes to the `heatdir` topic.
* `launching` : Represents whether the Turtlebot ICBM is currently firing flares.
* `rpi_publisher` : A `Int8` publisher that signals the flare launching sequence.
* `rpi_subscriber` : A subscriber that subscribes to the `move` topic.


### Methods

#### get_orientation
* Arguments: None
* Returns: The x and y coordinates of the Turtlebot3 ICBM and its current angle with respect to the `map` frame

#### rotate_bot
* Arguments: 
    * `rot_angle` : The angle (degrees) of which the Turtlebot3 ICBM should rotate.
    * `speed` : The linear velocity (meters/second) of the Turtlebot3 ICBM when rotating.
* Returns: None
* Effects: The Turtlebot3 ICBM will rotate `rot_angle` degrees

#### stop_bot
* Arguments: None
* Returns: None
* Effects: The Turtlebot3 ICBM will stop in place.

#### plan_route
* Arguments: 
    * `visualize` : Whether the planned route should be displayed/saved
    * `goal` : The coordinates that the Turtlebot ICBM wants to navigate to.
* Retuns: A list of nodes to travel to until the goal is reached. If no goal is specified, a path to the nearest unexplored area is returned.
* Effects: If `visualize` is True, then copies of maps containing different information will be saved.
![plan_route diagram](.\flow_chart_diagrams\plan_route_flow_chart_diagram.png)

