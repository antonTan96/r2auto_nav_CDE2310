import math
import rclpy
def path_test(auto_node):
    '''tests the path planning algorithm once'''
    while rclpy.ok():
        
        auto_node.path = auto_node.plan_route(visualize=True)

        if len(auto_node.path) != 0:
            auto_node.get_logger().info('exiting')
            break
        
    auto_node.get_logger().info('Current position: x=%f, y=%f' % (auto_node.coords[0], auto_node.coords[1]))
    auto_node.get_logger().info('Next node is: x=%f, y=%f'% (auto_node.nextcoords.x, auto_node.nextcoords.y))
    auto_node.get_logger().info("next node types are: x=%s, y=%s" % (type(auto_node.nextcoords.x), type(auto_node.nextcoords.y)))
    #calculate distance from current to next
    distance = math.sqrt((auto_node.nextcoords.x - auto_node.coords[0])**2 + (auto_node.nextcoords.y - auto_node.coords[1])**2)
    auto_node.get_logger().info('Distance to next node: %f' % distance)
    
