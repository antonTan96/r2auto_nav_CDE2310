import rclpy
import time
def full_navigation(auto_node):
    '''tests the path planning algorithm until the bot explores the entire map'''
    start = time.time()
    while start - time.time() < 10 and len(auto_node.path) < 1:
        auto_node.path = auto_node.plan_route()  
    
    if len(auto_node.path) < 1:
        auto_node.get_logger().info('No path found')
        return
    
    auto_node.nextcoords = auto_node.path[1]
    auto_node.get_logger().info('Next node is: x=%f, y=%f'% (auto_node.nextcoords.x, auto_node.nextcoords.y))
    while rclpy.ok():
            auto_node.travel_to_node(auto_node.nextcoords)
            auto_node.path.pop(0)
            if len(auto_node.path) == 1:
                tries =  10
                while tries != 0:
                    
                    auto_node.path=auto_node.plan_route()
                    if len(auto_node.path) == 0:
                        tries -= 1
                        time.sleep(1)
                    else:
                        break
                if tries == 0:
                    auto_node.get_logger().info('No path found')
                    break
            auto_node.nextcoords = auto_node.path[1]
    auto_node.stopbot()