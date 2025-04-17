import rclpy
import time
def test_heat_source_approach(auto_node):
    '''
    tests the heat source approach algorithm
    test with teleop on another terminal
    '''
    
    auto_node.get_logger().info("Testing heat source approach")
    start = time.time()
    while rclpy.ok() and len(auto_node.visited_heat_sources) != 2:
        if time.time() - start > 1:
            auto_node.get_logger().info("searching for heat source")
            start = time.time()
            
        if auto_node.verify_heat_source():
            auto_node.get_logger().warn("Heat source found")
            auto_node.approach_heat_source()
            auto_node.get_logger().info("heat source approached")
            break