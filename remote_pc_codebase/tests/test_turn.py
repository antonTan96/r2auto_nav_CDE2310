import rclpy
def test_turn(auto_node):
    '''tests the path planning algorithm once'''
    while rclpy.ok():
        auto_node.rotatebot(90)
        auto_node.get_logger().info('turned 90 degrees')
        return        
        
    
