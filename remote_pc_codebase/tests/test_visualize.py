import rclpy
def test_visualize(auto_node):
    '''tests visualization '''
    while rclpy.ok():
        auto_node.path = auto_node.plan_route(True)
        