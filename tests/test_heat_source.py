import rclpy

def test_heat_source_detection(auto_node):

    auto_node.get_logger().info("Testing heat source approach")
    auto_node.approach_heat_source()
    for heat_source in auto_node.visited_heat_sources:
        auto_node.get_logger().info("heat source is at x=%f, y=%f" % (heat_source.x, heat_source.y))
    auto_node.get_logger().info("Current position: x=%f, y=%f" % (auto_node.coords[0], auto_node.coords[1]))
