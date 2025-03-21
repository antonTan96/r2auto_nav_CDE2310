def path_test(auto_node):
    while len(auto_node.path) == 0:
        auto_node.path = auto_node.plan_route()    
