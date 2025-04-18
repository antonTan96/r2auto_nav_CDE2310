# Remote PC setup instructions

1. Install ROS2 and Turtlebot3 packages by following the instructions [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) 

2. Create a new directory to clone the auto_nav package.
``` bash 
$ mkdir -p ~/colcon_ws/src 
```

3. Create a ros2 package and git clone auto_nav code into the newly created package.
```bash
$ cd ~/colcon_ws/src
$ ros2 pkg create --build-type ament_python auto_nav
$ cd ~/colcon_ws/src/auto_nav/auto_nav
$ git clone git@github.com:antonTan96/r2auto_nav_CDE2310.git . -b base_anton

```

4. Replace the package.xml and setup.py files with symbolic links to the version in the repository.
```bash
$ cd ~/colcon_ws/src/auto_nav
$ rm package.xml setup.py
$ ln -s auto_nav/package.xml .
$ ln -s auto_nav/setup.py .
```

5. You can now build the package on your laptop. 
```bash
$ cd ~/colcon_ws
$ colcon build
```

6. Run auto_nav using the following commands.
```bash
$ source ~/colcon_ws/install/setup.bash
$ ros2 run auto_nav r2auto_nav
```
