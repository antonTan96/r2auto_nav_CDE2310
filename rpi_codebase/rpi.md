# Raspberry Pi Setup instructions

1. Install Turtlebot3 and ROS2 packages by following the instructions [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup)

2. Install other hardware packages.
```bash
$ pip install adafruit-circuitpython-amg88xx
$ pip install adafruit-circuitpython-pca9685
$ pip install adafruit-circuitpython-servokit
``` 

3. Create a ROS2 package and clone the `rpi_code` branch into the package
```bash
$ mkdir -p ~/colcon_ws/src 
$ cd ~/colcon_ws/src
$ ros2 pkg create --build-type ament_python ir_cam
$ cd ~/colcon_ws/src/ircam/ircam
$ git clone git@github.com:antonTan96/r2auto_nav_CDE2310.git . -b rpi_code
```

4. Replace the package.xml and setup.py files with symbolic links to the version in the repository.
```bash
$ cd ~/colcon_ws/src/ircam
$ rm package.xml setup.py
$ ln -s ircam/package.xml .
$ ln -s ircam/setup.py .
```

5. Build the package on Raspberry Pi. 
```bash
$ cd ~/colcon_ws
$ colcon build
$ source ~/colcon_ws/install/setup.bash
```

6. Modify your `~/.bashrc` file to have the following line:
```bash
export TURTLEBOT3_MODEL=burger
alias rosbu='ros2 launch turtlebot3_bringup robot.launch.py & ros2 run ircam ircam_on'
```

7. Create a symbolic link from `reload.py` into the home directory
```bash
ln -s ~/colcon_ws/src/ircam/ircam/reload.py ~
```
