# CDE2310 Turtlebot ICBM Introduction
## Introduction
### Mission Overview
In this project, we are required to design a Turtlebot system to navigate a disaster zone maze, detect heat signals through an opaque acrylic wall, and deploy flares in a 2-4-2 pattern as a signal that the bot has reached its targets. The Turtlebot will operate without line-following capabilities and it will only be relying on sensors to navigate and perform its tasks. There will also be an added challenge, which requires the robot to scale a ramp using a 2D LiDAR.
### Problem Definition
![image](https://github.com/user-attachments/assets/2f851125-9ab7-42b5-9e77-ed3f52621180)
The Maze Zone consists of 4 parts: 
- Start Zone: This is where the Turtlebot 3 will be deployed at the start of every mission.
- Randomized Maze Zone: Obstacles will be randomly placed to obstruct navigation of the Turtlebot. The Turtlebot must not run into any of the obstacles during navigation.
- Survivor Zone: This is where a heat source will be placed. The Turtlebot will have to locate a heat source in this zone and commence the flare firing sequence of 2-4-2s pattern. There are 3 Survivor Zones in the Maze Zone, with the 3rd Survivor Zone being optional to detect.
- Ramp Zone: Connected to the 3rd Survivor Zone, the Turtlebot has to scale a ramp to access the 3rd Survivor Zone. This section is optional.
	The maze zone will have the following dimensions labeled in the figure shown below:
![image](https://github.com/user-attachments/assets/f272488c-d5a2-43f1-a5f9-157028748428)
### 

