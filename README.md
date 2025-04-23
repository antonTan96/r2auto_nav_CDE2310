# CDE2310 Turtlebot ICBM Introduction
# Introduction
## Mission Overview
In this project, we are required to design a Turtlebot system to navigate a disaster zone maze, detect heat signals through an opaque acrylic wall, and deploy flares in a 2-4-2 pattern as a signal that the bot has reached its targets. The Turtlebot will operate without line-following capabilities and it will only be relying on sensors to navigate and perform its tasks. There will also be an added challenge, which requires the robot to scale a ramp using a 2D LiDAR.
## Problem Definition
![image](https://github.com/user-attachments/assets/2f851125-9ab7-42b5-9e77-ed3f52621180)
The Maze Zone consists of 4 parts: 
- Start Zone: This is where the Turtlebot 3 will be deployed at the start of every mission.
- Randomized Maze Zone: Obstacles will be randomly placed to obstruct navigation of the Turtlebot. The Turtlebot must not run into any of the obstacles during navigation.
- Survivor Zone: This is where a heat source will be placed. The Turtlebot will have to locate a heat source in this zone and commence the flare firing sequence of 2-4-2s pattern. There are 3 Survivor Zones in the Maze Zone, with the 3rd Survivor Zone being optional to detect.
- Ramp Zone: Connected to the 3rd Survivor Zone, the Turtlebot has to scale a ramp to access the 3rd Survivor Zone. This section is optional.
	The maze zone will have the following dimensions labeled in the figure shown below:
![image](https://github.com/user-attachments/assets/f272488c-d5a2-43f1-a5f9-157028748428)
## System Requirements Analysis
### Project Deliverables
| Mission Requirements | Project Deliverables |
|----------|----------|
| Traverse Map Autonomously    | System shall use mapping techniques to create a map of maze environment     |
| Navigate through Maze autonomously    | System shall have capability for obstacle detection and avoidance during navigation and traversal     |
| Identify heat sources | System shall have capability for detecting presence of heat soources  |
| Navigate towards heat sources | System shall have capability for identifying location and bearing of heat sources and navigate towards heat sources |
| Launch ping pong balls near heat sources | System shall have capability for launching 3 ping pong balls in a 2-4-2 delay pattern for each survivor, for a maximum of 3 survivors with 9 balls |
| Recognise visited heat sources | System shall recognise heat sources that it has visited and not launch ping pong balls at the same survivor |
### Functional Requirements
| Mission Requirements | Functional Requirements |
|----------|----------|
| Traverse Map Autonomously    | System shall use Simultaneous Localization and Mapping to create a map of maze environment     |
| Navigate through Maze autonomously    | System shall utilise developed map to promptly detect obstacles and navigate around them to ensure smooth traversal.     |
| Identify heat sources | System shall use IR Camera to detect presence of heat soources  |
| Navigate towards heat sources | System shall have capability for identifying location of heat sources via temperature data from IR Camera and localisation data from SLAM to navigate towards heat sources |
| Launch ping pong balls near heat sources | System shall have capability for launching 3 ping pong balls in a 2-4-2 delay pattern for each survivor, for a maximum of 3 survivors with 9 balls |
| Recognise visited heat sources | System shall use location data to recognise heat sources that it has visited and not launch ping pong balls at the same survivor |
### Non-functional Requirements
| **Category**        | **System Requirement**                                                                                   |
|---------------------|----------------------------------------------------------------------------------------------------------|
| **Performance**     | System shall accomplish the entire mission, including calibration of IR sensors, generation of SLAM map, within a time limit of 25 minutes. |
| **Reliability**     | System shall operate reliably under various environmental conditions, including different maze configurations and lighting conditions. <br> System shall be resilient to sensor noise and external disturbances. |
| **Safety**          | System shall prioritise safety during operation to prevent collisions with obstacles and map elements.    |
| **Scalability**     | System shall be designed with scalability in mind to accommodate future enhancements or modifications. <br> System shall be capable of adapting to changes in maze layouts or task requirements. |
| **Usability**       | System shall have a user-friendly interface for easy setup, operation, and monitoring by human operators. |
| **Accuracy**        | System shall achieve high accuracy in map generation, obstacle detection, and task execution. <br> System shall minimise errors in localization and navigation to ensure precise performance. |
| **Maintainability** | System shall be designed for ease of maintenance and repair. <br> System shall allow for quick diagnosis and replacement of any faulty components. |
| **Power Efficiency**| System shall optimise power usage to prolong battery life by employing energy-efficient algorithms and hardware components. |
### Constraints
| **Constraint**                    | **Description**                                                                                                                                                                  |
|----------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Power Supply Limitations**     | The robot must operate within constraints imposed by the power source, such as the battery capacity and voltage requirements.                                                   |
| **Sensor and Actuator Limitations** | The selection and integration of sensors and actuators are constrained by factors such as cost, availability, and compatibility with the robot operating platform - ROS2   |
| **Processing Power and Memory**  | The onboard processing capabilities and memory resources of the robot system component (Raspberry Pi) are limited, affecting the complexity and efficiency of algorithms.       |
| **Environmental**                | Environmental factors such as ambient temperatures may affect sensor performance, navigation accuracy, and reliability. |
| **Cost**                         | The development and deployment of the robot system are constrained by a budget limitation of $80. Cost-effective solutions and resource prioritization are essential.         |

# System Overview (TODO)
## High Level Concept
![image](https://github.com/user-attachments/assets/6d46b0d1-4d58-4254-a3af-e8cf655599b5)

![image](https://github.com/user-attachments/assets/a5fae769-6c52-4238-b840-5f4d292ab051)
Building upon the Turtlebot frame, we utilise an IR Camera to identify heat sources in the maze and a spring launcher mechanism actuated by servos to launch the ping pong balls upon detection and navigation to the survivor.
## High Level Algorithm
![image](https://github.com/user-attachments/assets/2841cfe0-a33c-4b37-a0d7-b50b7e4c347b)
![image](https://github.com/user-attachments/assets/f210a045-70a0-42b1-8de7-d4162eecaf18)

## Technical Specifications
| **Technical Components** | **Details**                                                                                       |
|--------------------------|---------------------------------------------------------------------------------------------------|
| **Model**                | TurtleBot3 Burger ICBM V1.2                                                                       |
| **Software Version**     | 1.4.7                                                                                             |
| **Weight**               | 1606.63g                                                                                          |
| **Battery Capacity**     | Lithium polymer 11.1V 1800mAh / 19.98Wh                                                           |
| **Max Runtime**          | 1.5 – 2 hours                                                                                     |
| **Charging Time**        | ~2.5 hours                                                                                        |
| **Purpose**              | 1. To autonomously navigate the maze and avoid obstacles<br>2. Successfully find 3 heat signatures<br>3. Launches ping pong balls at predetermined intervals when heat signature found |


# Electrical Subsystem
- [Electrical Documentation](https://github.com/antonTan96/r2auto_nav_CDE2310/tree/main/elec_doc)
# Mechanical Subsystem
- [Mechanical Documentation](https://github.com/antonTan96/r2auto_nav_CDE2310/tree/main/mech_doc)
# Software Codebases
- [Remote PC Codebase](https://github.com/antonTan96/r2auto_nav_CDE2310/tree/main/remote_pc_codebase)
- [Raspberry Pi Codebase](https://github.com/antonTan96/r2auto_nav_CDE2310/tree/main/rpi_codebase)
# More Documentation
- [User Guide](https://github.com/antonTan96/r2auto_nav_CDE2310/blob/main/general_doc/User%20Guide.pdf)
- [Budget Breakdown](https://github.com/antonTan96/r2auto_nav_CDE2310/blob/main/general_doc/Budget%20Breakdown.xlsx)
