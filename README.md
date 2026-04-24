# AAV Drone Software

This was inspired by a Duke Robotics Repo linked [HERE](https://github.com/DukeRobotics/turtlesim-ros2-public) 

## RQT Graph Diagram
[View Diagram Here](https://miro.com/app/board/uXjVIjMwI14=/?focusWidget=3458764645715611275)

## ROS2 Cheatsheet
[Cheatsheet](https://docs.google.com/document/d/1DnRJy_DEjzJgxxBqZH9xCd9jRVEWYbPsAdSq6WGnnYM/edit?usp=sharing)

## Overview
This repository serves as the main development environment for the AAV Drone

### Software Implementation Summaries
[C-UASC Summary](https://docs.google.com/document/d/1i0-yuOBz0uaueun8jnia9QPYaHs8-D_kmw1PnQ6_2wI/edit?usp=sharing)
[SUAS Summary](https://docs.google.com/document/d/1u48MPF5nE5hkrVeKE1iGp-DRbENpCMyZmo32L6hcvU8/edit?usp=sharing)

## Develop in Docker

### SETUP with GitHub Codespaces
1. Open this repository on GitHub.
2. To create the codespace for the first time (skip to step 3 if you've already created the codespace):
    1. Click the green "Code" button.
    2. Switch to the "Codespaces" tab.
    3. Click the "+" button to create a new codespace.
    4. A new tab will open with the codespace. It will take approximately five minutes to build the container and prepare the codespace.
    5. Once the codespace is ready, you will see the terminal in the bottom of the window, and you will be in the `/workspaces/{repo-name}` directory.
    6. If you are prompted to install the recommended extensions, click "Install All" to install the recommended extensions. If you are not prompted, you can install the recommended extensions by clicking the extensions icon on the left sidebar, searching for "@recommended" in the search bar, and clicking the cloud icon next to "Workspace Recommendations".
3. To open the codespace after it has been created:
    1. Click the green "Code" button.
    2. Switch to the "Codespaces" tab.
    3. Click on the codespace you want to open.
    4. Optional: If you would like to open the codespace in VS Code on your local machine, perform steps 1-2, click the three dots on the right side of the codespace and select "Open with Visual Studio Code".
4. After you have finished working, close the tab with the codespace or close the VS Code window if you opened the codespace in VS Code.
5. To stop the codespace:
    1. Go back to the repository on GitHub.
    2. Click the "Code" button.
    3. Switch to the "Codespaces" tab.
    4. Click the three dots on the right side of the codespace and select "Stop codespace".

### Desktop Viewer
If you're using GitHub Codespaces, to view the RQT_GRAPH, perform the following:
1. Press Ctrl/Cmd + Shift + P to open the command palette.
2. Type ">Ports" and select "Ports: Focus on Ports View".
3. In the "Ports" view, right-click on `6080` in the first column and select "Open in Browser" to open the desktop in a new tab. Alternatively, select "Preview in Editor" to view the desktop in the codespace.
5. You may need to wait a few minutes for the desktop to load.
6. Once you see the "noVNC" logo, click the "Connect" button to open the desktop.
7. The RQT_GRAPH will be visible in the desktop (if you've started the RQT_GRAPH).

## Test AAV Software Code
Use colcon test to do unit tests and Python lint checking.
```
cd aav_ws/
colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
```

## AAV Software Python Files
### Location Logger
Used to log the locations and labels of objects/targets we detect through our vision recognition.

### Manav's Magic Code
Used to detect the latitude and longitude of objects/targets on the ground that are detected through our vision recognition.

### Object Alignment Controller
This is the main software that controls the drone's autonomous logic. It is designed as a Finite State Machine. [Link to diagram of Finite State Machine](https://miro.com/app/board/uXjVIjMwI14=/?focusWidget=3458764648320440275).

### Topic Converter
This acts as a bridge between our software and either the simulation or the actual drone. It handles converting our own topics into either Ardupilot or MAVROS topics.

## Launch Files (Run the software!)
### aav_simulation_launch
Used to run the full software setup with Gazebo, yolo, and AAV Software Nodes. Run this on a computer with Ubuntu Linux and an NVIDIA Graphics card. You can edit the launch file directly in order to try out different missions.
#### Command:
```
ros2 launch aav_bringup aav_simulation_launch.py
```

### aav_drone_cuasc_package_delivery_drone_launch
Used to run the software needed on the actual drone. Runs yolo and AAV Software Nodes. Run this on the Jetson located on the drone itself.
Runs the package delivery mission for the C-UASC competition
#### Command:
```
ros2 launch aav_bringup aav_drone_cuasc_package_delivery_drone_launch.py
```

### aav_drone_cuasc_payload_drop_launch
Used to run the software needed on the actual drone. Runs yolo and AAV Software Nodes. Run this on the Jetson located on the drone itself.
Runs the payload drop mission for the C-UASC competition
#### Command:
```
ros2 launch aav_bringup aav_drone_cuasc_payload_drop_launch.py
```

### aav_drone_cuasc_gcp_marker_detection_launch
Used to run the software needed on the actual drone. Runs yolo and AAV Software Nodes. Run this on the Jetson located on the drone itself.
Runs the gcp marker mission for the C-UASC competition
#### Command:
```
ros2 launch aav_bringup aav_drone_cuasc_gcp_marker_detection_launch.py
```

### aav_drone_suas_launch
Used to run the software needed on the actual drone. Runs yolo and AAV Software Nodes. Run this on the Jetson located on the drone itself.
Runs the search and rescue mission for the SUAS competition
#### Command:
```
ros2 launch aav_bringup aav_drone_suas_launch.py
```

### aav_drone_manual_suas_launch
Used to run the software needed on the actual drone. Runs yolo and AAV Software Nodes. Run this on the Jetson located on the drone itself.
This disables any autonomous aligning logic. Used for manual alignment.
#### Command:
```
ros2 launch aav_bringup aav_drone_manual_suas_launch.py
```

## Setup and Run Simulation
[Follow Slideshow Tutorial](https://docs.google.com/presentation/d/1nkopkXmOMbf2FSYXp7-02WSqbt8vuJUTEI2T_HQRqNE/edit?usp=sharing)

### Pull new code and rebuild
Type these commands

```
cd aavDroneSoftware/
git pull
cd aav_ws/
colcon build
```

## Setup Drone
[Follow Slideshow Tutorial](https://docs.google.com/presentation/d/1RjGnmdPsTrMh_nokPCGsJA6VWc1nO-TS1V9CDui4KNo/edit?usp=sharing)
