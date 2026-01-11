# AAV Drone Software

This was inspired by a Duke Robotics Repo linked [HERE](https://github.com/DukeRobotics/turtlesim-ros2-public) 

## RQT Graph Diagram
[View Diagram Here](https://miro.com/app/board/uXjVIjMwI14=/?focusWidget=3458764645715611275)

## Overview
This repository serves as the main development environment for the AAV Drone

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

## Launch Files (Run the software!)
### aav_simulation_launch
Used to run the full software setup with Gazebo, yolo, and AAV Software Nodes. Run this on a computer with Ubuntu Linux and an NVIDIA Graphics card.
#### Command:
ros2 launch aav_bringup aav_simulation_launch

### aav_drone_launch
Used to run the software needed on the actual drone. Runs yolo and AAV Software Nodes. Run this on the Jetson located on the drone itself.
#### Command:
ros2 launch aav_bringup aav_drone_launch

## Setup and Run Simulation
[Follow Slideshow Tutorial](https://docs.google.com/presentation/d/1nkopkXmOMbf2FSYXp7-02WSqbt8vuJUTEI2T_HQRqNE/edit?usp=sharing)

## Setup Drone
[Follow Slideshow Tutorial](https://docs.google.com/presentation/d/1RjGnmdPsTrMh_nokPCGsJA6VWc1nO-TS1V9CDui4KNo/edit?usp=sharing)
