# AAV Drone Software

## RQT Graph Diagram
[View Diagram Here](https://miro.com/app/board/uXjVIjMwI14=/?focusWidget=3458764645715611275)

## ROS2 Cheatsheet
[Cheatsheet](https://docs.google.com/document/d/1DnRJy_DEjzJgxxBqZH9xCd9jRVEWYbPsAdSq6WGnnYM/edit?usp=sharing)

## Overview
This repository serves as the main development environment for the AAV Drone

### Software Implementation Summaries
- [C-UASC Summary](https://docs.google.com/document/d/1i0-yuOBz0uaueun8jnia9QPYaHs8-D_kmw1PnQ6_2wI/edit?usp=sharing)
- [SUAS Summary](https://docs.google.com/document/d/1u48MPF5nE5hkrVeKE1iGp-DRbENpCMyZmo32L6hcvU8/edit?usp=sharing)

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

### aav_drone_manual_cuasc_launch
Used to run the software needed on the actual drone. Runs yolo and AAV Software Nodes. Run this on the Jetson located on the drone itself.
This disables any autonomous aligning logic. Used for manual alignment and detecting of GCP markers.
#### Command:
```
ros2 launch aav_bringup aav_drone_manual_cuasc_launch.py
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


## In-Depth Repo Explanation
Explantation:
The above link is the main development environment we work in. This will be the GitHub repo you will spend the most time working in. It is the heart of the software team and makes everything on the drone work.

AAV ROS2 Message Interfaces
https://github.com/AAV-VirginiaTech/aavDroneSoftware/tree/main/aav_ws/src/aav_msgs

This folder contains all of our custom ROS2 message interfaces. These are used within our /AAV ROS2 topics. You can edit .msg files to change the format of the ROS2 interfaces.

AAV Software
https://github.com/AAV-VirginiaTech/aavDroneSoftware/tree/main/aav_ws/src/aav_software

This folder contains all of our Python programs that actually make everything we do work. These are the main files that the software team will be editing.

Below is a link to an RQT Graph diagram that explains how all of our software connects together:
https://miro.com/app/board/uXjVIjMwI14=/?focusWidget=3458764645715611275


Object Alignment Controller:
This is the main file that actually does the logic for our autonomous navigation and alignment.
You will definitely have to adapt this code in order to fit new competition guidelines and missions.
It is a Finite State Machine.

Below is a diagram for the object alignment controller:
https://miro.com/app/board/uXjVIjMwI14=/?focusWidget=3458764648320440275

AAV Test
https://github.com/AAV-VirginiaTech/aavDroneSoftware/tree/main/aav_ws/src/aav_software/test

This folder contains all of our unit tests for our code. Adding more unit tests in this folder might be a good task to give to software members. Right now, we just have some very basic unit tests (fully generated by ChatGPT) and also linting (Ruff) and type-checking (PyLance) tests.

AAV Bringup
https://github.com/AAV-VirginiaTech/aavDroneSoftware/tree/main/aav_ws/src/aav_bringup

launch Folder:
This folder contains the ROS2 launch files. This is the main way we launch all our software with a single command.

aav_yolo_models Folder:
We store all of the YOLO model files in this folder. The YOLO_ROS software will use the models in this folder for vision recognition tasks. You should take the .pt files the Vision Recognition team creates and place them in this folder. You can specify which YOLO model to use in the ROS2 launch files.

aav_worlds Folder:
This folder contains our Gazebo simulation world. All the folders contained within the aav_worlds folder are objects present in the Gazebo world. You can edit the contents of this folder to change the Gazebo world.
aav_runway.sdf is the main file you need to change in order to change the Gazebo world.


Github Codespace Docker Environment
https://github.com/AAV-VirginiaTech/aavDroneSoftware

This repo is built on top of another repo that we stole from Duke Robotics. I adapted it to fit our needs and added a ton of extra functionality.

We use a GitHub Codespace as our primary way of developing software. It is essentially a Docker environment hosted in the cloud. This gives us the benefit of not having to worry about issues between different people's personal computers.

It also lets us run our own software within VSCode. That means you can colcon build within the GitHub Codespace, and everything should build correctly. You can do this using standard ROS 2 run commands. You can also view the RQT Graph by following the steps shown in the lessons.
Dockerfile
https://github.com/AAV-VirginiaTech/aavDroneSoftware/blob/main/docker/Dockerfile

The Dockerfile creates a Docker environment that makes developing our software much easier. It ensures that everyone has access to the same dependencies and operating system (Ubuntu). GitHub Codespaces uses this Dockerfile to create the environment in the cloud. The current Docker file sets up a desktop version of Ubuntu, letting us get a GUI we can interact with (This is how we are able to access RQT_graph using noVNC).

Whenever you want to install any new frameworks or Python dependencies, you will need to update this Dockerfile. Other people will also need to rebuild their GitHub codespace after you update this file.

PostCreate.sh
https://github.com/AAV-VirginiaTech/aavDroneSoftware/blob/main/.devcontainer/postCreate.sh

This is a pretty special script that isn't common in other ROS2 setups. I created it as a workaround because GitHub Codespaces was having issues (it wasn't able to do this directly in the Dockerfile like you normally should).
 
It runs after the Docker environment has been fully built and copies the message interfaces from a bunch of separate GitHub repos. Right now, I am copying over the message interfaces for yolo_ros, Ardupilot, and MAVROS. Normally, in our simulation or on the actual drone, we install these GitHub repos separately from our AAV Software GitHub repo. This means the message interfaces aren't actually contained within the AAV GitHub repo we are developing on. In order to access these message interfaces, we have to install them separately. That is the job of the postCreate.sh file. It does a Git sparse-checkout to copy only the message interfaces and nothing else. Without this, our dev environment would complain that we are missing the ROS2 message interfaces referenced in our code.
 
Whenever you use ROS2 message interfaces from another GitHub repo, you will have to update this file. Other people will also have to rebuild their GitHub Codespaces after you do this.

