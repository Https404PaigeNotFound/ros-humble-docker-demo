# Set Up Docker Container for ROS 2 Humble in Windows 11 from scratch

This guide walks you through setting up a Docker-based ROS 2 Humble environment on a Windows 11 machine using WSL2 and Ubuntu 22.04, with support for GUI tools like RViz and Gazebo, and development in VS Code.

---

## Prerequisites

### 1. Install WSL2 and Ubuntu 22.04

To install WSL2 and Ubuntu 22.04:
```powershell
wsl --install
```

To check if WSL and Ubuntu are installed:
```powershell
wsl --list --verbose
```
You should see a distro like `Ubuntu-22.04` and the version column should be `2` (for WSL2).

If not:
- Download Ubuntu 22.04 from the Microsoft Store.
- Set WSL2 as the default version:
  ```powershell
  wsl --set-default-version 2
  ```

---

### 2. Install Docker Desktop

- Download from: [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
- During installation:
  - Enable WSL 2 integration
  - Enable Ubuntu 22.04 integration

You also need a [Docker Hub account](https://hub.docker.com/).

---

### 3. Install Visual Studio Code and Extensions

- Download VS Code: [https://code.visualstudio.com/](https://code.visualstudio.com/)

Recommended extensions:
- [Remote - WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl)
- [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
- [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
- [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python) *(if using Python nodes)*

---

## Set Up the ROS 2 Humble Docker Container

### 1. Create a Project Folder in WSL

Open the Ubuntu 22.04 terminal:
```bash
mkdir ros2_docker_ws && cd ros2_docker_ws
```

Then open VS Code in that folder:
```bash
code .
```

### 2. Create a `Dockerfile`

Create a new file in VS Code (no extension, named exactly `Dockerfile`) and paste:
```Dockerfile
# Use the official ROS 2 Humble image
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install useful tools and packages
RUN apt update && apt install -y \
    git \
    curl \
    build-essential \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-pip \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    && apt clean

# Set default shell
SHELL ["/bin/bash", "-c"]

# Create workspace and set working directory
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Automatically source ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. Login to Docker from WSL

Use the following command:
```bash
docker login -u <your-username>
```
> Note: If `docker login` fails without the `-u` option, this workaround is required. It resolves the `401 Unauthorized` error when pulling images.

---

### 4. Build the Docker Image

Still in the WSL terminal, run:
```bash
docker build -t ros2_humble_env .
```

This may take a few minutes. Once completed, you will have a ready-to-run Docker image for ROS 2 Humble.

---

## Set Up Dev Container for VS Code

### 1. Project Structure

Organise your project like this:
```
ros2_docker_ws/
├── .devcontainer/
│   ├── devcontainer.json
│   └── Dockerfile
```

Move your Dockerfile into the `.devcontainer/` directory:
```bash
mkdir -p .devcontainer
mv Dockerfile .devcontainer/Dockerfile
```

### 2. Create `.devcontainer/devcontainer.json`

Create the following configuration to avoid common schema errors:
```json
{
  "name": "ROS 2 Humble Dev Container",
  "build": {
    "dockerfile": "Dockerfile",
    "context": ".."
  },
  "customizations": {
    "vscode": {
      "settings": {
        "terminal.integrated.shell.linux": "/bin/bash"
      },
      "extensions": [
        "ms-iot.vscode-ros",
        "ms-vscode.cpptools",
        "ms-python.python",
        "ms-azuretools.vscode-docker",
        "ms-vscode.remote-containers"
      ]
    }
  },
  "mounts": [
    "source=${localWorkspaceFolder}/ros2_ws,target=/ros2_ws,type=bind"
  ],
  "postCreateCommand": "/bin/bash -c 'source /opt/ros/humble/setup.bash'",
  "remoteUser": "root",
  "workspaceFolder": "/ros2_ws"
}
```

### 3. Open in Dev Container

1. Press `Ctrl+Shift+P` (or `F1`)
2. Type: `Dev Containers: Reopen in Container`
3. VS Code will:
   - Build the container
   - Install extensions
   - Open inside the `/ros2_ws` workspace
   - Provide a full terminal and dev environment

You should now see a terminal prompt like `root@...:/ros2_ws#`.

> **Note:** If you make changes to the `Dockerfile`, `.devcontainer/devcontainer.json`, or change system dependencies, run:
> ```
> Dev Containers: Rebuild and Reopen in Container
> ```

---

## Clone and Build ROS 2 Workspace (Leo Rover Example)

We’ll use Leo Rover’s ROS 2 packages to test the full setup.

### 1. Create the Workspace and Clone Repos

In the container terminal:
```bash
mkdir -p src
cd src

git clone -b humble https://github.com/LeoRover/leo_simulator-ros2.git
git clone https://github.com/LeoRover/leo_common-ros2.git
```

### 2. Install Dependencies

From the workspace root:
```bash
cd /ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
cd /ros2_ws
colcon build
```

### 4. Launch the Simulation

```bash
source /ros2_ws/install/setup.bash
ros2 launch leo_gz_bringup leo_gz.launch.py use_sim_time:=true
```

> 📝 **Note:** In ROS 2 Humble, the launch file is located in the `leo_gz_bringup` package.
> You may see ALSA audio errors — these are harmless unless you require audio support.

### Send Commands to Control the Rover

In a **new terminal** (while simulation is running):

```bash
source /ros2_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

You should see the rover move forward and rotate.

### Launch RViz for Visualisation

In another terminal:

```bash
source /ros2_ws/install/setup.bash
rviz2
```

In RViz2 you will encounter the error `Fixed Frame [map] does not exist`. This happens because The robot does not publish a `map` frame and there is no transform being broadcast between `map` and `odom`.
To resolve this in a static environment (i.e. when not using SLAM or localisation), you can publish a static identity transform that makes `map` and `odom` equivalent.
Open a new terminal and run:
```bash
source /ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
> When using SLAM or localisation you should not use a static transform. Instead, use a SLAM or localisation node that dynamically estimates the robot’s pose in the `map` frame and publishes the `map → odom` transform.


### Other Useful Tools

```bash
ros2 topic list
ros2 run tf2_tools view_frames
rqt_graph
```

These let you:

- Inspect active topics
- Visualise the TF transform tree
- Explore the node graph

> ⚠️ **Gazebo Issue**: If you notice Gazebo constantly flickering between "play" and "pause", try restarting the container or disabling GUI synchronisation features in Gazebo settings.

---

## 📦 Prepare Your Project for GitHub

### 🎯 Goal
You want your project to be **reproducible, shareable, and portable** across systems. Version control via GitHub allows you (and others) to:
- Collaborate and track changes
- Clone the workspace on any machine
- Automate setup using your files

### 🗂️ Project Structure After This Step

```
ros2_docker_ws/ 
├── .devcontainer/ 
│   ├── devcontainer.json 
│   └── Dockerfile 
├── ros2_ws/ 
│   ├── src/ 
│   │   └── (leo_rover clones) 
│   ├── requirements.txt       ← frozen pip packages (inside container) 
│   └── setup.sh               ← automation script for rosdep, colcon 
├── .gitignore 
├── README.md 
```

### ✅ Steps to Prepare Your Project

1. **Create a GitHub Repository**
   - Go to GitHub and create a new public or private repository (e.g., `leo-rover-docker-ws`)
   - Do **not** initialise it with a `.gitignore` — we’ll handle that locally

2. **Create a `.gitignore` File**
   - Prevents unnecessary, auto-generated, or large files from being committed (e.g., ROS `build/`, `install/`, logs, etc.)
   - You’ll find a well-configured `.gitignore` in this repo to copy
   - Note that we ignore repos that we've not edited like the `leo_common-ros2` and `leo_simulator-ros2`. We'll install them using the `setup.sh` script

3. **Freeze Python Dependencies**
   - Inside the Docker container, run:
     ```bash
     pip freeze > requirements.txt
     ```
   - This ensures all pip-installed Python packages are reproducible on other machines

4. **Write a `setup.sh` Script**
   - Automates:
     - Cloning necessary repos (like Leo Rover packages)
     - Running `rosdep install`
     - Installing pip dependencies
     - Building with `colcon`
   - Allows any developer to set up the workspace with one command
   - You can cope the `setup.sh` in this repo to copy

5. **Write Your `README.md`**
   - Provide an overview of the project, how to run it, and any other relevant documentation


6. **Commit and Push to GitHub**
   ```bash
   git init
   git remote add origin https://github.com/yourusername/leo-rover-docker-ws.git
   git add .
   git commit -m "Initial commit: ROS 2 Humble Leo Rover workspace"
   git push -u origin main
   ```

---

✅ Your project is now version-controlled, portable, and shareable. Any collaborator or future you can clone the repo and get started right away using the setup instructions.

