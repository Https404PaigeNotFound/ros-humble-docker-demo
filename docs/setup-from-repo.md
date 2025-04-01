# Set Up Docker Container for ROS 2 Humble in Windows 11 from this repo

# Using This Repository

## 🧪 Requirements

- Docker Desktop (WSL2 backend)
- VS Code with Dev Containers extension

## 🚀 Steps

### Clone this repo:
```bash
git clone https://github.com/yourusername/ros-humble-docker-demo.git
cd ros-humble-docker-demo
```



# Set Up Docker Container for ROS 2 Humble in Windows 11 from this repo

---

## 🧪 Requirements

You will need:

### ✅ Windows 10/11 with WSL2 and Ubuntu 22.04
- Make sure WSL 2 is installed and Ubuntu 22.04 is set as the default distro.

### ✅ Docker Desktop
- Download from: [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
- During installation:
  - Enable WSL 2 integration
  - Enable Ubuntu 22.04 integration
- You will also need a [Docker Hub account](https://hub.docker.com/)

### ✅ Visual Studio Code + Extensions
- Download VS Code: [https://code.visualstudio.com/](https://code.visualstudio.com/)

#### Recommended extensions:
- [Remote - WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl)
- [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
- [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
- [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)

---

## 🚀 Steps to Run This Project
In the WSL terminal, change directory to where you'd like to clone this repo. 

### 1. Clone this repository:
```bash
git clone https://github.com/yourusername/ros-humble-docker-demo.git
cd ros-humble-docker-demo
```

### 2. Open the project in VS Code
```bash
code .
```

### 3. Reopen in Dev Container
- Press `Ctrl+Shift+P`
- Select: `Dev Containers: Reopen in Container`
- VS Code will build and open the Docker container using the `.devcontainer/` setup

### 4. Run the setup script inside the container
```bash
cd ros2_ws
./setup.sh
source install/setup.bash
```

### 5. Launch the Leo Rover simulation
```bash
ros2 launch leo_gz_bringup leo_gz.launch.py
```

---

## 🛠️ Optional Extras

### 📡 Control the robot
In a new terminal:
```bash
source ros2_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

### 👁️ Launch RViz2
```bash
rviz2
```
- Set the fixed frame to `odom` to remove errors.

### 🧭 Static transform workaround
If you see `Fixed Frame [map] does not exist`, run:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### 🔍 Explore the ROS graph and TF tree
```bash
ros2 topic list
ros2 run tf2_tools view_frames
rqt_graph
```

---

This project is now reproducible, sharable, and ready for development or demonstration on any machine with Docker and VS Code.

