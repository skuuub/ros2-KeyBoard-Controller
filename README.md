# Turtle Key Controller

<!-- Add a screenshot or GIF of the node in action -->
![Keyboard Controller Demo](https://github.com/skuuub/ros2-KeyBoard-Gesture-Controller/blob/e8ba4ed3451921a11683675686c52c1bbd89e7c7/Screenshot%202025-06-06%20at%2018.35.05.png)



A simple ROS 2 Python node that allows controlling the turtlesim turtle using keyboard input (W/A/S/D).

## Features

- Move forward/backward
- Turn left/right
- Exit on Ctrl+C

## Prerequisites

- ROS 2 (e.g., Humble Hawksbill) installed and sourced
- `turtlesim` package
- Python 3 (with standard `termios` and `tty` modules)

## Installation

1. **Source your ROS 2 installation**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Clone this repository** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/<your-username>/<your-repo>.git
   ```

3. **Install dependencies and build the workspace**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   ```

4. **Source your workspace**:
   ```bash
   source install/setup.bash
   ```

## Usage

1. **Run turtlesim** in one terminal:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

2. **Run the key controller** in a new terminal (make sure to source your workspace first):
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run <your_package_name> key_board_controller
   ```

3. **Control the turtle** using the following keys:
   - `W`: move forward
   - `S`: move backward
   - `A`: turn left
   - `D`: turn right
   - `Ctrl+C`: exit the node

## Node Details

- **Node Name**: `turtle_key_controller`
- **Publish Topic**: `/turtle1/cmd_vel` (type: `geometry_msgs/msg/Twist`)

## File Structure

```
<your-repo>/
├── package.xml
├── setup.py
├── resource/
│   └── <package_name>
├── turtle_key_controller/
│   └── turtle_key_controller.py
└── README.md
```

## License

This project is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE) for details.

