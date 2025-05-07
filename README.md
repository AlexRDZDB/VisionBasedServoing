# VisionBasedServoing

This project incorporates CoppeliaSIM and ROS2 Humble Hawksbill to create a simulation of a Pioneer3DX mobile robot following a green sphere through visual feedback.

[![Watch the demo](https://img.youtube.com/vi/Si36V4LxL7g/0.jpg)](https://youtu.be/Si36V4LxL7g)

## Project Structure
```plaintext
vision_based_servoing/
├── src/image_processor/         # Package for processing camera input
├── src/robot_controller/        # Package for PID-based control
├── CoppeliaSimObjectScripts/    # Object Scripts for inside CoppeliaSIM simulation
├── CoppeliaSimScenes/           # Prebuilt scenes for demonstrations
├── requirements.txt             # Python dependencies
└── README.md
```

## Requirements

To run this project successfully, make sure you have the following software installed:

- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2 Distribution**: Humble Hawksbill  
  - Follow the official ROS 2 installation guide for [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- **CoppeliaSim**: Educational Version `V4_0_9`
  - Download from the [CoppeliaSim website](https://www.coppeliarobotics.com/)
- **Python**: 3.10+ (comes by default with Ubuntu 22.04)
- **Additional Python Packages**:
  - `opencv-python`
  - `numpy`
  - `cv_bridge`
  - `sensor_msgs`, `geometry_msgs`, `std_msgs` (ROS interfaces)

To install the required Python packages, run:

```bash
pip install -r requirements.txt
```

> **Note**: Ensure that `cv_bridge` is properly sourced with your ROS 2 environment, and that CoppeliaSim is launched with ROS 2 bridge support (e.g., via `simROS2` plugin).

## Building the Workspace

Clone the repository inside your ROS 2 workspace `src/` directory:

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/vision_based_servoing.git
cd ..
colcon build
source install/setup.bash
```

## Running the simulation
In order to run the simulation, make sure you launch CoppeliaSIM with ROS2 bridge support and have your ROS environment sourced in the terminal. Open the *VisualServo.ttt* file provided in the repository in CoppeliaSIM and press the Play Button to start the simulation.

To start robot control, run the following command within the terminal where you have both your ROS2 installation and the corresponding packages in this repo sourced.

```bash
ros2 launch robot_controller visualservo.launch.py
```

## License

MIT License (or your preferred one)

## Acknowledgements

- Coppelia Robotics
- OpenCV
- ROS 2 Community
