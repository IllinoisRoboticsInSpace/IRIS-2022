# IRIS-2022

### Getting Started
1. Create a colcon workspace and `cd ~/colcon_ws`
2. Install the required dependencies with `rosdep install -i --from-path src --rosdistro galactic -y`
3. Build just this package with `colcon build --packages-select basic_sim`
4. Source the setup script so ros2 can find the packages in this workspace `source ~/colcon_ws/install/setup.bash`

### Usage
- `ros2 launch basic_sim arena_world.launch.py`
    - Launches gazebo with the arena world

