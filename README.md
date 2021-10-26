# IRIS-2022

### Getting Started
1. Create a colcon workspace
    - if you're colcon workspace folder does not exist: `mkdir -p ~/colcon_ws/src`
    - Make sure the `~/colcon_ws/src` subfolder exists
2. Clone this Repo
    -  Clone this repo into the `~/colcon_ws/src` subfolder
2. Install the required dependencies with rosdep
    - `cd ~/colcon_ws`
    - `rosdep install -i --from-path src --rosdistro galactic -y`
3. Build the desired package
    - `colcon build --packages-select <Package folder name>`
4. Source the setup script so ros2 can find the packages in this workspace 
    - `source ~/colcon_ws/install/setup.bash`

### Teleop Package Usage
- `ros2 run teleop teleop_node`
    - Launches the teleop node *ONLY*
- `ros2 launch teleop teleop_launch.py`
    - Launches everything, including joystick nodes and teleop node

### Simulation Package Usage
- `ros2 launch basic_sim arena_world.launch.py`
    - Launches gazebo with the arena world

### End Package Folder Structure
```
~/colcon_ws/
    build/
    install/
    log/
    src/
        IRIS-2022/
            basic_sim/
            py_pubsub/
            teleop/
```
