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
- TODO

### Simulation Package Usage
- Terminal 1: `ros2 launch basic_sim arena_world.launch.py`
    - Launches gazebo with the basic arena world(Skid-steer model rover + arena walls + flat ground)
- Terminal 2: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/rover/cmd_vel`
    - Control the rover model with your keyboard
    - Start at a low speed to prevent rover from jumping all over the place

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
            joystick_sub/
```
