# Pluto
![pluto_robot](docs/images/pluto.jpg?raw=true "Pluto")

## Maintainers
* [Sebastian Pütz](mailto:spuetz@uos.de)

## Installation from Repository 
If you want to install the robot with its configuration and launch files from the repository just follow the next steps.
Otherwise use `sudo apt install ros-medlod-pluto-robot`

### 1. Install wstool
```
sudo apt install python-wstool
mkdir -p ~/pluto_ws/src
```

### 2. Use wstool to pull all required packages
```
cd ~/pluto_ws
wstool init src https://raw.githubusercontent.com/uos/uos_rosinstalls/master/pluto.rosinstall
wstool update -t src
```

### 3. Use rosdep to install all
```
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build and source the workspace
```
catkin_make
source devel/setup.bash
```

## Pluto in Simulation
You can use Pluto in an outdoor simulation environment. We provides several datasets and the corresponding environments
for the Gazebo simulation. For navigation purposes the corresponding navigation launch file should be started, too. 
The following simulation environments are currently available:
- Botanical Garden at Osnabrück University: `roslaunch pluto_gazebo pluto_botanical_garden.launch`
- Stone Quarry in the Forest in Brockum: `roslaunch pluto_gazebo pluto_stone_quarry.launch`
- Physics building at Osnabrück University: `roslaunch pluto_gazebo pluto_physics.launch`

## Pluto in Real Life
After the `roscore` has been started a provided launchfile starts the general functionalities of the robot, e.g. driving, 
Velodyne point scanning, reading IMU, etc:
`roslaunch pluto_bringup pluto.launch`

## Pluto Mesh Navigation
The [mesh_navigation](https://github.com/uos/mesh_navigation) stack provides a navigation server for 
[Move Base Flex](https://github.com/magazino/move_base_flex). It provides a couple of configuration files and launch 
files to start the navigation server with the configured layer plugins for the layered mesh map, and the configured
planners and controller to perform path planning and motion control in 3D (or more specifically on 2D-manifold). 

See the Demo Video:
[![Mesh Navigation with Pluto](http://img.youtube.com/vi/qAUWTiqdBM4/0.jpg)](http://www.youtube.com/watch?v=qAUWTiqdBM4)

For more information take a look at [pluto_navigation](pluto_navigation).
