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

### Experiments

We provide experiments consisting of starting positions and goals to test the navigation. To run an experiment you first need to launch the navigation stack with the chosen [Environment Dataset](#environment-datasets) and then publish the experiments poses:

```bash
roslaunch pluto_navigation experiment.launch experiment:={experiment}
```

Replace `{experiment}` with the experiment you choose.
Available experiments for each dataset can be found in the [Environment Dataset Section](#environment-datasets) in the description of the correspoiding dataset.

### How to get our datasets

To manage our datasets we use a git extension called "Git Large File Storage" or for short git LFS.
This allows us to keep this repository small while also embedding our datasets inside this repository.

To download our datasets you first need to install [git LFS](https://git-lfs.github.com) on your system.
After installing the git extension you will be able to clone this repository like any other git repository and the datasets will be downloaded.

If you already cloned this repository without having git LFS installed, the dataset files will just be placeholders on your local copy.
To download the datasets you again need to install git LFS on your system.
After installing the Extension you need to install the git LFS configuration locally.
This needs to be ran just once per system and repository inside the directory of your local copy of this repository.

```
git lfs install
```

Now simply fetch all available git LFS resources for your local copy of this repository.
This will download all datasets and make them available locally.

```
git lfs fetch --all
```

### Environment Datasets

The following environment dataset are available:

- Botanical Garden at Osnabrück University:  
  `roslaunch pluto_navigation botanical_garden_osnabrueck.launch`
- Stone Quarry in the forest in Brockum:  
  `roslaunch pluto_navigation stone_quarry_brockum.launch`
- Physics building at Campus Westerberg, Osnabrück University:  
  `roslaunch pluto_navigation physics_campus_westerberg.launch`
- Farmer's Pit in Stemwede:  
  `roslaunch pluto_navigation farmers_pit_stemwede.launch` 
- Market Garden (complete) in Ibbenbüren:  
  `roslaunch pluto_navigation market_garden_ibbenbueren_all.launch`
- Market Garden (beds) in Ibbenbüren:  
  `roslaunch pluto_navigation market_garden_ibbenbueren_inner.launch`

| Dataset                                                   | # Vertices | # Triangles | BB x[m] | BB y[m] | BB z[m] | Overview                                                                                                                                                                            |
|:--------------------------------------------------------- | ----------:| -----------:| -------:| -------:| -------:| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Botanical Garden Osnabrück](#botanical-garden-osnabrück) | 714 760    | 1 430 188   | 39.05   | 49.25   | 6.67    | ![BotanicalGardenOsnabrueckOverview](pluto_navigation/maps/botanical_garden_osnabrueck/botanical_garden_osnabrueck_sideview.png?raw=true "Botanical Garden Osnabrueck Overview")    |
| [Stone Quarry Brockum](#stone-quarry-brockum)             | 992 879    | 1 904 178   | 100.58  | 100.58  | 23.94   | ![StoneQuarryBrockumOverview](pluto_navigation/maps/stone_quarry_brockum/stone_quarry_brockum_sideview.png?raw=true "Stone Quarry Brockum Overview")                                |
| [Physics Campus Westerberg](#physics-campus-westerberg)   | 719 080    | 1 617 772   | 166.02  | 83.61   | 26.33   | ![PhysicsCampusWesterbergOverview](pluto_navigation/maps/physics_campus_westerberg/physics_campus_westerberg_sideview.png?raw=true "Physics Campus Westerberg Overview")            |
| [Farmer's Pit Stemwede](#farmers-pit-stemwede)            | 401 036    | 794 509     | 122.23  | 104.57  | 14.84   | ![FarmersPitStemwedeOverview](pluto_navigation/maps/farmers_pit_stemwede/farmers_pit_stemwede_sideview.png?raw=true "Farmers Pit Stemwede Overview")                                |
| [Market Garden Ibbenbüren All](#market-garden-ibbenbüren) | 1 361 308  | 2 656 283   | 174.33  | 149.61  | 24.58   | ![MarketGardenIbbenbuerenAllOverview](pluto_navigation/maps/market_garden_ibbenbueren/market_garden_ibbenbueren_all_sideview.png?raw=true "Market Garden Ibbenbueren All Overview") |

#### Botanical Garden Osnabrück

in progress, description coming soon.

#### Stone Quarry Brockum

in progress, description coming soon.

#### Physics Campus Westerberg

in progress, description coming soon.

#### Farmer's Pit Stemwede

in progress, description coming soon.

#### Market Garden Ibbenbüren

in progress, description coming soon.
