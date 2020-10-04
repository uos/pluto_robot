# pluto_navigation
This package uses the [mesh_navigation](https://github.com/uos/mesh_navigation) stack for navigation.
It provides a couple of launch files that start the navigation for a given set of environments and one launch file that publishes a example goals for each environment.

## How to get our datasets
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

## Environment Datasets
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

|Dataset                                                      | # Vertices | # Triangles | BB x[m] | BB y[m] | BB z[m] |
|:------------------------------------------------------------|-----------:|------------:|--------:|--------:|--------:|
| [Botanical Garden Osnabrück](#botanical-garden-osnabrück)   |   714 760  |  1 430 188  |   39.05 |   49.25 |    6.67 |
| [Stone Quarry Brockum](#stone-quarry-brockum)               |   992 879  |  1 904 178  |  100.58 |  100.58 |   23.94 |
| [Physics Campus Westerberg](#physics-campus-westerberg)     |   719 080  |  1 617 772  |  166.02 |   83.61 |   26.33 |
| [Farmer's Pit Stemwede](#farmers-pit-stemwede)              |   401 036  |    794 509  |  122.23 |  104.57 |   14.84 |
| [Market Garden Ibbenbüren All](#market-garden-ibbenbüren)   | 1 361 308  |  2 656 283  |  174.33 |  149.61 |   24.58 |
| [Market Garden Ibbenbüren Inner](#market-garden-ibbenbüren) |   725 841  |  1 224 448  |   44.23 |   21.12 |    3.33 |

### Botanical Garden Osnabrück
in progress, description coming soon.

### Stone Quarry Brockum
in progress, description coming soon.

### Physics Campus Westerberg
in progress, description coming soon.

### Farmer's Pit Stemwede
in progress, description coming soon.

### Market Garden Ibbenbüren
in progress, description coming soon.

## Example Goals
The example goals can be set by using the `navigation_goals` launch file.
You can use this launch file to publish the given goals for each environment by typing `roslaunch pluto_navigation navigation_goals.launch goal:=<goal>`.

The following goals are available:
- First example for a navigation between different elevations on the physics dataset `physics1`
- Second example for a navigation between different elevations on the physics dataset `physics2`
- Navigation on plain surface with tight corners on the physics dataset `physics3`
