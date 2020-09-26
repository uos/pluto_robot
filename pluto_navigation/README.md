# pluto_navigation
This package uses the [mesh_navigation](https://github.com/uos/mesh_navigation) stack for navigation.
It provides a couple of launchfiles that start the navigation for a given set of environments and one launchfile that publishes a example goals for each environment.

## Environment Datasets
The following environment dataset are available:
- Botanical Garden at Osnabrück University:  
`roslaunch pluto_navigation botanical_garden_osnabrueck.launch`
- Stone Quarry in the forest in Brockum:  
`roslaunch pluto_navigation stone_quarry_brockum.launch`
- Physics building at Campus Westerberg, Osnabrück University:  
`roslaunch pluto_navigation physics_campus_westerberg.launch`

## Example Goals
The example goals can be set by using the `navigation_goals` launchfile.
You can use this launchfile to publish the given goals for each environment by typing `roslaunch pluto_navigation navigation_goals.launch goal:=<goal>`.


The following goals are available:
- First example for a navigation between different elevations on the physics dataset `physics1`
- Second example for a navigation between different elevations on the physics dataset `physics2`
- Navigation on plain surface with tight corners on the physics dataset `physics3`
