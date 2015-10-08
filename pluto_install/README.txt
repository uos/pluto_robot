### Installation Instructions
### Just use the shell script install.sh 
### with the destination folder name as parameter
### or follow the next lines

### In the pluto_install folder type:

  PLUTO_INSTALL_DIR=$(pwd) 

### create your pluto workspace anywhere:
  mkdir pluto
  cd pluto
  PLUTO_WORKSPACE_DIR=$(pwd)

### to make the short commands available type:
  cp $PLUTO_INSTALL_DIR/pluto.rc $PLUTO_WORKSPACE
  echo PLUTO_WORKSAPCE=$PLUTO_WORKSPACE >> $PLUTO_WORKSPACE/pluto.rc
  echo . $PLUTO_WORKSPACE/pluto.rc >> ~/.bashrc
  source ~/.bashrc

### install ros packages and compile:
  source /opt/ros/indigo/setup.bash
  wstool init src $PLUTO_INSTALL_DIR/pluto.rosinstall
  catkin_init_workspace src 
  catkin_make
### now all short commands are available, try:
  pluto
