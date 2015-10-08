PLUTO_INSTALL_DIR=$(readlink -f $(dirname $0)) 
echo installing from echo $PLUTO_INSTALL_DIR/pluto.rosinstall
if [ -z "$1" ]; then
  PLUTO_WORKSPACE_INSTALL_DIR=".";
else
  PLUTO_WORKSPACE_INSTALL_DIR=$1
fi

PLUTO_WORKSPACE_INSTALL_DIR=$(readlink -f $PLUTO_WORKSPACE_INSTALL_DIR)
echo install the workspace to $PLUTO_WORKSPACE_INSTALL_DIR
### pluto catkin workspace ### 

mkdir -p $PLUTO_WORKSPACE_INSTALL_DIR/pluto
PLUTO_WORKSPACE=$PLUTO_WORKSPACE_INSTALL_DIR/pluto
cd $PLUTO_WORKSPACE
cp $PLUTO_INSTALL_DIR/pluto.rc $PLUTO_WORKSPACE
echo export PLUTO_WORKSPACE=$PLUTO_WORKSPACE >> ~/.bashrc
echo '. ${PLUTO_WORKSPACE}/pluto.rc' >> ~/.bashrc
. /opt/ros/indigo/setup.sh
. ~/.bashrc
wstool init src $PLUTO_INSTALL_DIR/pluto.rosinstall
#catkin_init_workspace src 
catkin_make
#pluto
