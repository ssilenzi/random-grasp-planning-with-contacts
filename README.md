# Random grasp planning with contacts
## Edge detection package
### Download and build
```
cd workspace
sudo apt-get update
sudo apt-get install python-catkin-tools
source /opt/ros/kinetic/setup.bash
rosdep update
git clone -b edge_detection https://github.com/CentroEPiaggio/random-grasp-planning-with-contacts.git src/
export ROS_PACKAGE_PATH=$PWD/src/edge_detection:$ROS_PACKAGE_PATH
rosdep install edge_detection
catkin build
source devel/setup.bash
```
### Change the run configuration
```
gedit src/edge_detection/config/edged.yaml &
```
### Launch the package
```
roslaunch edge_detection edged.launch
```
### Launch the package and display the output topic
```
roslaunch edge_detection edged.launch echo:=true
```
The output topic will be displayed in a separate terminal
### Stop the package
Press any key in any window or Ctrl-C