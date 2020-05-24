#! /bin/bash
# Note: Use this script as a regular user. It will request sudo on demand

## Install yocs stuff
P=ros-melodic-yocs-
sudo aptitude install ${P}ar-marker-tracking ${P}ar-pair-approach ${P}ar-pair-tracking ${P}cmd-vel-mux ${P}controllers \
${P}joyop ${P}keyop ${P}localization-manager ${P}math-toolkit ${P}msgs ${P}navi-toolkit ${P}navigator ${P}rapps ${P}safety-controller ${P}velocity-smoother ${P}virtual-sensor ${P}waypoint-provider ${P}waypoints-navi -y

# Install kobuki stuff
sudo aptitude install ros-melodic-kobuki-core ros-melodic-kobuki-dock-drive ros-melodic-kobuki-driver ros-melodic-kobuki-ftdi ros-melodic-kobuki-msgs -y

# Install turtlebot3 packages
sudo aptitude install ros-melodic-turtlebot3-gazebo -y

# Additional utils
sudo aptitude install pyqt5-dev-tools ros-melodic-ecl-core -y

# Install the rosinstall stuff in the
roscd
cd ../src
rosinstall . robotics_challenge/robotics_challenge.rosinstall

# Compile the stuff
cd ..
catkin_make
