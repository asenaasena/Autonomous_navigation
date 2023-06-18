# Remove currently installed package (or symlink)
rm -rf ~/catkin_ws/src/mueavi_tf
# Link to catkin_ws
cd ~/catkin_ws
ln -s ~/repos/mueavi_tf ./src/
# Build package
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
