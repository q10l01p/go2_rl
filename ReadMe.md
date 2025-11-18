# 编译
```
sudo apt install -y libmpfr-dev libgmp-dev

sudo apt-get install ros-$ROS_DISTRO-grid-map

rosdep update
rosdep install --from-paths src --ignore-src -r -y

catkin build pybind11_catkin cmake_clang_tools
catkin build elevation_mapping_cupy





```