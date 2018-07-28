./install_libfreenect2.sh

PWD_DIR=$(pwd)
cd ../../../
wstool init src src/motoman_project/dependencies.rosinstall
rosdep update
rosdep install -i -r -y --from-paths src --ignore-src

catkin_make
source devel/setup.bash

catkin_make run_tests
catkin_test_results

