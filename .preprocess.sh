PWD_DIR=$(pwd)
echo "***************************************"
echo "*****         for freenect        *****"
echo "***************************************"
echo "***** apt-get install software-properties-common python-software-properties *****"
apt-get install -y software-properties-common python-software-properties
echo "***** apt-get install build-essential cmake pkg-config *****"
apt-get install -y build-essential cmake pkg-config
echo "***** yes | yes | apt-add-repository ppa:floe/libusb && apt-get update -qq && apt-get install -y libusb-1.0-0-dev libusb-1.0-0-dev *****"
yes | apt-add-repository ppa:floe/libusb && apt-get update -qq && apt-get install -y libusb-1.0-0-dev libusb-1.0-0-dev
echo "***** apt-get install -y libturbojpeg libjpeg-turbo8-dev *****"
apt install -y libturbojpeg libjpeg-turbo8-dev
echo "***** dpkg -i debs/libglfw3*deb; apt-get install -f; apt-get install -y libgl1-mesa-dri-lts-vivid *****"
dpkg -i debs/libglfw3*deb; apt install -f; apt install -y libgl1-mesa-dri-lts-vivid
echo "***** yes | apt-add-repository ppa:pmjdebruijn/beignet-testing; apt-get update -qq; apt-get install -y beignet-dev; *****"
yes | apt-add-repository ppa:pmjdebruijn/beignet-testing; apt update -qq; apt install -y beignet-dev;
echo "***** pwd *****"
pwd
echo "***** ls -l *****"
ls -l
echo "***** cd src/libs/libfreenect2 *****"
cd ./src/libs/libfreenect2
echo "***** pwd *****"
pwd
echo "***** ls -l *****"
ls -l
echo "***** cmake . *****"
cmake . 
echo "***** make *****"
make
echo "***** make install *****"
make install

echo "***************************************"
echo "*****         for PCL 1.8         *****"
echo "***************************************"
echo "***** apt install gdebi -y *****"
apt install gdebi -y
echo "***** git clone https://github.com/MoriKen254/PCL-Linux-Release.git pcl-trunk *****"
git clone https://github.com/MoriKen254/PCL-Linux-Release.git pcl-trunk
echo "***** ls *****"
ls
echo "***** cd pcl-trunk *****"
cd pcl-trunk
echo "***** ls *****"
ls
echo "***** gdebi pcl_1.8.1-1_amd64.deb *****"
yes | gdebi pcl_1.8.1-1_amd64.deb

echo "***************************************"
echo "*****      Reinstall PCL 1.7      *****"
echo "***************************************"
apt remove -y libpcl-1.7-all libpcl-1.7-all-dev libpcl-1.7-bin;libpcl-1.7-doc
apt remove -y libpcl-apps-1.7 libpcl-apps-1.7-dev
apt remove -y libpcl-common-1.7 libpcl-common-1.7-dev
apt remove -y libpcl-features-1.7 libpcl-features-1.7-dev
apt remove -y libpcl-filters-1.7 libpcl-filters-1.7-dev
apt remove -y libpcl-geometry-1.7-dev libpcl-geometry-1.7-dev
apt remove -y libpcl-io-1.7 libpcl-io-1.7-dev
apt remove -y libpcl-kdtree-1.7 libpcl-kdtree-1.7-dev
apt remove -y libpcl-keypoints-1.7 libpcl-keypoints-1.7-dev
apt remove -y libpcl-octree-1.7 libpcl-octree-1.7-dev
apt remove -y libpcl-outofcore-1.7 libpcl-outofcore-1.7-dev
apt remove -y libpcl-people-1.7 libpcl-people-1.7-dev
apt remove -y libpcl-recognition-1.7 libpcl-recognition-1.7-dev
apt remove -y libpcl-registration-1.7 libpcl-registration-1.7-dev
apt remove -y libpcl-sample-consensus-1.7 libpcl-sample-consensus-1.7-dev
apt remove -y libpcl-search-1.7 libpcl-search-1.7-dev
apt remove -y libpcl-segmentation-1.7 libpcl-segmentation-1.7-dev
apt remove -y libpcl-surface-1.7 libpcl-surface-1.7-dev
apt remove -y libpcl-tracking-1.7 libpcl-tracking-1.7-dev
apt remove -y libpcl-visualization-1.7 libpcl-visualization-1.7-dev
apt install -y ros-indigo-jsk-interactive ros-indigo-jsk-interactive-marker
apt install -y ros-indigo-jsk-interactive-test ros-indigo-jsk-recognition-utils
apt install -y ros-indigo-jsk-rqt-plugins ros-indigo-jsk-rviz-plugins
apt install -y ros-indigo-jsk-visualization ros-indigo-pcl-conversions ros-indigo-pcl-ros

echo "***************************************"
echo "*****       for AR Toolkit        *****"
echo "***************************************"
echo "***** apt install libv4l-dev *****"
apt install -y libv4l-dev
echo "***** cd /usr/include/linux *****"
cd /usr/include/linux
echo "***** ln -s ../libv4l1-videodev.h videodev.h *****"
ln -s ../libv4l1-videodev.h videodev.h

echo "***************************************"
echo "*****       Apply Lib Pkgs        *****"
echo "***************************************"
echo "***** ldconfig *****"
ldconfig
echo "***** ldconfig -p *****"
ldconfig -p
cd ${PWD_DIR}
