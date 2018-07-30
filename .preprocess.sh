PWD_DIR=$(pwd)
echo "***** apt install build-essential cmake pkg-config *****"
apt install -y build-essential cmake pkg-config
# echo "***** apt install -y apt-file *****"
# apt install -y apt-file
# echo "***** apt-file update -y *****"
# apt-file update -y
# echo "***** apt-file search add-apt-repository -y *****"
# apt-file search add-apt-repository -y
# echo "***** apt-get install software-properties-common -y *****"
# apt-get install software-properties-common -y
# echo "***** apt-add-repository ppa:floe/libusb -y *****"
# apt-add-repository ppa:floe/libusb -y
# echo "***** apt update -qq *****"
# apt update -qq
echo "***** apt install -y libusb-1.0-0-dev *****"
apt install -y libusb-1.0-0-dev
echo "***** apt install -y libturbojpeg libjpeg-turbo8-dev *****"
apt install -y libturbojpeg libjpeg-turbo8-dev
#echo "**** dpkg -i debs/libglfw3*deb; apt install -f; apt install -y libgl1-mesa-dri-lts-vivid *****"
#dpkg -i debs/libglfw3*deb; apt install -f; apt install -y libgl1-mesa-dri-lts-vivid
echo "***** apt-get install libglfw3-dev *****"
apt install -y libglfw3-dev
#echo "***** apt-add-repository ppa:pmjdebruijn/beignet-testing -y *****"
#apt-add-repository ppa:pmjdebruijn/beignet-testing -y
#echo "***** apt update -qq *****"
#apt update -qq
#echo "***** apt install -y beignet-dev *****"
#apt install -y beignet-dev
echo "***** pwd *****"
pwd
echo "***** ls -l *****"
ls -l
echo "***** cd /builds/MoriKen254/motoman_project/catkin_workspace/src/libs/libfreenect2 *****"
cd /builds/MoriKen254/motoman_project/catkin_workspace/src/libs/libfreenect2
cd 
echo "***** pwd *****"
pwd
echo "***** ls -l *****"
ls -l
echo "***** cmake . *****"
cmake . 
#cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/src/freenect2
#cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/src/freenect2
#cmake .. -Dfreenect2_DIR=$HOME/src/freenect2/lib/cmake/freenect2
#cmake .. -Dfreenect2_DIR=$HOME/src/freenect2/lib/cmake/freenect2
echo "***** make *****"
make
echo "***** make install *****"
make install
 
echo "***************************************"
echo "*****       for AR Toolkit        *****"
echo "***************************************"
echo "***** apt install libv4l-dev *****"
apt install -y libv4l-dev
echo "***** cd /usr/include/linux *****"
cd /usr/include/linux
echo "***** ln -s ../libv4l1-videodev.h videodev.h *****"
ln -s ../libv4l1-videodev.h videodev.h
echo "***** ldconfig *****"
ldconfig
echo "***** ldconfig -p *****"
ldconfig -p
cd ${PWD_DIR}
