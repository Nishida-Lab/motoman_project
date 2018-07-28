PWD_DIR=$(pwd)
echo "***** apt install -y apt-file *****"
apt install -y apt-file
echo "***** apt-file update -y *****"
apt-file update -y
echo "***** apt-file search add-apt-repository -y *****"
apt-file search add-apt-repository -y
echo "***** apt-get install software-properties-common -y *****"
apt-get install software-properties-common -y
echo "***** apt-add-repository ppa:floe/libusb -y *****"
apt-add-repository ppa:floe/libusb -y
echo "***** apt update -qq *****"
apt update -qq
echo "***** apt install -y libusb-1.0-0-dev *****"
apt install -y libusb-1.0-0-dev
echo "***** apt install -y libturbojpeg libjpeg-turbo8-dev *****"
apt install -y libturbojpeg libjpeg-turbo8-dev
echo "****dpkg -i debs/libglfw3*deb; apt install -f; apt install -y libgl1-mesa-dri-lts-vivid *****"
dpkg -i debs/libglfw3*deb; apt install -f; apt install -y libgl1-mesa-dri-lts-vivid
echo "***** apt-add-repository ppa:pmjdebruijn/beignet-testing -y *****"
apt-add-repository ppa:pmjdebruijn/beignet-testing -y
echo "***** apt update -qq *****"
apt update -qq
echo "***** apt install -y beignet-dev *****"
apt install -y beignet-dev
echo "***** cd root/catkin_ws/src/libs/libfreenect2 *****"
cd root/catkin_ws/src/libs/libfreenect2
echo "***** pwd *****"
pwd
echo "***** ls -l *****"
ls -l
#echo "***** mkdir build *****"
#mkdir build
#echo "***** cd build *****"
#cd build
echo "***** cmake *****"
cmake 
echo "***** make *****"
make
echo "***** make install *****"
make install
echo "***** ldconfig *****"
ldconfig
cd ${PWD_DIR}
