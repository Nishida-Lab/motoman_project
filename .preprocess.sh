PWD_DIR=$(pwd)
echo "***** apt install build-essential cmake pkg-config *****"
apt install -y build-essential cmake pkg-config
echo "***** apt install -y libusb-1.0-0-dev *****"
apt install -y libusb-1.0-0-dev
echo "***** apt install -y libturbojpeg libjpeg-turbo8-dev *****"
apt install -y libturbojpeg libjpeg-turbo8-dev
echo "***** apt-get install libglfw3-dev *****"
apt install -y libglfw3-dev
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
echo "*****       for AR Toolkit        *****"
echo "***************************************"
echo "***** apt install libv4l-dev *****"
apt install -y libv4l-dev
echo "***** cd /usr/include/linux *****"
cd /usr/include/linux
echo "***** ln -s ../libv4l1-videodev.h videodev.h *****"
ln -s ../libv4l1-videodev.h videodev.h
echo "***** wget https://github.com/MoriKen254/ARToolKit_Release/raw/master/ARToolKit_2.72.1_Ubuntu_amd64.deb *****"
wget https://github.com/MoriKen254/ARToolKit_Release/raw/master/ARToolKit_2.72.1_Ubuntu_amd64.deb
yes|dpkg -i ARToolKit_2.72.1_Ubuntu_amd64.deb

echo "***************************************"
echo "*****       Apply Lib Pkgs        *****"
echo "***************************************"
echo "***** ldconfig *****"
ldconfig
echo "***** ldconfig -p *****"
ldconfig -p
cd ${PWD_DIR}
