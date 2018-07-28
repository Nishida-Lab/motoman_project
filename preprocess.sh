PWD_DIR=$(pwd)
yes | apt install apt-file; apt-file update; apt-file search add-apt-repository;
sudo apt-get install software-properties-common
yes | apt-add-repository ppa:floe/libusb && apt update -qq && apt install -y libusb-1.0-0-dev
apt install -y libturbojpeg libjpeg-turbo8-dev
dpkg -i debs/libglfw3*deb; apt install -f; apt install -y libgl1-mesa-dri-lts-vivid
yes | apt-add-repository ppa:pmjdebruijn/beignet-testing; apt update -qq; apt install -y beignet-dev;
cd ${PWD_DIR}
cd ../../../
mkdir libs
cd libs
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build && cd build
cmake ..
make
make install
ldconfig
cd ${PWD_DIR}
