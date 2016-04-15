PWD_DIR=$(pwd)

yes | sudo apt-add-repository ppa:floe/libusb && sudo apt-get update -qq && sudo apt-get install -y libusb-1.0-0-dev
sudo apt-get install -y libturbojpeg libjpeg-turbo8-dev
sudo dpkg -i debs/libglfw3*deb; sudo apt-get install -f; sudo apt-get install -y libgl1-mesa-dri-lts-vivid
yes | sudo apt-add-repository ppa:pmjdebruijn/beignet-testing; sudo apt-get update -qq; sudo apt-get install -y beignet-dev;
cd ${PWD_DIR}
cd ../../../
mkdir libs
cd libs
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig
cd ${PWD_DIR}
