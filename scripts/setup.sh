sudo apt install libmatio-dev
cd src/localization/matio-cpp
mkdir build && cd build
cmake ..
make
sudo make install