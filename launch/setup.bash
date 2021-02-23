
sudo apt install -y libeigen3-dev libassimp-dev libccd-dev
cd ~
git clone https://github.com/flexible-collision-library/fcl.git
mkdir fcl/build
cd fcl/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
