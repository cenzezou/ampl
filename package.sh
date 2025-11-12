#!/bin/bash
mkdir -p package
./build.sh
cd build
sudo cpack -G DEB
cd ..
cp ./build/_CPack_Packages/Linux/DEB/*.deb package/
cd python
source /home/czhou/miniforge3/bin/activate py311
./build.sh
python3 -m build

conda deactivate
cd ..
cp  ./python/dist/*.whl package/

