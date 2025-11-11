# AMPL = ANOTHER MOTION PLANNING LIBRARY

## Installation 

### C++
Compile and generate `deb` package  

```bash
chmod +x ./rebuild.sh
./rebuild.sh
cd build
sudo cpack -G DEB
```
See `build/_CPack_Packages/Linux/DEB/` for `.deb` file

### Python

Create a conda environment to build a specific version of python wheel.
```bash
conda -n create py3XX python=3.XX
conda activate py3XX
conda install scikit-build-core
conda install nanobind
pip3 install build
```
Build nanobind of AMPL in conda environment `py3XX`
```bash
cd python
chmod +x ./build.sh
./build.sh
python3 -m build
```
See `python/dist/` for `.whl` file


