rm build -rf
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_MARCH_NATIVE=ON -DBUILD_OPENMP=ON  -DEIGEN_USE_LAPACKE=ON  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
sudo cmake --build build --target install -- -j8
