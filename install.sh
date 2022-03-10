# Generate python binding.
cd cpp/core/src
swig -c++ -python py_rigid_body_simulation_core.i

# Compile c++ code.
cd ../../
mkdir -p build
cd build
cmake -DPARDISO_AVAILABLE=OFF ..
make -j4
./rigid_body_simulation_demo

# Python binding.
cd ../core/src/
mv py_rigid_body_simulation_core.py ../../../python/py_rigid_body_simulation/core
mv ../../build/libpy_rigid_body_simulation_core.so ../../../python/py_rigid_body_simulation/core/_py_rigid_body_simulation_core.so

# Log absolute path.
cd ../../../
root_path=$(pwd)
printf "root_path = '%s'\n" "$root_path" > python/py_rigid_body_simulation/common/project_path.py