#!/bin/bash
echo "Formating code..."
find ./core -regex '.*\.\(cpp\|hpp\|cc\|cxx\)' -exec clang-format -i {} \;
clang-format -i main.cpp

mkdir -p build
cd build
cmake ..
echo "Building..."
make

cp driver ../bin/