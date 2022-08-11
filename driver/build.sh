#!/bin/bash
rm -rf build
find . -regex '.*\.\(cpp\|hpp\|cc\|cxx\)' -exec clang-format -i {} \;
mkdir build
cd build
cmake ..
make

