#!/bin/bash
# author: Sebastian Blumenthal
# date: 24.09.2009

echo "Starting to install BRICS_3D library."
if [ ! -d ./build ]
then 
   mkdir ./build
fi

echo ""
echo "Compilation of 6DSLAM precondition:"
cd ./external/6dslam/build/
cmake ..
make
cd ../../../

echo ""
echo "Compilation of BRICS_3D library:"
cd build #compile as out-of-source build
cmake ..
make
cd ..

echo "Done."
