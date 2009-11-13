#!/bin/bash
# author: Sebastian Blumenthal
# date: 24.09.2009

echo "Starting to install BRICS_3D library."


echo ""
echo "Compilation of 6DSLAM precondition:"
if [ ! -d ./external/6dslam/build ]
then 
   mkdir ./external/6dslam/build 
fi
cd ./external/6dslam/build/
cmake ..
make
cd ../../../


echo ""
echo "Compilation of BRICS_3D library:"
if [ ! -d ./build ]
then 
   mkdir ./build
fi

cd build #compile as out-of-source build
cmake .. -DCGAL_DIR=/usr/local/lib/CGAL/
make
cd ..

echo "Done."
