#!/bin/bash
# author: Sebastian Blumenthal
# date: 09.07.2010

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
