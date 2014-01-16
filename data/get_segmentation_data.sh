#!/bin/bash

# Ecexute this scrip if you can to execute the demoComarision or demoEvaluation examples.
echo "Retrieving data for comparision of segmentation algorithms"

wget http://www.best-of-robotics.org/brics_3d/downloads/segmentation_data.tar.gz
tar -xvf segmentation_data.tar.gz

echo "Done."
