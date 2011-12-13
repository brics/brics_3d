/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#include "algorithm/segmentation/RegionBasedSACSegmentation.h"
#include "core/PointCloud3D.h"
#include "core/Point3D.h"
#include <iostream>
using namespace std;
int main(){
	//Create a pointcloud object
	BRICS_3D::PointCloud3D cloud;

	//read the points into the pointcloud
	//Please modify the path if there is a file read error.
	cloud.readFromTxtFile("./src/algorithm/segmentation/evaluation/data/demoCloud.txt");

	if (cloud.getSize()>0){
		cout<< "INFO: Current PointCloud Size: " <<cloud.getSize()<<endl;
	} else {
		cout<< "INFO: Current PointCloud Size: " <<cloud.getSize()<<endl;
		return 0;
	}


	//Create the vector to hold the model coefficients
	Eigen::VectorXd modelCoefficients;

	//Create the vector to hold the indexes of the model inliers
	std::vector<int> inliers;

	//Create the SACSegmentation Object
	BRICS_3D::RegionBasedSACSegmentation sacSegmenter;

	//Initialize the segmenter
	sacSegmenter.setPointCloud(&cloud);
	sacSegmenter.setDistanceThreshold(0.01);
	sacSegmenter.setMaxIterations(1000);
	sacSegmenter.setMethodType(sacSegmenter.SAC_RANSAC);
	sacSegmenter.setModelType(sacSegmenter.OBJMODEL_PLANE_FROM_LINE_AND_POINT);
	sacSegmenter.setProbability(0.99);

	//Perform the segmentation
	sacSegmenter.segment();
	sacSegmenter.getInliers(inliers);
	sacSegmenter.getModelCoefficients(modelCoefficients);

	if (inliers.size() == 0)
	{
		cout<<"Could not estimate a planar model for the given dataset."<<endl;
		return (-1);
	}else {
		cout<<"Found Inliers: " << inliers.size()<<endl;
	}

	cout<<"The model-coefficients are: (" << modelCoefficients[0]<<", " << modelCoefficients[1]<<
			", " << modelCoefficients[2]<<", " << modelCoefficients[3]<<")" <<endl;

	return(1);
}
