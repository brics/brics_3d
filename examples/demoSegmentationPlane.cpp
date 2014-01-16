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

#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/core/Point3D.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/algorithm/segmentation/RegionBasedSACSegmentation.h>
#include <brics_3d/algorithm/filtering/MaskROIExtractor.h>
#include <brics_3d/util/OSGPointCloudVisualizer.h>

#include <iostream>

using brics_3d::Logger;

using namespace std;
int main(int argc, char **argv) {

	//Configure the logger
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::INFO);

	//Cofigure input
	string filename;
	if (argc == 2) {
		filename = argv[1];
	} else {
		filename = "../data/segmentation_data/groundTruthData/bureau3/bureau3.txt";
		LOG(INFO) << "No parameter given. Using default data set file: " << filename;
	}

	//Create a pointcloud object
	brics_3d::PointCloud3D cloud;
	brics_3d::PointCloud3D planeCloud;

	//read the points into the pointcloud
	//Please modify the path if there is a file read error.
	cloud.readFromTxtFile(filename);

	LOG(INFO) << "INFO: Current PointCloud Size: " <<cloud.getSize();

	//Create the vector to hold the model coefficients
	Eigen::VectorXd modelCoefficients;

	//Create the vector to hold the indexes of the model inliers
	std::vector<int> inliers;

	//Create the SACSegmentation Object
	brics_3d::RegionBasedSACSegmentation sacSegmenter;

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
		LOG(INFO) <<"Could not estimate a planar model for the given dataset.";
		return (-1);
	}else {
		LOG(INFO) <<"Found Inliers: " << inliers.size();
	}

	LOG(INFO) <<"The model-coefficients are: (" << modelCoefficients[0]<<", " << modelCoefficients[1]<<
			", " << modelCoefficients[2]<<", " << modelCoefficients[3]<<")";

	brics_3d::MaskROIExtractor extractor;
	extractor.extractIndexedPointCloud(&cloud, inliers, &planeCloud);

	brics_3d::OSGPointCloudVisualizer visualizer;
	visualizer.addPointCloud(&cloud);
	visualizer.visualizePointCloud(&planeCloud, 0, 1, 0, 0.8);

	return(1);
}
