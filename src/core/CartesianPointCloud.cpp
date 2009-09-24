/**
 * @file
 * brics_3dpm 
 * PointCloud.cpp
 * 
 * @author: Sebastian Blumenthal
 * @date: Aug 24, 2009
 * @version: 0.1
 */

#include "CartesianPointCloud.h"
#include <iostream>
#include <fstream>

namespace brics {

CartesianPointCloud::CartesianPointCloud() {
	pointCloud = new vector<CartesianPoint3D> ();
	pointCloud->clear();

}

CartesianPointCloud::~CartesianPointCloud() {
	if (pointCloud != NULL) {
		pointCloud->clear();
		delete pointCloud;
	}
}

void CartesianPointCloud::addPoint(CartesianPoint3D point) {
	pointCloud->push_back(point);
}

vector<CartesianPoint3D> *CartesianPointCloud::getPointCloud() {
	return pointCloud;

}

void CartesianPointCloud::setPointCloud(vector<CartesianPoint3D> *pointCloud) {
	this->pointCloud = pointCloud;
}

int CartesianPointCloud::getSize() {
	return pointCloud->size();
}

void CartesianPointCloud::storeToPlyFile(string filename) {
	ofstream outputFile;
	outputFile.open(filename.c_str());
	cout << "INFO: Saving point cloud to: " << filename << endl;

	/* write ply header */
	outputFile << "ply" << endl;
	outputFile << "format ascii 1.0" << endl;
	outputFile << "comment created by brics::CartesianPointCloud::storeToPlyFile" << endl;
	outputFile << "element vertex " << pointCloud->size() << endl;
	outputFile << "property float32 x" << endl;
	outputFile << "property float32 y" << endl;
	outputFile << "property float32 z" << endl;
	outputFile << "end_header" << endl;

	/*
	ply
	format ascii 1.0
	comment created by MATLAB plywrite
	element vertex 24364
	property float32 x
	property float32 y
	property float32 z
	end_header
	*/

	/* add data to file */
	for (int i = 0; i < pointCloud->size(); ++i) {
		outputFile << (*pointCloud)[i].x << " ";
		outputFile << (*pointCloud)[i].y << " ";
		outputFile << (*pointCloud)[i].z << endl;
	}

	outputFile.close();
}

void CartesianPointCloud::storeToTxtFile(string filename) {
	ofstream outputFile;
	outputFile.open(filename.c_str());
	cout << "INFO: Saving point cloud to: " << filename << endl;

	for (int i = 0; i < pointCloud->size(); ++i) {
		outputFile << (*pointCloud)[i].x << " ";
		outputFile << (*pointCloud)[i].y << " ";
		outputFile << (*pointCloud)[i].z << endl;
	}

	outputFile.close();
}

}

/* EOF */
