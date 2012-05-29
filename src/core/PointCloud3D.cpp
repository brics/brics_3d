/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Sebastian Blumenthal
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

#include "PointCloud3D.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>

using namespace std;

namespace BRICS_3D {

PointCloud3D::PointCloud3D() {

#ifdef USE_POINTER_VECTOR
	pointCloud = new boost::ptr_vector<Point3D>();
#else
	pointCloud = new vector<Point3D> ();
#endif
	pointCloud->clear();

}

PointCloud3D::~PointCloud3D() {
	if (pointCloud != NULL) {
		pointCloud->clear();
		delete pointCloud;
	}
}

#ifdef USE_POINTER_VECTOR

void PointCloud3D::addPoint(Point3D point) {
	pointCloud->push_back(new Point3D(point));
}

boost::ptr_vector<Point3D> *PointCloud3D::getPointCloud() {
	return pointCloud;
}

void PointCloud3D::setPointCloud(boost::ptr_vector<Point3D> *pointCloud) {
	if (pointCloud != NULL) {
		pointCloud->clear();
		delete pointCloud;
	}
	this->pointCloud = pointCloud;
}

#else

void PointCloud3D::addPoint(Point3D point) {
	pointCloud->push_back(point);
}

std::vector<Point3D> *PointCloud3D::getPointCloud() {
	return pointCloud;
}

void PointCloud3D::setPointCloud(std::vector<Point3D> *pointCloud) {
	if (pointCloud != NULL) {
		pointCloud->clear();
		delete pointCloud;
	}
	this->pointCloud = pointCloud;
}


#endif

unsigned int PointCloud3D::getSize() {
	return pointCloud->size();
}

void PointCloud3D::storeToPlyFile(std::string filename) {
	ofstream outputFile;
	outputFile.open(filename.c_str());
	cout << "INFO: Saving point cloud to: " << filename << endl;

	/* write ply header */
	outputFile << "ply" << endl;
	outputFile << "format ascii 1.0" << endl;
	outputFile << "comment created by BRICS_3D::PointCloud3D::storeToPlyFile" << endl;
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
	for (unsigned int i = 0; i < pointCloud->size(); ++i) {
		outputFile << (*pointCloud)[i].getX() << " ";
		outputFile << (*pointCloud)[i].getY() << " ";
		outputFile << (*pointCloud)[i].getZ() << endl;
	}

	outputFile.close();
}

void PointCloud3D::storeToTxtFile(std::string filename) {
	ofstream outputFile;
	outputFile.open(filename.c_str());
	cout << "INFO: Saving point cloud to: " << filename << endl;

	for (unsigned int i = 0; i < pointCloud->size(); ++i) {
		outputFile << (*pointCloud)[i].getX() << " ";
		outputFile << (*pointCloud)[i].getY() << " ";
		outputFile << (*pointCloud)[i].getZ() << endl;
	}

	outputFile.close();
}

void PointCloud3D::readFromTxtFile(std::string filename) {
	ifstream inputFile;
	string line;

	inputFile.open(filename.c_str());

	if(inputFile.is_open()){
		cout << "INFO: Reading point cloud from: " << filename << endl;
		while(getline(inputFile, line)) {
		Point3D tmpPoint;
		stringstream lineStream(line);
		if (!inputFile.good()) {
			throw runtime_error("ERROR: cannot read point.");
		}
		lineStream >> tmpPoint;
		this->addPoint(&tmpPoint);
	}
	inputFile.close();
	} else {
		cout << "INFO: Error reading point cloud from: " << filename << endl;
	}
}

istream& operator>>(istream &inStream, PointCloud3D &pointCloud) {
	string line;

	while(getline(inStream, line)) {
		Point3D tmpPoint;
		stringstream lineStream(line);
		if (!lineStream.good()) {
			throw runtime_error("ERROR: cannot read point.");
		}
		lineStream >> tmpPoint;
		pointCloud.addPoint(&tmpPoint);
	}

	return inStream;
}

ostream& operator<<(ostream &outStream, PointCloud3D &pointCloud) {
	for (unsigned int i = 0; i < pointCloud.getSize(); ++i) {
		outStream << (*pointCloud.getPointCloud())[i] << endl;
	}

	return outStream;
}

void PointCloud3D::homogeneousTransformation(IHomogeneousMatrix44 *transformation) {
	for (unsigned int i = 0; i < pointCloud->size(); ++i) {
		(*pointCloud)[i].homogeneousTransformation(transformation);
	}
}

}

/* EOF */
