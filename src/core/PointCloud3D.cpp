/**
 * @file
 * PointCloud.cpp
 * 
 * @author: Sebastian Blumenthal
 * @date: Aug 24, 2009
 * @version: 0.1
 */

#include "PointCloud3D.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>

using namespace std;

namespace BRICS_3D {

PointCloud3D::PointCloud3D() {
	pointCloud = new vector<Point3D> ();
	pointCloud->clear();

}

PointCloud3D::~PointCloud3D() {
	if (pointCloud != NULL) {
		pointCloud->clear();
		delete pointCloud;
	}
}

void PointCloud3D::addPoint(Point3D point) {
	pointCloud->push_back(point);
}

std::vector<Point3D> *PointCloud3D::getPointCloud() {
	return pointCloud;

}

void PointCloud3D::setPointCloud(std::vector<Point3D> *pointCloud) {
	this->pointCloud = pointCloud;
}

int PointCloud3D::getSize() {
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
	for (int i = 0; i < pointCloud->size(); ++i) {
		outputFile << (*pointCloud)[i].x << " ";
		outputFile << (*pointCloud)[i].y << " ";
		outputFile << (*pointCloud)[i].z << endl;
	}

	outputFile.close();
}

void PointCloud3D::storeToTxtFile(std::string filename) {
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

void PointCloud3D::readFromTxtFile(std::string filename) {
	Point3D *tmpPoint;
	ifstream inputFile;
	string line;

	inputFile.open(filename.c_str());
	cout << "INFO: Reading point cloud from: " << filename << endl;

	while(getline(inputFile, line)) {
		tmpPoint = new Point3D;
		stringstream lineStream(line);
		if (!inputFile.good()) {
			throw runtime_error("ERROR: cannot read point.");
		}
		lineStream >> *tmpPoint;
		this->addPoint(tmpPoint);
	}

	inputFile.close();
}

istream& operator>>(istream &inStream, PointCloud3D &pointCloud) {
	Point3D *tmpPoint;
	string line;

	while(getline(inStream, line)) {
		tmpPoint = new Point3D;
		stringstream lineStream(line);
		if (!lineStream.good()) {
			throw runtime_error("ERROR: cannot read point.");
		}
		lineStream >> *tmpPoint;
		pointCloud.addPoint(tmpPoint);
	}

	return inStream;
}

ostream& operator<<(ostream &outStream, PointCloud3D &pointCloud) {

	for (int i = 0; i < pointCloud.getSize(); ++i) {
		outStream << (*pointCloud.getPointCloud())[i] << endl;
	}

	return outStream;
}

}

/* EOF */
