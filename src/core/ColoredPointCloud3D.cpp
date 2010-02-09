/**
 * @file 
 * ColoredPointCloud3D.cpp
 *
 * @date: Dec 21, 2009
 * @author: sblume
 */

#include "ColoredPointCloud3D.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>

using std::vector;
using std::string;
using std::runtime_error;
using std::stringstream;
using std::endl;

namespace BRICS_3D {

ColoredPointCloud3D::ColoredPointCloud3D() {
	this->pointCloud = new vector<ColoredPoint3D>();
	this->pointCloud->clear();

}

ColoredPointCloud3D::~ColoredPointCloud3D() {
	if (this->pointCloud != NULL) {
		this->pointCloud->clear();
		delete this->pointCloud;
	}
}

unsigned int ColoredPointCloud3D::getSize() {
	return this->pointCloud->size();
}

void ColoredPointCloud3D::addPoint(ColoredPoint3D point) {
	this->pointCloud->push_back(point);
}

//std::vector<Point3D>* ColoredPointCloud3D::getPointCloud() {
//	return dynamic_cast<std::vector<Point3D>>(this->pointCloud);
//}

std::vector<ColoredPoint3D>* ColoredPointCloud3D::getPointCloud() {
//std::vector<ColoredPoint3D>* ColoredPointCloud3D::getColoredPointCloud() {
	return this->pointCloud;

}

void ColoredPointCloud3D::setPointCloud(std::vector<ColoredPoint3D>* pointCloud) {
	this->pointCloud = pointCloud;
}

istream& operator>>(istream &inStream, ColoredPointCloud3D &pointCloud) {
	ColoredPoint3D *tmpPoint;
	string line;

	while(getline(inStream, line)) {
//		tmpPoint = new ColoredPoint3D;
		tmpPoint = new ColoredPoint3D(new Point3D());
		stringstream lineStream(line);
		if (!lineStream.good()) {
			throw runtime_error("ERROR: cannot read point.");
		}
		lineStream >> *tmpPoint;
		pointCloud.addPoint(tmpPoint);
	}

	return inStream;
}

ostream& operator<<(ostream &outStream, ColoredPointCloud3D &pointCloud) {
	for (unsigned int i = 0; i < pointCloud.getSize(); ++i) {
		outStream << (*pointCloud.getPointCloud())[i] << endl;
	}

	return outStream;
}

void ColoredPointCloud3D::homogeneousTransformation(IHomogeneousMatrix44 *transformation) {
	for (unsigned int i = 0; i < this->pointCloud->size(); ++i) {
		(*this->pointCloud)[i].homogeneousTransformation(transformation);
	}
}

}

/* EOF */
