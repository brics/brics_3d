/**
 * @file 
 * ColoredPointCloud3D.cpp
 *
 * @date: Dec 21, 2009
 * @author: sblume
 */

#include "ColoredPointCloud3D.h"

using std::vector;

namespace BRICS_3D {

ColoredPointCloud3D::ColoredPointCloud3D() {
	pointCloud = new vector<ColoredPoint3D>();
	pointCloud->clear();

}

ColoredPointCloud3D::~ColoredPointCloud3D() {
	if (pointCloud != NULL) {
		pointCloud->clear();
		delete pointCloud;
	}
}

unsigned int ColoredPointCloud3D::getSize() {
	return pointCloud->size();
}

void ColoredPointCloud3D::addPoint(ColoredPoint3D point) {
	pointCloud->push_back(point);
}

std::vector<ColoredPoint3D>* ColoredPointCloud3D::getPointCloud() {
	return pointCloud;

}

void ColoredPointCloud3D::setPointCloud(std::vector<ColoredPoint3D>* pointCloud) {
	this->pointCloud = pointCloud;
}


}

/* EOF */
