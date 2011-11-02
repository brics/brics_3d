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

#include "Triangle.h"
#include "assert.h"

namespace BRICS_3D {

Triangle::Triangle() {
	for (int i = 0; i <= 2; ++i) {
		vertices[i] = Point3D();
	}
}

Triangle::Triangle(Triangle* triangle) {
	for (int i = 0; i <= 2; ++i) {
		vertices[i] = Point3D(triangle->getVertex(i));
	}
}

Triangle::Triangle(Point3D vertex1, Point3D vertex2, Point3D vertex3) {
	vertices[0] = vertex1;
	vertices[1] = vertex2;
	vertices[2] = vertex3;
}

Triangle::~Triangle() {

}

Point3D* Triangle::getVertex(int vertexIndex){
	assert ((0 <= vertexIndex) && (vertexIndex <= 2)); // check if parameter is in range

	return &(vertices[vertexIndex]);
}

void Triangle::setVertex(int vertexIndex, Point3D vertex) {
	assert ((0 <= vertexIndex) && (vertexIndex <= 2)); // check if parameter is in range

	vertices[vertexIndex] = vertex;
}

void Triangle::homogeneousTransformation(IHomogeneousMatrix44 *transformation) {
	for (int i = 0; i <= 2; ++i) { // propagate transformation to vertices
		vertices[i].homogeneousTransformation(transformation);
	}
}

ostream& operator<<(ostream &outStream, Triangle &triangle) {
	outStream << *(triangle.getVertex(0))  << ", ";
	outStream << *(triangle.getVertex(1)) << ", ";
	outStream << *(triangle.getVertex(2));

	return outStream;
}

}

/* EOF */
