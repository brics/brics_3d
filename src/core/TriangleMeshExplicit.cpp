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

#include "TriangleMeshExplicit.h"
#include "assert.h"


namespace BRICS_3D {

TriangleMeshExplicit::TriangleMeshExplicit() {
	triangles = new std::vector<Triangle> ();
	triangles->clear();

}

TriangleMeshExplicit::~TriangleMeshExplicit() {
	if (triangles != NULL) {
		triangles->clear();
		delete triangles;
	}
}

std::vector<Triangle>* TriangleMeshExplicit::getTriangles() {
	return triangles;
}

void TriangleMeshExplicit::setTriangles(std::vector<Triangle>* triangles) {
	if (this->triangles != NULL) {
		this->triangles->clear();
		delete this->triangles;
	}
	this->triangles = triangles;
}

int TriangleMeshExplicit::getSize() {
	return static_cast<int>(triangles->size());
}

int TriangleMeshExplicit::getNumberOfVertices() {
	return (this->getSize() * 3); //each triangle has three dedicated vertices.
}

Point3D* TriangleMeshExplicit::getTriangleVertex(int triangleIndex, int vertexIndex) {
	/* check if parameters are in range */
	assert ((0 <= triangleIndex) && (triangleIndex < this->getSize()));
	assert ((0 <= vertexIndex) && (vertexIndex <= 2));

	return (*triangles)[triangleIndex].getVertex(vertexIndex);
}

int TriangleMeshExplicit::addTriangle(Point3D vertex1, Point3D vertex2, Point3D vertex3) {
	triangles->push_back(Triangle(vertex1, vertex2, vertex3));
	return (getSize() - 1); // inserted as last entry
}

int TriangleMeshExplicit::addTriangle(Triangle triangle) {
	triangles->push_back(triangle);
	return (getSize() - 1); // inserted as last entry
}

void TriangleMeshExplicit::removeTriangle (int triangleIndex) {
//	triangles->erase(triangleIndex); // FIXME
}

void TriangleMeshExplicit::homogeneousTransformation(IHomogeneousMatrix44 *transformation) {
	for (int i = 0; i < this->getSize(); ++i) { // propagate transformation to triangles
		(*triangles)[i].homogeneousTransformation(transformation);
	}
}

void TriangleMeshExplicit::read(std::istream& inStream) {
	//TODO
}

void TriangleMeshExplicit::write(std::ostream& outStream) {
	for (int i = 0; i < this->getSize(); ++i) {
		outStream << (*triangles)[i] << std::endl;
	}
}

}

/* EOF */
