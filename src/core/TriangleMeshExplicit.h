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

#ifndef TRIANGLEMESHEXPLICIT_H_
#define TRIANGLEMESHEXPLICIT_H_

#include "core/ITriangleMesh.h"
#include "core/Triangle.h"
#include <vector>

namespace BRICS_3D {

/**
 * @brief @e Explicit triangle mesh representation.
 */
class TriangleMeshExplicit : public ITriangleMesh {
public:

	/**
	 * @brief Standard constructor
	 */
	TriangleMeshExplicit();

	/**
	 * @brief Standard destructor
	 */
	virtual ~TriangleMeshExplicit();

	/**
	 * @brief Get the pointer to the triangles
	 * To get access to the vertices of the mesh, prefer getTriangleVertex(),
	 * as that function is more generic and complies to the ITriangleMesh interface.
	 *
	 * @return Pointer to triangles
	 */
    std::vector<Triangle>* getTriangles();

	/**
	 * @brief Set the pointer to the triangles
	 * @param triangles Pointer to new triangle set
	 */
    void setTriangles(std::vector<Triangle>* triangles);

	int getSize();

	int getNumberOfVertices();

	Point3D* getTriangleVertex(int triangleIndex, int vertexIndex);

	int addTriangle(Point3D vertex1, Point3D vertex2, Point3D vertex3);

	/**
	 * @brief Add a new triangle to the mesh
	 *
	 * @param triangle The triangle that will be added
	 * @return Returns the triangle index where it is stored
	 *
	 */
	int addTriangle(Triangle triangle);

	void removeTriangle (int triangleIndex);

	void homogeneousTransformation(IHomogeneousMatrix44 *transformation);

protected:

	///Pointer to vector which represents a triangle set
	std::vector<Triangle>* triangles;

private:

	void read(std::istream& inStream);

	void write(std::ostream& outStream);
};

}

#endif /* TRIANGLEMESHEXPLICIT_H_ */

/* EOF */
