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

#ifndef BRICS_3D_TRIANGLEMESHIMPLICIT_H_
#define BRICS_3D_TRIANGLEMESHIMPLICIT_H_

#include "brics_3d/core/ITriangleMesh.h"
#include <vector>

namespace brics_3d {

/**
 * @brief @e Implicit triangle mesh representation.
 */
class TriangleMeshImplicit : public ITriangleMesh {
public:

	/**
	 * @brief Standard constructor
	 */
	TriangleMeshImplicit();

	/**
	 * @brief Standard destructor
	 */
	virtual ~TriangleMeshImplicit();

	/**
	 * @brief Get the pointer to the vertices
	 * To get access to the vertices of the mesh, prefer getTriangleVertex(),
	 * as that function is more generic and complies to the ITriangleMesh interface.
	 *
	 * @return Pointer to vertices
	 */
    std::vector<Point3D>* getVertices();

	/**
	 * @brief Set the pointer to the vertices
	 * @param vertices Pointer to new vertex set
	 */
    void setVertices(std::vector<Point3D>* vertices);

	/**
	 * @brief Get the pointer to the vertex indices
	 * To get access to the vertices of the mesh, prefer getTriangleVertex(),
	 * as that function is more generic and complies to the ITriangleMesh interface.
	 *
	 * @return Pointer to indices
	 */
    std::vector<int>* getIndices();

	/**
	 * @brief Set the pointer to the indices
	 * @param indices Pointer to new index set
	 */
    void setIndices(std::vector<int>* indices);

	int getSize();

	int getNumberOfVertices();

	Point3D* getTriangleVertex(int triangleIndex, int vertexIndex);

	int addTriangle(Point3D vertex1, Point3D vertex2, Point3D vertex3);

	void removeTriangle (int triangleIndex);

	void homogeneousTransformation(IHomogeneousMatrix44 *transformation);

protected:

	/// Pointer to vector with vertices
	std::vector<Point3D>* vertices;

	/**
	 * @brief Pointer to vector with indices. The \f$k\f$-th triangle is defined as a triple of the following indices:
	 * \f$ (3k, 3k+1, 3k+2) \f$
	 */
	std::vector<int>* indices;

private:

	void read(std::istream& inStream);

	void write(std::ostream& outStream);

};

}

#endif /* BRICS_3D_TRIANGLEMESHIMPLICIT_H_ */

/* EOF */
