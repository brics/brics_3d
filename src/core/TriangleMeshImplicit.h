/**
 * @file 
 * TriangleMeshImplicit.h
 *
 * @date: Feb 24, 2010
 * @author: sblume
 */

#ifndef TRIANGLEMESHIMPLICIT_H_
#define TRIANGLEMESHIMPLICIT_H_

#include "core/ITriangleMesh.h"
#include <vector>

namespace BRICS_3D {

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


	int getSize();

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

#endif /* TRIANGLEMESHIMPLICIT_H_ */

/* EOF */
