/**
 * @file 
 * ITriangleMesh.h
 *
 * @date: Feb 24, 2010
 * @author: sblume
 */

#ifndef ITRIANGLEMESH_H_
#define ITRIANGLEMESH_H_

#include "Point3D.h"

namespace BRICS_3D {

/**
 * @brief Abstract interface for a triangle mesh.
 *
 * This interface should be generic enough to be independent from the underling representation of a triangle mesh.
 * Typical implementations realize a triangle either with an implicit (one vector for vertices and one vector for indices that form the triangles)
 * or an explicit representation (one vector of "triangles")
 * However, if certain properties of the implementation needs to be exploited (e.g. specialized versions of the triangle class are needed) then simply
 * use the more specialized version of a triangle mesh.
 */
class ITriangleMesh {
public:

	/**
	 * @brief Standard constructor
	 */
	ITriangleMesh(){};

	/**
	 * @brief Standard destructor
	 */
	virtual ~ITriangleMesh(){};

	/**
	 * @brief Get the number of triangles in the mesh
	 */
	virtual int getSize() = 0;

	/**
	 * @brief Get a vertex that belongs to a triangle
	 *
	 * @param triangleIndex The index of a triangle. The fist triangle has index 0, the second has 1 and so on...
	 *        The valid range is [0, getSize()]
	 * @param vertexIndex The index of a vertex the belongs to the triangle, specified by the triangleIndex.
	 *        The valid range is [0, 2]
	 * @return Returns a pointer to the vertex
	 *
	 */
	virtual Point3D* getTriangleVertex(int triangleIndex, int vertexIndex) = 0;

	/**
	 * @brief Add a new triangle to the mesh
	 *
	 * NOTE: The semantics on <b>how</b> a triangle is inserted depends on the actual implementation of this interface.
	 *
	 * @param vertex1 First vertex that represents the triangle
	 * @param vertex2 Second vertex that represents the triangle
     * @param vertex3 Third vertex that represents the triangle
	 * @return Returns the triangle index where it is stored
	 *
	 */
	virtual int addTriangle(Point3D vertex1, Point3D vertex2, Point3D vertex3) = 0;

	/**
	 * @brief Remove a triangle from the mesh
	 * NOTE: depending on the implementation of the triangle mesh, this might be a costly operation,
	 * as indices list might have to be updated
	 *
	 * @param triangleIndex The index of a triangle. The fist triangle has index 0, the second has 1 and so on...
	 *        The valid range is [0, getSize()]
	 */
	virtual void removeTriangle (int triangleIndex) = 0;

	/**
	 * @brief Applies a homogeneous transformation to the mesh
	 *
	 * @param[in] transformation The homogeneous transformation matrix that will be applied
	 */
	virtual void homogeneousTransformation(IHomogeneousMatrix44 *transformation) = 0;

	/**
	 * @brief Overridden >> operator.
	 *
	 * Reads a mesh from a stream an stores it e.g. std::cin >> meshObj;
	 *
	 * @param inStream The input stream
	 * @param mesh Reference to mesh where input is stored
	 * @return Input stream
	 */
	friend istream& operator>>(istream &inStream, ITriangleMesh &mesh);

	/**
	 * @brief Overridden << operator.
	 *
	 * Writes a mesh to a stream e.g. std::cout << pointObj;
	 *
	 * @param outStream The output stream
	 * @param mesh Reference to mesh which data is forwarded to output stream
	 * @return Output stream
	 */
	friend ostream& operator<<(ostream &outStream, ITriangleMesh &mesh);

private:

	/**
	 * @brief Helper function for streaming input
	 *
	 * This function enforces that streaming functionality is implemented in derived classes.
	 * This is necessaray as the operator>> cannot be inherited (as it is already a friend)
	 *
	 * @param[in,out] inStream The input stream
	 * @return Input stream
	 */
	virtual void read(std::istream& inStream) = 0;

	/**
	 * @brief Helper function for streaming output
	 *
	 * This function enforces that streaming functionality is implemented in derived classes.
	 * This is necessary as the operator<< cannot be inherited (as it is already a friend)
	 *
	 * @param[in,out] outStream The output stream
	 * @return Output stream
	 */
	virtual void write(std::ostream& outStream) = 0;

};

}

#endif /* ITRIANGLEMESH_H_ */

/* EOF */
