/**
 * @file 
 * Triangle.h
 *
 * @date: Feb 24, 2010
 * @author: sblume
 */

#ifndef TRIANGLE_H_
#define TRIANGLE_H_

#include "core/Point3D.h"

namespace BRICS_3D {

/**
 * @brief Basic representation of a trinagle
 *
 * The triangle is represented by three vertices.
 */
class Triangle {
public:

	/**
	 * @brief Standard constructor
	 */
	Triangle();

	/**
	 * @brief Copy constructor
	 */
	Triangle(Triangle* triangle);

	/**
	 * Constuctor that allows to fully define a triangle
	 */
	Triangle(Point3D vertex1, Point3D vertex2, Point3D vertex3);

	/**
	 * @brief Standard destructor
	 */
	virtual ~Triangle();

	/**
	 * @brief Get a vertex that belongs to this triangle
	 *
	 * @param vertexIndex The index of a vertex the belongs to this triangle.
	 *        The valid range is [0, 2]
	 * @return Returns a pointer to the vertex
	 */
	Point3D* getVertex(int vertexIndex);

	/**
	 * @brief Set a vertex that belongs to this triangle
	 *
	 * @param vertexIndex The index of a vertex the belongs to this triangle.
	 *        The valid range is [0, 2]
	 * @param vertex The new vertex that will replace the previous one.
	 */
	void setVertex(int vertexIndex, Point3D vertex);

	/**
	 * @brief Applies a homogeneous transformation to the triangle
	 *
	 * @param[in] transformation The homogeneous transformation matrix that will be applied
	 */
	void homogeneousTransformation(IHomogeneousMatrix44 *transformation);

	/**
	 * @brief Overridden << operator.
	 *
	 * Writes a point to a stream e.g. std::cout << triangleObj;
	 *
	 * @param outStream The output stream
	 * @param triangle Reference to point which data is forwarded to output stream
	 * @return Output stream
	 */
	friend ostream& operator<<(ostream &outStream, Triangle &triangle);

protected:

	/// Array that holds three vertices, that form the triangle
	Point3D vertices[3];
};

}

#endif /* TRIANGLE_H_ */

/* EOF */