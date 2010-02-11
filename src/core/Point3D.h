/**
 * @file 
 * Point3D.h
 *
 * @date: Oct 14, 2009
 * @author: sblume
 */

#ifndef POINT3D_H_
#define POINT3D_H_

#include <iostream>
using std::ostream;
using std::istream;

#include "core/IHomogeneousMatrix44.h"

/**
 * @namespace BRICS_3D
 */
namespace BRICS_3D {

/**
 * @brief Coordinate data type.
 *
 * This typedef represents the a Cartesian coordinate.
 * Typically it is a set to a double, but it can be changed to float
 * if needed. Use redefinition with care.
 */
typedef double	Coordinate;				// coordinate data type
//typedef float	Coordinate;				// coordinate data type

/**
 * @brief A class to represent a point in the Cartesian space
 *
 * This class is the basic representation of a three dimensional point in a Cartesian space.
 * Basic vector functionality is also provided.
 * Note that all data fields (coordinates) are private. (This is required by the Point3DDecorator.)
 *
 */
class Point3D {
public:

	/**
	 * @brief Default constructor
	 */
	Point3D();

	/**
	 * @brief Constructor with full initialization
	 * @param x X coordinate in Cartesian system
	 * @param y Y coordinate in Cartesian system
	 * @param z Z coordinate in Cartesian system (height)
	 */
	Point3D(Coordinate x, Coordinate y, Coordinate z);

	/**
	 * @brief Copy constructor
	 * @param[in] point Pointer to point that will be copied
	 */
	Point3D(Point3D *point);

	/**
	 * @brief Standard destructor
	 */
	virtual ~Point3D();

	/**
	 * @brief Get the x coordinate
	 * This is the only way to access a coordinate
	 */
	virtual Coordinate getX() const;

	/**
	 * @brief Get the y coordinate
	 * This is the only way to access a coordinate
	 */
	virtual Coordinate getY() const;

	/**
	 * @brief Get the y coordinate
	 * This is the only way to access a coordinate
	 */
	virtual Coordinate getZ() const;

	/**
	 * @brief Set the x coordinate
	 * This is the only way to manipulate a coordinate
	 */
	virtual void setX(Coordinate x);

	/**
	 * @brief Set the y coordinate
	 * This is the only way to manipulate a coordinate
	 */
	virtual void setY(Coordinate y);

	/**
	 * @brief Set the z coordinate
	 * This is the only way to manipulate a coordinate
	 */
	virtual void setZ(Coordinate z);

	/**
	 * @brief Copy raw data to input buffer (array)
	 * @param[out] pointBuffer Pointer to buffer, where to store the raw data
	 */
	virtual void getRawData(Coordinate *pointBuffer);

	/**
	 * @brief Add a point to another point e.i each coordinate is added.
	 * @param[in] point Pointer to point that will be added
	 * @return Result of addition
	 */
	virtual Point3D operator+(const Point3D *point);

	/**
	 * @brief Subtract a point to another point e.i each coordinate is substracted
	 * @param[in] point Pointer to point that will be subtracted
	 * @return Result of substraction
	 */
	virtual Point3D operator-(const Point3D *point);

	/**
	 * @brief Multiply a point by a scalar
	 * @param scalar Scalar that will be multiplied to each coordinate
	 * @return Result of scalar multiplication
	 */
	virtual Point3D operator*(double scalar);

	virtual Point3D& operator=(const Point3D &point);

	/**
	 * @brief Applies a homogeneous transformation matrix to the point.
	 *
	 * @param[in] transformation The homogeneous transformation matrix that will be applied
	 */
	virtual void homogeneousTransformation(IHomogeneousMatrix44 *transformation);

	/**
	 * @brief Overridden >> operator.
	 *
	 * Reads a point from a stream an stores it. E.g. e.g. std::cin >> pointObj;
	 *
	 * @param inStream The input stream
	 * @param point Pointer to point where input is stored
	 * @return Input stream
	 */
	friend istream& operator>>(istream &inStream, Point3D &point);

	/**
	 * @brief Overridden << operator.
	 *
	 * Writes a point to a stream e.g. std::cout << pointObj;
	 *
	 * @param outStream The output stream
	 * @param point Pointer to point which data is forwarded to output stream
	 * @return Output stream
	 */
	friend ostream& operator<<(ostream &outStream, const Point3D &point);

private:

	/// X coordinate in Cartesian system
	Coordinate x;

	/// Y coordinate in Cartesian system
	Coordinate y;

	/// Z coordinate in Cartesian system (height)
	Coordinate z;
};

}

#endif /* POINT3D_H_ */

/* EOF */

