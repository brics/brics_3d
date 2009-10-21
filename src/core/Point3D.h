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

/**
 * @namespace brics
 */
namespace brics {

class HomogeneousMatrix4x4;
/**
 * @brief A class to represent a point in the Cartesian space
 *
 * This class is the basic representation of a three dimensional point in a Cartesian space.
 * Basic vector functionality is also provided
 *
 */
class Point3D {
public:

	/// X coordinate in Cartesian system
	double x;

	/// Y coordinate in Cartesian system
	double y;

	/// Z coordinate in Cartesian system (heigth)
	double z;

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
	Point3D(double x, double y, double z);

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
	 * @brief Copy raw data to input buffer (array)
	 * @param[out] pointBuffer Pointer to buffer, where to store the raw data
	 */
	void getRawData(double *pointBuffer);

	/**
	 * @brief Add a point to another point e.i each coordinate is added.
	 * @param[in] point Pointer to point that will be added
	 * @return Result of addition
	 */
	Point3D operator+(const Point3D *point);

	/**
	 * @brief Subtract a point to another point e.i each coordinate is substracted
	 * @param[in] point Pointer to point that will be subtracted
	 * @return Result of substraction
	 */
	Point3D operator-(const Point3D *point);

	/**
	 * @brief Multiply a point by a scalar
	 * @param scalar Scalar that will be mulitiplied to each coordinate
	 * @return Result of scalar multiplication
	 */
	Point3D operator*(double scalar);

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

	/// TODO: implement!
	Point3D homogeneousTransformation(HomogeneousMatrix4x4 *transformation); //TODO
};

}

#endif /* POINT3D_H_ */

/* EOF */

