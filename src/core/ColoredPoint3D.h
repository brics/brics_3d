/**
 * @file 
 * ColoredPoint3D.h
 *
 * @date: Dec 21, 2009
 * @author: sblume
 */

#ifndef COLOREDPOINT3D_H_
#define COLOREDPOINT3D_H_

//#include "PointCloud3D.h"
#include "Point3DDecorator.h"

namespace BRICS_3D {

/**
 * @brief A class to represent a point in the Cartesian space that carries color information (RGB)
 */
//class ColoredPoint3D : public Point3D {
class ColoredPoint3D : public Point3DDecorator {
public:

//	/*
//	 * @brief Standard constructor
//	 */
//	ColoredPoint3D();
//
//	/*
//	 * @brief Constructor with full initialization
//	 * @param x X coordinate in Cartesian system
//	 * @param y Y coordinate in Cartesian system
//	 * @param z Z coordinate in Cartesian system (height)
//	 * @param red Red color channel
//	 * @param green Green color channel
//	 * @param blue Blue color channel
//	 */
//	ColoredPoint3D(double x, double y, double z , unsigned char red, unsigned char green, unsigned char blue);
//
//	/*
//	 * @brief Copy constructor
//	 * @param[in] point Pointer to point that will be copied
//	 */
//	ColoredPoint3D(ColoredPoint3D* point);

	/**
	 * @brief Constructor that requires a Point3D pointer
	 * @param[in] point The Point3D that will be decorated with color functionality.
	 */
	ColoredPoint3D(Point3D* point);

	/**
	 * @brief Constructor with full initialization
	 * @param[in] point The Point3D that will be decorated with color functionality.
	 * @param red Red color channel
	 * @param green Green color channel
	 * @param blue Blue color channel
	 */
	ColoredPoint3D(Point3D* point, unsigned char red, unsigned char green, unsigned char blue);

	/**
	 * @brief Copy constructor
	 * @param[in] point Pointer to point that will be copied
	 */
	ColoredPoint3D(ColoredPoint3D* point);

	/**
	 * @brief Standard destructor
	 */
	virtual ~ColoredPoint3D();

	Point3D operator+(const Point3D *point);

	Point3D operator-(const Point3D *point);

	Point3D operator*(double scalar);

	/**
	 * @brief Overridden >> operator.
	 *
	 * Reads a point from a stream an stores it. E.g. e.g. std::cin >> coloredPointObj;
	 *
	 * @param inStream The input stream
	 * @param point Pointer to point where input is stored
	 * @return Input stream
	 */
	friend istream& operator>>(istream &inStream, ColoredPoint3D &point);

	/**
	 * @brief Overridden << operator.
	 *
	 * Writes a point to a stream e.g. std::cout << coloredPointObj;
	 *
	 * @param outStream The output stream
	 * @param point Pointer to point which data is forwarded to output stream
	 * @return Output stream
	 */
	friend ostream& operator<<(ostream &outStream, const ColoredPoint3D &point);

	/// Red color channel for the point
	unsigned char red;

	/// Green color channel for the point
	unsigned char green;

	/// Blue color channel for the point
	unsigned char blue;

};

}

#endif /* COLOREDPOINT3D_H_ */

/* EOF */
