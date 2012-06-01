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

#ifndef COLOREDPOINT3D_H_
#define COLOREDPOINT3D_H_

#include "Point3DDecorator.h"

namespace BRICS_3D {

/**
 * @brief A class to represent a point in the Cartesian space that carries color information (RGB)
 */
class ColoredPoint3D : public Point3DDecorator {
public:

	/**
	 * @brief Standard constructor
	 */
	ColoredPoint3D();

	/**
	 * @brief Copy constructor that requires a Point3D pointer.
	 * @param[in] point The Point3D that will be copied and decorated with color functionality.
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
	 * @brief Copy constructor
	 * @param[in] point Reference to point that will be copied
	 */
	ColoredPoint3D(const ColoredPoint3D &point);


	ColoredPoint3D& operator=(const ColoredPoint3D &point);

	/**
	 * @brief Standard destructor
	 */
	virtual ~ColoredPoint3D();

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

	virtual Point3D* clone() const;

    ColoredPoint3D* asColoredPoint3D();

	/**
	 * @brief Sets the red-value for the point
	 * @param red	the new red-value
	 */
	inline void setR(unsigned char red){
		this->red = red;
	}

	/**
	 * @brief Sets the green-value for the point
	 * @param green	the new green-value
	 */
	inline void setG(unsigned char green){
		this->green = green;
	}

	/**
	 * @brief Sets the blue-value for the point
	 * @param blue	the new blue-value
	 */
	inline void setB(unsigned char blue){
		this->blue = blue;
	}



	/**
	 * @brief Returns the red-value for the point
	 * @return	the red-value
	 */
	inline unsigned char getR(){
		return this->red;
	}

	/**
	 * @brief Returns the green-value for the point
	 * @return	the green-value
	 */
	inline unsigned char getG(){
		return this->green;
	}

	/**
	 * @brief Returns the blue-value for the point
	 * @return	the blue-value
	 */
	inline unsigned char getB(){
		return this->blue;
	}


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
