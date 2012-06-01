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

#ifndef POINT3DDECORATOR_H_
#define POINT3DDECORATOR_H_

#include "core/Point3D.h"

namespace BRICS_3D {

/**
 * @brief Abstract class that can extend or "decorate" a Point3D.
 *
 * Here we force the user/implementor if this decorator interface to re-implement all functions of Point3D, which will be most of the times just forwarded to the
 * Point3D reference "point".
 */
class Point3DDecorator : public Point3D {
public:

	/**
	 * @brief Standard constructor.
	 */
	Point3DDecorator();

	/**
	 * @brief Constructor that requires a Point3D pointer
	 * @param[in] point The Point3D that will be copied and wrapped/enhanced/decorated by this decorator.
	 *
	 * Note that there is no standard constructor, to prevent a situation with an Point3DDecorator instance,
	 * but no internal Point3D specified.
	 */
	Point3DDecorator(Point3D* point);

	/**
	 * @brief Copy constructor
	 * @param point Pointer to a decorated point that will be copied and decorated
	 */
	Point3DDecorator(Point3DDecorator* point);

	/**
	 * @brief Copy constructor
	 * @param point Reference to a decorated point that will be copied and decorated
	 */
	Point3DDecorator(const Point3DDecorator &point);

	/**
	 * @brief Assignement operator.
	 */
	Point3DDecorator& operator=(Point3DDecorator &point);

	/**
	 * @brief Standard destructor
	 */
	virtual ~Point3DDecorator();

	Coordinate getX() const;

	Coordinate getY() const;

	Coordinate getZ() const;

	void setX(Coordinate x);

	void setY(Coordinate y);

	void setZ(Coordinate z);

    void getRawData(Coordinate *pointBuffer);

	Point3D& operator=(const Point3D &point);

	Point3D operator+(const Point3D *point);

	Point3D operator-(const Point3D *point);

	Point3D operator*(double scalar);

	void homogeneousTransformation(IHomogeneousMatrix44 *transformation);

    friend istream& operator>>(istream &inStream, Point3D &point);

    friend ostream& operator<<(ostream &outStream, const Point3D &point);

	/**
	 * @brief decorates a point
	 *
	 * Decorating means that the decorator takes over ownership/responsibility for that (inner) point.
	 * NOTE: there is no copy created, in contrast to the copy constructors.
	 */
    void decorate(Point3D* point);

    /**
     * @brief Gets the next inner skin of decoration layer
     * With this function it is possible to traverse recursively the layers, depending on what kind of object type is returned.
     */
    Point3D* getPoint();

    virtual ColoredPoint3D* asColoredPoint3D();

protected:

    ///Pointer to the next inner skin of decoration layer. Can be either another decoration or a (real) Point3D.
	Point3D* point;

};

/**
 * @brief Deduce if a point has a certain layer of decoration.
 * This is rather generic way to intospect the decorated points. For some decoration layers ther are some
 * (faster and) convenient functions like BRICS_3D::Point3D::asColoredPoint3D()
 *
 * @param queryPoint Point of intrest that shall be investigated if it has a certain decoration.
 * @return Pointer to type as specified by DecorationT or null if the point does not have such a decoration layer.
 */
template<class DecorationT>
inline DecorationT* getPointType(Point3D* queryPoint) {
	if (dynamic_cast<DecorationT*>(queryPoint) != 0) {
		return dynamic_cast<DecorationT*>(queryPoint); //found
	} else if (dynamic_cast<Point3DDecorator*>(queryPoint) != 0) {
		return getPointType<DecorationT>( dynamic_cast<Point3DDecorator*>(queryPoint)->getPoint() ); //feed forward
	} else {
		return 0; //not found
	}
};

}

#endif /* POINT3DDECORATOR_H_ */

/* EOF */
