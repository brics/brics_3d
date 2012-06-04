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

#ifndef CUBEROIEXTRACTOR_H_
#define CUBEROIEXTRACTOR_H_

#include "IFiltering.h"
#include "core/IHomogeneousMatrix44.h"

namespace BRICS_3D {

/**
 * The origin of box is considered to be in the center of each size value.
 */
class BoxROIExtractor : public IFiltering  {
public:
	BoxROIExtractor();

	BoxROIExtractor(Coordinate sizeX, Coordinate sizeY, Coordinate sizeZ);

	virtual ~BoxROIExtractor();


	void filter(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud);


    Coordinate getSizeX() const
    {
        return sizeX;
    }

    Coordinate getSizeY() const
    {
        return sizeY;
    }

    Coordinate getSizeZ() const
    {
        return sizeZ;
    }

    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getBoxOrigin() const
    {
        return boxOrigin;
    }

    void setSizeX(Coordinate sizeX)
    {
        this->sizeX = sizeX;
    }

    void setSizeY(Coordinate sizeY)
    {
        this->sizeY = sizeY;
    }

    void setSizeZ(Coordinate sizeZ)
    {
        this->sizeZ = sizeZ;
    }

    void setBoxOrigin(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr boxOrigin)
    {
        this->boxOrigin = boxOrigin;
    }

private:

    Coordinate sizeX;
    Coordinate sizeY;
    Coordinate sizeZ;

    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr boxOrigin;

};

}  // namespace BRICS_3D

#endif /* CUBEROIEXTRACTOR_H_ */

/* EOF */
