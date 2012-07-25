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

#ifndef IFUNCTIONBLOCK_H_
#define IFUNCTIONBLOCK_H_

#include <vector>
#include <core/ParameterSet.h>
#include <worldModel/WorldModel.h>

namespace BRICS_3D {

namespace RSG {

/**
 * A function block as it will be used for percetion algorithms within the world model.
 */
class IFunctionBlock {
public:
	IFunctionBlock(BRICS_3D::WorldModel* wmHandle){
		this->wm = wmHandle;
	};
	virtual ~IFunctionBlock(){};

	virtual void configure(BRICS_3D::ParameterSet parameters) = 0;
	virtual void setData(std::vector<unsigned int>& inputDataIds) = 0;
	virtual void execute() = 0;
	virtual void getData(std::vector<unsigned int>& newDataIds) = 0;


protected:
	/// Handle to the world model that stores all 3D data.
	BRICS_3D::WorldModel* wm;

	std::vector<unsigned int> inputDataIds;
	std::vector<unsigned int> outputDataIds;

private:
	IFunctionBlock(){};

};

}

}

#endif /* IFUNCTIONBLOCK_H_ */

/* EOF */
