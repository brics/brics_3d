/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2014, KU Leuven
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
#ifndef HDF5UPDATEDESERIALIZER_H_
#define HDF5UPDATEDESERIALIZER_H_

#include "brics_3d/core/Logger.h"
#include "brics_3d/worldModel/WorldModel.h"
#include "brics_3d/util/HDF5Typecaster.h"

namespace brics_3d {
namespace rsg {

class IInputPort {
public:
	IInputPort(){};
	virtual ~IInputPort(){};
	virtual int write(const char *dataBuffer, int dataLength, int &transferredBytes) = 0; // triggers update of WM
};

class HDF5UpdateDeserializer : public IInputPort {
public:
	HDF5UpdateDeserializer(WorldModel* wm);
	virtual ~HDF5UpdateDeserializer();

	int write(const char *dataBuffer, int dataLength, int &transferredBytes);

protected:

	bool handleSceneGraphUpdate(const char *dataBuffer, int dataLength, int &transferredBytes);

	//Functions that do the actual work (template method)
	virtual bool doAddNode(H5::Group& group);
	virtual bool doAddGroup(H5::Group& group);
	virtual bool doAddTransformNode(H5::Group& group);
	virtual bool doAddGeometricNode(H5::Group& group);
	virtual bool doAddRemoteRootNode(H5::Group& group);
	virtual bool doAddConnection(H5::Group& group);
	virtual bool doSetNodeAttributes(H5::Group& group);
	virtual bool doSetTransform(H5::Group& group);
	virtual bool doDeleteNode(H5::Group& group);
	virtual bool doAddParent(H5::Group& group);
	virtual bool doRemoveParent(H5::Group& group);

private:
	WorldModel* wm;
	Id parentId;
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* HDF5UPDATEDESERIALIZER_H_ */

/* EOF */
