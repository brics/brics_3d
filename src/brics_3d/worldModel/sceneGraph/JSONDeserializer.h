/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2015, KU Leuven
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

#ifndef RSG_JSONDESERIALIZER_H_
#define RSG_JSONDESERIALIZER_H_

#include "brics_3d/util/JSONTypecaster.h"
#include "brics_3d/worldModel/WorldModel.h"

namespace brics_3d {
namespace rsg {



class JSONDeserializer {
public:
	JSONDeserializer(WorldModel* wm);
	virtual ~JSONDeserializer();

	int write(const char *dataBuffer, int dataLength, int &transferredBytes);
	int write(std::string data);

private:

	virtual bool handleGraphPrimitive(libvariant::Variant& atom, rsg::Id parentId);

	virtual bool doAddNode(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddGroup(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddTransformNode(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddGeometricNode(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddRemoteRootNode(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddConnection(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doSetNodeAttributes(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doSetTransform(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doDeleteNode(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddParent(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doRemoveParent(libvariant::Variant& group, rsg::Id parentId);

	WorldModel* wm;
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_JSONDESERIALIZER_H_ */

/* EOF */
