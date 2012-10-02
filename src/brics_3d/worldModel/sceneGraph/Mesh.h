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

#ifndef RSG_MESH_H
#define RSG_MESH_H

#include "Shape.h"

namespace brics_3d {

namespace rsg {

/**
 * Abstract interface for mesh data in the scenegraph.
 * @ingroup sceneGraph
 */
template<typename MeshT>
class Mesh : public Shape {
  public:

	typedef boost::shared_ptr< Mesh<MeshT> > MeshPtr;
	typedef boost::shared_ptr< Mesh<MeshT> const> MeshConstPtr;

    Mesh(){};

    virtual ~Mesh(){};

    boost::shared_ptr<MeshT> data;

};

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

