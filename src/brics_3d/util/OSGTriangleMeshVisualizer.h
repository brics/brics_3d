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

#ifndef BRICS_3D_OSGTRIANGLEMESHVISUALIZER_H_
#define BRICS_3D_OSGTRIANGLEMESHVISUALIZER_H_

#include <osg/Group>
#include <osg/Geometry>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>

#include "core/ITriangleMesh.h"

namespace brics_3d {

/**
 * @brief Displays triangle meshes in an Open Scene Graph (OSG) viewer.
 * @ingroup visualization
 */
class OSGTriangleMeshVisualizer {
public:

	OSGTriangleMeshVisualizer();

	virtual ~OSGTriangleMeshVisualizer();

	void addTriangleMesh(ITriangleMesh* mesh);

	void visualize();

	static osg::ref_ptr<osg::Node> createTriangleMeshNode(ITriangleMesh* mesh, float red=1.0f, float green=1.0f, float blue=1.0f, float alpha=1.0f);

private:

	/**
	 * @brief Create an OSG TriangleMesh from an ITriangleMesh.
	 */
	static osg::ref_ptr<osg::TriangleMesh> createTriangleMesh(ITriangleMesh* mesh);

	/// OSG viewer object
	osgViewer::Viewer viewer;

	/// Root node for scenegraph
	osg::Group* rootGeode;

};

}

#endif /* BRICS_3D_OSGTRIANGLEMESHVISUALIZER_H_ */

/* EOF */
