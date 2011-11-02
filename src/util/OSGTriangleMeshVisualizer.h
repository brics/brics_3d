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

#ifndef OSGTRIANGLEMESHVISUALIZER_H_
#define OSGTRIANGLEMESHVISUALIZER_H_

#include <osg/Group>
#include <osg/Geometry>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>

#include "core/ITriangleMesh.h"

namespace BRICS_3D {

class OSGTriangleMeshVisualizer {
public:

	OSGTriangleMeshVisualizer();

	virtual ~OSGTriangleMeshVisualizer();

	void addTriangleMesh(ITriangleMesh* mesh);

	void visualize();

	osg::Node* createTriangleMeshNode(ITriangleMesh* mesh);


private:

	/**
	 * @brief Create an OSG TriangleMesh from an ITriangleMesh.
	 */
	osg::TriangleMesh* createTriangleMesh(ITriangleMesh* mesh);

	/// OSG viewer object
	osgViewer::Viewer viewer;

	/// Root node for scenegraph
	osg::Group* rootGeode;

};

}

#endif /* OSGTRIANGLEMESHVISUALIZER_H_ */

/* EOF */
