/**
 * @file 
 * OSGTriangleMeshVisualizer.h
 *
 * @date: Feb 25, 2010
 * @author: sblume
 */

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
