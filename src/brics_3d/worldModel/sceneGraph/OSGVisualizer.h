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

#ifndef RSG_OSGVISUALIZER_H_
#define RSG_OSGVISUALIZER_H_

#include "ISceneGraphUpdateObserver.h"
#include "brics_3d/worldModel/sceneGraph/PointCloud.h"
#include "brics_3d/worldModel/sceneGraph/Mesh.h"
#include "brics_3d/worldModel/sceneGraph/Box.h"
#include "brics_3d/worldModel/sceneGraph/Cylinder.h"
#include "brics_3d/worldModel/sceneGraph/Attribute.h"
#include "brics_3d/util/OSGPointCloudVisualizer.h" //unfortunately libbrics3d_world_model depends now on libbrics3d_util
#include "brics_3d/util/OSGTriangleMeshVisualizer.h"

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>
#include <map>

namespace brics_3d {

namespace rsg {


/**
 * @brief Displays a robot scene graph via OSG
 * @ingroup sceneGraph
 * @ingroup visualization
 */
class OSGVisualizer : public ISceneGraphUpdateObserver {
public:
	OSGVisualizer();
	virtual ~OSGVisualizer();

	/* implemetntations of observer interface */
	bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false);
    bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId = false);
	bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false);
	bool setNodeAttributes(Id id, vector<Attribute> newAttributes);
	bool setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
    bool setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp);
	bool deleteNode(Id id);
	bool addParent(Id id, Id parentId);
    bool removeParent(Id id, Id parentId);

    /**
     * Return true if  viewer's work is done and should exit the frame loop.
     */
    bool done();


private:

    ///Internal initialization
	void init();

	/// Viewer will get its own thread such that visialization does not block the main program.
	void threadFunction(OSGVisualizer* obj);

	/// get intranl root ID
    Id getRootId();

    /// map IDs to real references
    osg::ref_ptr<osg::Node> findNodeRecerence(Id id);

    /// Helper function to visualize coordinate frame
    osg::ref_ptr<osg::Node> createFrameAxis(double axisLength = 1.0);

    /// Helper function to visualize uncertainty of a frame
    osg::ref_ptr<osg::Node> createUncertaintyVisualization(double radiusX, double radiusY, double radiusZ);

    /// Helper function to visualize attributes as text
    osg::ref_ptr<osg::Node> createAttributeVisualization(vector<Attribute> attributes, Id id = 0);

	/// OSG viewer object
	osgViewer::Viewer viewer;

	/// Root node for scenegraph
	osg::ref_ptr<osg::Group> rootGeode;

	/// Thread handle for visualization thread
	boost::thread* thread;

	///Grant helper classes full access
	friend class OSGOperationAdd;
	friend class OSGOperationRemove;
	friend class OSGOperationUpdateTransform;

	///ID management for OSG
    std::map<Id, osg::ref_ptr<osg::Node> > idLookUpTable;
    std::map<Id, osg::ref_ptr<osg::Node> >::const_iterator nodeIterator;

    /// Determines dimensions of visualizazion of the frames axis. Default is 1.0
    double frameAxisVisualisationScale;


};

}  // namespace rsg

}  // namespace brics_3d

#endif /* RSG_OSGVISUALIZER_H_ */

/* EOF */
