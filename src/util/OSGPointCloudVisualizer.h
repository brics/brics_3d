/**
 * @file 
 * OSGPointCloudVisualizer.h
 *
 * @date: Nov 23, 2009
 * @author: sblume
 */

#ifndef OSGPOINTCLOUDVISUALIZER_H_
#define OSGPOINTCLOUDVISUALIZER_H_

#include <osg/Group>
#include <osg/Geometry>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>

#include "core/PointCloud3D.h"


namespace BRICS_3D {

/**
 * @class OSGPointCloudVisualizer
 * @brief Displays point clouds in an Open Scene Graph (OSG) viewer
 */
class OSGPointCloudVisualizer {
public:

	/**
	 * @brief Standard constructor
	 */
	OSGPointCloudVisualizer();

	/**
	 * @brief Standard destructor
	 */
	virtual ~OSGPointCloudVisualizer();

	/**
	 * @brief Visualizes a point cloud within the OSG framework
	 * @param[in] pointCloud Pointer to point cloud that will visualized
	 */
	void visualizePointCloud(PointCloud3D* pointCloud);

	/**
	 * @brief Creates a "geode" (geometric node) element for OSG out of a point cloud
	 * @param[in] pointCloud Pointer to point cloud that will be transformed into an OSG geode
	 */
	osg::Node* createPointCloudNode(PointCloud3D* pointCloud);

private:

/// OSG viewer object
osgViewer::Viewer viewer;

};

}

#endif /* OSGPOINTCLOUDVISUALIZER_H_ */

/* EOF */
