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

/* BRICS_3D includes */
#include <util/DepthImageLoader.h>
#include <util/OSGPointCloudVisualizer.h>
#include <util/OSGTriangleMeshVisualizer.h>
#include <worldModel/sceneGraph/OSGVisualizer.h>
#include <core/PointCloud3D.h>
#include <core/Logger.h>
#include <core/HomogeneousMatrix44.h>
#include <core/TriangleMeshImplicit.h>
#include <core/TriangleMeshExplicit.h>
#include <algorithm/filtering/Octree.h>
#include <algorithm/filtering/BoxROIExtractor.h>
#include <algorithm/registration/IterativeClosestPointFactory.h>
#include <algorithm/depthPerception/DepthImageToPointCloudTransformation.h>
#include <algorithm/meshGeneration/DelaunayTriangulationOSG.h>
#include <worldModel/WorldModel.h>
#include <worldModel/sceneGraph/PointCloud.h>

/* general includes */
#include <iostream>
#include <cstring>

using namespace std;
using namespace BRICS_3D;

int main(int argc, char **argv) {

	string filename1;
	string filename2;

	char defaultFilename[255] = { BRICS_MODELS_DIR };
	strcat(defaultFilename, "/bunny000.txt\0");
	filename1 = defaultFilename;

	char defaultFilename2[255] = { BRICS_MODELS_DIR };
	strcat(defaultFilename2, "/bunny045.txt\0");
	filename2 = defaultFilename2;

	BRICS_3D::Logger::setMinLoglevel(BRICS_3D::Logger::LOGDEBUG);

	/* create the point clouds */
	BRICS_3D::PointCloud3D::PointCloud3DPtr pointCloud1(new BRICS_3D::PointCloud3D());
	BRICS_3D::PointCloud3D::PointCloud3DPtr pointCloud2(new BRICS_3D::PointCloud3D());

	/* load 3d data */
	pointCloud1->readFromTxtFile(filename1);
	pointCloud2->readFromTxtFile(filename2);

	cout << "Size of point cloud1 : " << pointCloud1->getSize() << endl;
	cout << "Size of point cloud2: " << pointCloud2->getSize() << endl;

	/* add data to the world model */
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+---------------------
	 *        |          |          |         |
	 *       pc1        pc2      pcReduced	boxROI
	 */
	unsigned int pc1Id;
	unsigned int pc2Id;
	unsigned int pcReducedId;
	unsigned int pcBoxROIId;
	unsigned int pcResultId;

	WorldModel* wm = new WorldModel();
	RSG::OSGVisualizer* wmObserver = new RSG::OSGVisualizer();
	wm->scene.attachUpdateObserver(wmObserver); //enable visualization


	PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pc1Node(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pc1Node->data = pointCloud1;
	vector<RSG::Attribute> tmpAttributes;
	tmpAttributes.push_back(Attribute("name","point_cloud_1"));
	wm->scene.addGeometricNode(wm->getRootNodeId(), pc1Id, tmpAttributes, pc1Node, TimeStamp(0.0));
	cout << "assigend ID for first point cloud: " << pc1Id << endl;

	/* reduce point cloud with Octree filter */
	Octree* octreeFilter = new Octree();
	octreeFilter->setVoxelSize(0.002); 	//value deduced from roughly knowing the data in advance...
	BRICS_3D::PointCloud3D::PointCloud3DPtr reducedPointCloud(new BRICS_3D::PointCloud3D());

	/* query world model for relevant data */
	vector<unsigned int> resultIds;
	wm->scene.getNodes(tmpAttributes, resultIds);
	assert(resultIds.size() == 1);
	pcResultId = resultIds[0];
	cout << "Found ID for label point_cloud_1: " << pcResultId << endl;
	Shape::ShapePtr resultShape;
	TimeStamp resultTime;
	wm->scene.getGeometry(pcResultId, resultShape, resultTime);
	RSG::PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr resultNode(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	resultNode = boost::dynamic_pointer_cast<PointCloud<BRICS_3D::PointCloud3D> >(resultShape);
	assert(resultNode != 0);

//	octreeFilter->filter(pointCloud1.get(), reducedPointCloud);
	octreeFilter->filter(resultNode->data.get(), reducedPointCloud.get());

	/* Add subsampled point cloud to wm */
	PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pcReducedNode(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pcReducedNode->data = reducedPointCloud;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_reduced"));
	wm->scene.addGeometricNode(wm->getRootNodeId(), pcReducedId, tmpAttributes, pcReducedNode, TimeStamp(0.1));

	/* Create a point cloud based on a box ROI */
	BoxROIExtractor boxFilter(0.2,0.015,0.2); // NOTE: each value describes range [origin-value/2, origin+value/2]

	//HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0,0.2,0));
	Eigen::AngleAxis<double> rotation(M_PI_2/4.0, Eigen::Vector3d(0,0,1));
	Transform3d transformation;
	transformation = rotation;
	transformation.translate(Eigen::Vector3d(0,0.15,0));
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44(&transformation));

	boxFilter.setBoxOrigin(transform);
	BRICS_3D::PointCloud3D::PointCloud3DPtr boxROIPointCloud(new BRICS_3D::PointCloud3D());
	boxFilter.filter(resultNode->data.get(), boxROIPointCloud.get());
	cout << "ROI has " << boxROIPointCloud->getSize() << " points" << endl;

	/* Add a transformation node to wm */
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr offsetTransform(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0,0.001,0));
	unsigned int tfId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","offset_transform"));
	wm->scene.addTransformNode(wm->getRootNodeId(), tfId, tmpAttributes, offsetTransform, TimeStamp(0.2));

	/* Add ROI point cloud to wm */
	PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pcBoxROINode(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pcBoxROINode->data = boxROIPointCloud;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_box_roi"));
	wm->scene.addGeometricNode(tfId, pcBoxROIId, tmpAttributes, pcBoxROINode, TimeStamp(0.2));

	/* optionally perform registration via ICP */
//	if(false) {
//		IterativeClosestPointFactory* icpFactory = new IterativeClosestPointFactory();
//		IIterativeClosestPointPtr icp;
//		icp = icpFactory->createIterativeClosestPoint(); //take default ICP, otherwise pass a configuration file as parameter
//		IHomogeneousMatrix44* resultTransformation = new HomogeneousMatrix44();
//		icp->match(pointCloud1, pointCloud2, resultTransformation);
//
//		/* here we use the streaming functionality to append a point cloud to another */
//		stringstream tmpSteam;
//		tmpSteam << *pointCloud2;
//		tmpSteam >> *pointCloud1;
//	}

	cout << "Size of point cloud: " << pointCloud1->getSize() << endl;
	cout << "Size of point cloud: " << reducedPointCloud->getSize() << endl;


	/*
	 * Some further capabilities of the world model:
	 */
	unsigned int dymmyId = 0;
	unsigned int groupId = 0;
	unsigned int nodeId = 0;
	TimeStamp dummyTime(1.0);
	tmpAttributes.clear();

	/* add an group leaf node */
	wm->scene.addGroup(wm->scene.getRootId(), groupId, tmpAttributes);

	/* add an empty leafe node (child of ) */
	wm->scene.addNode(groupId, nodeId, tmpAttributes);

	/* delete the same one */
	wm->scene.deleteNode(nodeId);
//	wm->scene.deleteNode(pcReducedId);

	/* add some more shapes */

	/* update transform data */
	wm->scene.setTransform(tfId, transform, dummyTime);


//	/* create mesh */
//	ITriangleMesh* mesh = new TriangleMeshExplicit();
//	DelaunayTriangulationOSG* meshGenerator = new DelaunayTriangulationOSG();
//	meshGenerator->triangulate(pointCloud1, mesh);
//	cout << "Number of generated triangles: " << mesh->getSize() << endl;

	/* clean up */
//	delete meshGenerator;


	while(!wmObserver->done()) { // wait until user closes the GUI
		//nothing here
	}

	delete wmObserver;
	delete wm;

	return 0;
}

/* EOF */
