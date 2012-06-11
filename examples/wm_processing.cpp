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
#include <worldModel/sceneGraph/Mesh.h>
#include <worldModel/sceneGraph/Box.h>
#include <worldModel/sceneGraph/Cylinder.h>
#include <worldModel/sceneGraph/DotGraphGenerator.h>

#ifdef EIGEN3
#include <algorithm/featureExtraction/BoundingBox3DExtractor.h>
#endif /*EIGEN3*/

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
	LOG(INFO) << "Size of point cloud1 : " << pointCloud1->getSize();
	LOG(INFO) << "Size of point cloud2: " << pointCloud2->getSize();

	/* add data to the world model */
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+---------------------
	 *        |          |          |         |
	 *       pc1        tfPc2     pcReduced	boxROI
	 *       			 |
	 *       		    pc2
	 */
	unsigned int pc1Id;
	unsigned int pc2Id;
	unsigned int pcReducedId;
	unsigned int pcBoxROIId;
	unsigned int pcResultId;
	vector<RSG::Attribute> tmpAttributes;

	WorldModel* wm = new WorldModel();
	RSG::OSGVisualizer* wmObserver = new RSG::OSGVisualizer();
	wm->scene.attachUpdateObserver(wmObserver); //enable visualization

	/* Hook in the point clouds */
	PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pc1Container(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pc1Container->data = pointCloud1;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_1"));
	wm->scene.addGeometricNode(wm->getRootNodeId(), pc1Id, tmpAttributes, pc1Container, TimeStamp(0.0));
	LOG(INFO) << "assigend ID for first point cloud: " << pc1Id;

	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tfPc2(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0.0,0.0,0.0));
	unsigned int tfPc2Id = 0;
	tmpAttributes.clear();
	wm->scene.addTransformNode(wm->getRootNodeId(), tfPc2Id, tmpAttributes, tfPc2, TimeStamp(0.0));
	PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pc2Container(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pc2Container->data = pointCloud2;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_2"));
	wm->scene.addGeometricNode(tfPc2Id, pc2Id, tmpAttributes, pc2Container, TimeStamp(0.0));
	LOG(INFO) << "assigend ID for second point cloud: " << pc2Id;

	/* reduce point cloud with Octree filter */
	Octree* octreeFilter = new Octree();
	octreeFilter->setVoxelSize(0.002); 	//value deduced from roughly knowing the data in advance...
	BRICS_3D::PointCloud3D::PointCloud3DPtr reducedPointCloud(new BRICS_3D::PointCloud3D());

	/* query world model for relevant data */
	vector<unsigned int> resultIds;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_1"));
	wm->scene.getNodes(tmpAttributes, resultIds);
	assert(resultIds.size() == 1);
	pcResultId = resultIds[0];
	LOG(INFO) <<  "Found ID for label point_cloud_1: " << pcResultId;
	Shape::ShapePtr resultShape;
	TimeStamp resultTime;
	wm->scene.getGeometry(pcResultId, resultShape, resultTime);
	RSG::PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pcResultContainer(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pcResultContainer = boost::dynamic_pointer_cast<PointCloud<BRICS_3D::PointCloud3D> >(resultShape);
	assert(pcResultContainer != 0);

//	octreeFilter->filter(pointCloud1.get(), reducedPointCloud);
	octreeFilter->filter(pcResultContainer->data.get(), reducedPointCloud.get());

	/* Add subsampled point cloud to wm */
	PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pcReducedContainer(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pcReducedContainer->data = reducedPointCloud;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_reduced"));
	tmpAttributes.push_back(Attribute("filterType","octree"));
	wm->scene.addGeometricNode(wm->getRootNodeId(), pcReducedId, tmpAttributes, pcReducedContainer, TimeStamp(0.1));

	/* Create a point cloud based on a box ROI */
	BoxROIExtractor boxFilter(0.2,0.015,0.2); // NOTE: each value describes range [origin-value/2, origin+value/2]

	//HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0,0.2,0));
	Eigen::AngleAxis<double> rotation(M_PI_2/4.0, Eigen::Vector3d(0,0,1));
	Transform3d transformation;
	transformation = rotation;
	transformation.translate(Eigen::Vector3d(0,0.1,0));
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44(&transformation));

	boxFilter.setBoxOrigin(transform);
	BRICS_3D::PointCloud3D::PointCloud3DPtr boxROIPointCloud(new BRICS_3D::PointCloud3D());
	boxFilter.filter(pcResultContainer->data.get(), boxROIPointCloud.get());
	LOG(INFO) <<  "ROI has " << boxROIPointCloud->getSize() << " points.";




	/* get a bounding box */
#ifdef EIGEN3
	BRICS_3D::BoundingBox3DExtractor* boundingBoxExtractor = new BRICS_3D::BoundingBox3DExtractor();
	BRICS_3D::Point3D resultBoxCenter;
	BRICS_3D::Vector3D resultBoxDimensions;
	boundingBoxExtractor->computeBoundingBox(pcResultContainer->data.get(), resultBoxCenter, resultBoxDimensions);
	BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr clusterTransform(new BRICS_3D::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, resultBoxCenter.getX(),resultBoxCenter.getY(),resultBoxCenter.getZ()));
	BRICS_3D::RSG::Box::BoxPtr clusterBoundingBox(new BRICS_3D::RSG::Box(resultBoxDimensions.getX(), resultBoxDimensions.getY(), resultBoxDimensions.getZ()));
	LOG(INFO) << "BoundingBox: center = " << resultBoxCenter << " dimensions = " << resultBoxDimensions;

	/* add TF + Box */
	unsigned int tfBBoxId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","cluster_tf"));
	wm->scene.addTransformNode(wm->scene.getRootId(), tfBBoxId, tmpAttributes, clusterTransform, BRICS_3D::RSG::TimeStamp(0.1));

	unsigned int bBoxId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","cluster_bbox"));
	tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	wm->scene.addGeometricNode(tfBBoxId, bBoxId, tmpAttributes, clusterBoundingBox, BRICS_3D::RSG::TimeStamp(0.1));
#endif


	/* Add a transformation node to wm */
//	HomogeneousMatrix44::IHomogeneousMatrix44Ptr offsetTransform(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0,0.001,0));
//	unsigned int tfId = 0;
//	tmpAttributes.clear();
//	tmpAttributes.push_back(Attribute("name","offset_transform"));
//	wm->scene.addTransformNode(wm->getRootNodeId(), tfId, tmpAttributes, offsetTransform, TimeStamp(0.2));

	/* Add ROI point cloud to wm */
	PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pcBoxROINode(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pcBoxROINode->data = boxROIPointCloud;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_box_roi"));
//	wm->scene.addGeometricNode(tfId, pcBoxROIId, tmpAttributes, pcBoxROINode, TimeStamp(0.2));
	wm->scene.addGeometricNode(wm->scene.getRootId(), pcBoxROIId, tmpAttributes, pcBoxROINode, TimeStamp(0.2));


	IterativeClosestPointFactory* icpFactory = new IterativeClosestPointFactory();
	IIterativeClosestPointPtr icp;
	icp = icpFactory->createIterativeClosestPoint(); //take default ICP, otherwise pass a configuration file as parameter
	IIterativeClosestPointSetupPtr icpConfigurator = icpFactory->getIcpSetupHandle();
	icpConfigurator->setMaxIterations(100);
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransformation(new HomogeneousMatrix44());
	icp->match(pointCloud1.get(), pointCloud2.get(), resultTransformation.get()); //TODO: don't touch internal data
	LOG(INFO) << *resultTransformation;
	wm->scene.setTransform(tfPc2Id, resultTransformation, TimeStamp(0.0)); // as we found the transform we can update the wm...


	/*
	 * Some further capabilities of the world model:
	 */
	unsigned int dummyId = 0;
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

	/* add some more shapes and transforms */
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tf1(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0.1,0.1,0));
	unsigned int tfBox1Id = 0;
	wm->scene.addTransformNode(groupId, tfBox1Id, tmpAttributes, transform, dummyTime);
	RSG::Box::BoxPtr box1(new RSG::Box(0.2,0.015,0.2));
	wm->scene.addGeometricNode(tfBox1Id, dummyId, tmpAttributes, box1, dummyTime);

//	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tf2(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0.1,0.05,0));
//	unsigned int tfCylinder1Id = 0;
//	wm->scene.addTransformNode(groupId, tfCylinder1Id, tmpAttributes, tf2, dummyTime);
//	RSG::Cylinder::CylinderPtr cylinder1(new RSG::Cylinder(0.02,0.2));
//	wm->scene.addGeometricNode(tfCylinder1Id, dummyId, tmpAttributes, cylinder1, dummyTime);


//	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tfReference1(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0.1,0.1,0.1));
//	unsigned int tfReference1Id = 0;
//	wm->scene.addTransformNode(wm->scene.getRootId(), tfReference1Id, tmpAttributes, tfReference1, dummyTime);
//	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tfReference2(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, -0.1,0.0,0.0));
//	wm->scene.addTransformNode(tfReference1Id, dummyId, tmpAttributes, tfReference2, dummyTime);

	/* update transform data */
//	wm->scene.setTransform(tfId, transform, dummyTime);


	/* create some mesh */
	BRICS_3D::ITriangleMesh::ITriangleMeshPtr newMesh(new BRICS_3D::TriangleMeshExplicit());
	BRICS_3D::RSG::Mesh<BRICS_3D::ITriangleMesh>::MeshPtr newMeshContainer(new BRICS_3D::RSG::Mesh<BRICS_3D::ITriangleMesh>());
	newMeshContainer->data = newMesh;

	DelaunayTriangulationOSG* meshGenerator = new DelaunayTriangulationOSG();
//	meshGenerator->triangulate(pcReducedContainer->data.get(), newMeshContainer->data.get());
	meshGenerator->triangulate(pc1Container->data.get(), newMeshContainer->data.get());
	LOG(INFO) << "Number of generated triangles: " << newMeshContainer->data->getSize();
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","mesh_1"));
	wm->scene.addGeometricNode(wm->scene.getRootId(), dummyId, tmpAttributes, newMeshContainer, dummyTime);

	BRICS_3D::RSG::DotGraphGenerator dotGraphTraverser;
	wm->scene.executeGraphTraverser(&dotGraphTraverser);
	cout << "GRAPH: "<< endl << dotGraphTraverser.getDotGraph() << endl;

	/* clean up */
	delete octreeFilter;
	delete icpFactory;
	delete meshGenerator;


	while(!wmObserver->done()) { // wait until user closes the GUI
		//nothing here
	}

	delete wmObserver;
	delete wm;

	return 0;
}

/* EOF */
