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
#include <brics_3d/util/OSGPointCloudVisualizer.h>
#include <brics_3d/util/OSGTriangleMeshVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/TriangleMeshImplicit.h>
#include <brics_3d/core/TriangleMeshExplicit.h>
#include <brics_3d/algorithm/filtering/Octree.h>
#include <brics_3d/algorithm/filtering/BoxROIExtractor.h>
#include <brics_3d/algorithm/registration/IterativeClosestPointFactory.h>
#include <brics_3d/algorithm/meshGeneration/DelaunayTriangulationOSG.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/PointCloud.h>
#include <brics_3d/worldModel/sceneGraph/Mesh.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/Cylinder.h>
#include <brics_3d/worldModel/sceneGraph/DotGraphGenerator.h>
#include <brics_3d/worldModel/sceneGraph/PointCloudAccumulatorIdAware.h>

#ifdef EIGEN3
#include <brics_3d/algorithm/featureExtraction/BoundingBox3DExtractor.h>
#endif /*EIGEN3*/

/* general includes */
#include <iostream>
#include <cstring>

using namespace std;
using namespace brics_3d;
using namespace brics_3d::rsg;

int main(int argc, char **argv) {

	string filename1;
	string filename2;

	char defaultFilename[255] = { BRICS_MODELS_DIR };
	strcat(defaultFilename, "/bunny000.txt\0");
	filename1 = defaultFilename;

	char defaultFilename2[255] = { BRICS_MODELS_DIR };
	strcat(defaultFilename2, "/bunny045.txt\0");
	filename2 = defaultFilename2;

	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	/* create the point clouds */
	brics_3d::PointCloud3D::PointCloud3DPtr pointCloud1(new brics_3d::PointCloud3D());
	brics_3d::PointCloud3D::PointCloud3DPtr pointCloud2(new brics_3d::PointCloud3D());

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
	rsg::Id pc1Id;
	rsg::Id pc2Id = 41; // just fot now
	rsg::Id pcReducedId;
	rsg::Id groupReducedClouds;
	rsg::Id pcBoxROIId;
	rsg::Id pcResultId;
//	rsg::Id tmpId;
	vector<rsg::Attribute> tmpAttributes;

	WorldModel* wm = new WorldModel();
	rsg::OSGVisualizer* wmObserver = new rsg::OSGVisualizer();
	rsg::VisualizationConfiguration osgConfiguration; // optional configuration
	osgConfiguration.visualizeIds = false;
	osgConfiguration.visualizeAttributes = false;
	wmObserver->setConfig(osgConfiguration);
	wm->scene.attachUpdateObserver(wmObserver); //enable visualization
	wm->scene.advertiseRootNode(); // Don't forget this one! Otherwise the observers cannot correctly handle the updates.

	/* Hook in the point clouds */
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pc1Container(new rsg::PointCloud<brics_3d::PointCloud3D>());
	pc1Container->data = pointCloud1;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_1"));
	wm->scene.addGeometricNode(wm->getRootNodeId(), pc1Id, tmpAttributes, pc1Container, TimeStamp(0.0));
	LOG(INFO) << "assigend ID for first point cloud: " << pc1Id;

	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tfPc2(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0.0,0.0,0.0));
	rsg::Id tfPc2Id = 0;
	tmpAttributes.clear();
	wm->scene.addTransformNode(wm->getRootNodeId(), tfPc2Id, tmpAttributes, tfPc2, TimeStamp(0.0));
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pc2Container(new rsg::PointCloud<brics_3d::PointCloud3D>());
	pc2Container->data = pointCloud2;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_2"));
	wm->scene.addGeometricNode(tfPc2Id, pc2Id, tmpAttributes, pc2Container, TimeStamp(0.0), true); //force - just for now
	LOG(INFO) << "assigend ID for second point cloud: " << pc2Id;

	/* reduce point cloud with Octree filter */
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","reduced_clouds"));
	wm->scene.addGroup(wm->scene.getRootId(), groupReducedClouds, tmpAttributes);
	Octree* octreeFilter = new Octree();
	octreeFilter->setVoxelSize(0.002); 	//value deduced from roughly knowing the data in advance...
	brics_3d::PointCloud3D::PointCloud3DPtr reducedPointCloud(new brics_3d::PointCloud3D());
	brics_3d::PointCloud3D::PointCloud3DPtr reducedPointCloud2(new brics_3d::PointCloud3D());

	/* query world model for relevant data */
	vector<rsg::Id> resultIds;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_1"));
	wm->scene.getNodes(tmpAttributes, resultIds);
	assert(resultIds.size() == 1);
	pcResultId = resultIds[0];
	LOG(INFO) <<  "Found ID for label point_cloud_1: " << pcResultId;
	Shape::ShapePtr resultShape;
	TimeStamp resultTime;
	wm->scene.getGeometry(pcResultId, resultShape, resultTime);
	rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pcResultContainer(new rsg::PointCloud<brics_3d::PointCloud3D>());
	pcResultContainer = boost::dynamic_pointer_cast<PointCloud<brics_3d::PointCloud3D> >(resultShape);
	assert(pcResultContainer != 0);

//	octreeFilter->filter(pointCloud1.get(), reducedPointCloud);
	octreeFilter->filter(pcResultContainer->data.get(), reducedPointCloud.get());

	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_2"));
	wm->scene.getNodes(tmpAttributes, resultIds);
	assert(resultIds.size() == 1);
	pcResultId = resultIds[0];
	LOG(INFO) <<  "Found ID for label point_cloud_2: " << pcResultId;
	wm->scene.getGeometry(pcResultId, resultShape, resultTime);
	pcResultContainer = boost::dynamic_pointer_cast<PointCloud<brics_3d::PointCloud3D> >(resultShape);
	assert(pcResultContainer != 0);

	octreeFilter->filter(pcResultContainer->data.get(), reducedPointCloud2.get());


	/* Add subsampled point clouds to wm */
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pcReducedContainer(new rsg::PointCloud<brics_3d::PointCloud3D>());
	pcReducedContainer->data = reducedPointCloud;
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pcReducedContainer2(new rsg::PointCloud<brics_3d::PointCloud3D>());
	pcReducedContainer2->data = reducedPointCloud2;

	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_reduced"));
	tmpAttributes.push_back(Attribute("filterType","octree"));
	wm->scene.addGeometricNode(groupReducedClouds, pcReducedId, tmpAttributes, pcReducedContainer, TimeStamp(0.1));

	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_reduced2"));
	tmpAttributes.push_back(Attribute("filterType","octree"));
	wm->scene.addGeometricNode(groupReducedClouds, pcReducedId, tmpAttributes, pcReducedContainer2, TimeStamp(0.1));


	/* Create a point cloud based on a box ROI */
	BoxROIExtractor boxFilter(0.2,0.015,0.2); // NOTE: each value describes range [origin-value/2, origin+value/2]

	//HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0,0.2,0));
	Eigen::AngleAxis<double> rotation(M_PI_2/4.0, Eigen::Vector3d(0,0,1));
	Transform3d transformation;
	transformation = rotation;
	transformation.translate(Eigen::Vector3d(0,0.1,0));
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44(&transformation));

	boxFilter.setBoxOrigin(transform);
	brics_3d::PointCloud3D::PointCloud3DPtr boxROIPointCloud(new brics_3d::PointCloud3D());
	boxFilter.filter(pcResultContainer->data.get(), boxROIPointCloud.get());
	LOG(INFO) <<  "ROI has " << boxROIPointCloud->getSize() << " points.";




	/* get a bounding box */
#ifdef EIGEN3
	brics_3d::BoundingBox3DExtractor* boundingBoxExtractor = new brics_3d::BoundingBox3DExtractor();
	brics_3d::Point3D resultBoxCenter;
	brics_3d::Vector3D resultBoxDimensions;
	boundingBoxExtractor->computeBoundingBox(pcResultContainer->data.get(), resultBoxCenter, resultBoxDimensions);
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr clusterTransform(new brics_3d::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, resultBoxCenter.getX(),resultBoxCenter.getY(),resultBoxCenter.getZ()));
	brics_3d::rsg::Box::BoxPtr clusterBoundingBox(new brics_3d::rsg::Box(resultBoxDimensions.getX(), resultBoxDimensions.getY(), resultBoxDimensions.getZ()));
	LOG(INFO) << "BoundingBox: center = " << resultBoxCenter << " dimensions = " << resultBoxDimensions;

	/* add TF + Box */
	rsg::Id tfBBoxId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","cluster_tf"));
	wm->scene.addTransformNode(wm->scene.getRootId(), tfBBoxId, tmpAttributes, clusterTransform, brics_3d::rsg::TimeStamp(0.1));

	rsg::Id bBoxId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","cluster_bbox"));
	tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	wm->scene.addGeometricNode(tfBBoxId, bBoxId, tmpAttributes, clusterBoundingBox, brics_3d::rsg::TimeStamp(0.1));
#endif


	/* Add a transformation node to wm */
//	HomogeneousMatrix44::IHomogeneousMatrix44Ptr offsetTransform(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0,0.001,0));
//	rsg::Id tfId = 0;
//	tmpAttributes.clear();
//	tmpAttributes.push_back(Attribute("name","offset_transform"));
//	wm->scene.addTransformNode(wm->getRootNodeId(), tfId, tmpAttributes, offsetTransform, TimeStamp(0.2));

	/* Add ROI point cloud to wm */
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pcBoxROINode(new rsg::PointCloud<brics_3d::PointCloud3D>());
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
	rsg::Id dummyId = 0;
	rsg::Id groupId = 0;
	rsg::Id nodeId = 0;
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
	rsg::Id tfBox1Id = 0;
	wm->scene.addTransformNode(groupId, tfBox1Id, tmpAttributes, transform, dummyTime);
	rsg::Box::BoxPtr box1(new rsg::Box(0.2,0.015,0.2));
	wm->scene.addGeometricNode(tfBox1Id, dummyId, tmpAttributes, box1, dummyTime);
	rsg::Sphere::SpherePtr sphere1(new rsg::Sphere(0.02, Units::Meter));
	wm->scene.addGeometricNode(tfBox1Id, dummyId, tmpAttributes, sphere1, dummyTime);

//	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tf2(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0.1,0.05,0));
//	rsg::Id tfCylinder1Id = 0;
//	wm->scene.addTransformNode(groupId, tfCylinder1Id, tmpAttributes, tf2, dummyTime);
//	rsg::Cylinder::CylinderPtr cylinder1(new rsg::Cylinder(0.02,0.2));
//	wm->scene.addGeometricNode(tfCylinder1Id, dummyId, tmpAttributes, cylinder1, dummyTime);


//	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tfReference1(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0.1,0.1,0.1));
//	rsg::Id tfReference1Id = 0;
//	wm->scene.addTransformNode(wm->scene.getRootId(), tfReference1Id, tmpAttributes, tfReference1, dummyTime);
//	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tfReference2(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, -0.1,0.0,0.0));
//	wm->scene.addTransformNode(tfReference1Id, dummyId, tmpAttributes, tfReference2, dummyTime);

	/* update transform data */
//	wm->scene.setTransform(tfId, transform, dummyTime);

	/* get an  aggregated point cloud based on the filtered point clouds */
	brics_3d::PointCloud3D::PointCloud3DPtr aggregatedPointCloud(new brics_3d::PointCloud3D());
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr aggregatedPointCloudContainer(new rsg::PointCloud<brics_3d::PointCloud3D>());
	aggregatedPointCloudContainer->data = aggregatedPointCloud;

	PointCloudAccumulatorIdAware* pcAccumulator = new PointCloudAccumulatorIdAware(&wm->scene, wm->scene.getRootId());
	wm->scene.executeGraphTraverser(pcAccumulator, groupReducedClouds);


	IPoint3DIterator* it = pcAccumulator->getAccumulatedPointClouds();
	for (it->begin(); !it->end(); it->next()) {
		aggregatedPointCloudContainer->data->addPoint(Point3D(it->getX(), it->getY(), it->getZ())); //we ignore potential decoration layers.
	}

	/* create some mesh */
	brics_3d::ITriangleMesh::ITriangleMeshPtr newMesh(new brics_3d::TriangleMeshExplicit());
	brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr newMeshContainer(new brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>());
	newMeshContainer->data = newMesh;

	DelaunayTriangulationOSG* meshGenerator = new DelaunayTriangulationOSG();
//	meshGenerator->triangulate(pcReducedContainer->data.get(), newMeshContainer->data.get());
//	meshGenerator->triangulate(pc1Container->data.get(), newMeshContainer->data.get());
	meshGenerator->triangulate(aggregatedPointCloudContainer->data.get(), newMeshContainer->data.get());
	LOG(INFO) << "Number of generated triangles: " << newMeshContainer->data->getSize();
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","mesh_1"));
	wm->scene.addGeometricNode(wm->scene.getRootId(), dummyId, tmpAttributes, newMeshContainer, dummyTime);



	brics_3d::rsg::DotGraphGenerator dotGraphTraverser;
	wm->scene.executeGraphTraverser(&dotGraphTraverser, wm->scene.getRootId());
	cout << "GRAPH: "<< endl << dotGraphTraverser.getDotGraph() << endl;


	/* processing with function blocks */
	string blockName = "roifilter";//"testblock";
	string blockPath = "/home/sblume/sandbox/brics_3d_function_blocks/lib/";

	wm->loadFunctionBlock(blockName, blockPath);

	vector<brics_3d::rsg::Id> input;
	input.push_back(wm->getRootNodeId()); // output hook
	input.push_back(pc2Id); // input  hook
	vector<brics_3d::rsg::Id> output;
	wm->executeFunctionBlock(blockName, input, output);

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
