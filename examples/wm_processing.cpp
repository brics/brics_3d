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
#include <core/PointCloud3D.h>
#include <core/HomogeneousMatrix44.h>
#include <core/TriangleMeshImplicit.h>
#include <core/TriangleMeshExplicit.h>
#include <algorithm/filtering/Octree.h>
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

	/* create the point cloud containers */
//	PointCloud3D* pointCloud1 = new PointCloud3D();
	BRICS_3D::PointCloud3D::PointCloud3DPtr pointCloud1(new BRICS_3D::PointCloud3D());
//	PointCloud3D* pointCloud2 = new PointCloud3D();
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
	 *        -----------+----------
	 *        |          |          |
	 *       pc1        pc2      pcReduced
	 */
	unsigned int pc1Id;
	unsigned int pc2Id;
	unsigned int pcReduced;

	WorldModel* wm = new WorldModel();



//	Shape::ShapePtr pointCloud1Shape(new);
	PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pc1Node(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pc1Node->data = pointCloud1;
	vector<RSG::Attribute> tmpAttibutes;
	tmpAttibutes.push_back(Attribute("name","point_cloud_1"));
	wm->scene.addGeometricNode(wm->getRootNodeId(), pc1Id, tmpAttibutes, pc1Node, TimeStamp(0.0));
	cout << "assigend ID for first point cloud: " << pc1Id << endl;

	/* reduce point cloud with Octree filter */
	Octree* octreeFilter = new Octree();
	octreeFilter->setVoxelSize(0.002); 	//value deduced from roughly knowing the data in advance...
	PointCloud3D* reducedPointCloud = new PointCloud3D();

	vector<unsigned int> resultIds;
	wm->scene.getNodes(tmpAttibutes, resultIds);
	assert(resultIds.size() == 1);
	pcReduced = resultIds[0];
	cout << "Found ID for label point_cloud_1: " << pcReduced << endl;
	Shape::ShapePtr resultShape;
	TimeStamp resultTime;
	wm->scene.getGeometry(pcReduced, resultShape, resultTime);
	RSG::PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr reducedNode(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	reducedNode = boost::dynamic_pointer_cast<PointCloud<BRICS_3D::PointCloud3D> >(resultShape);
	assert(reducedNode != 0);

//	octreeFilter->filter(pointCloud1.get(), reducedPointCloud);
	octreeFilter->filter(reducedNode->data.get(), reducedPointCloud);

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

	/* visualize the point cloud */
	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
	visualizer->addPointCloud(reducedPointCloud, 1, 0.1, 0.1, 1);
	visualizer->visualizePointCloud(pointCloud1.get());


//	/* create mesh */
//	ITriangleMesh* mesh = new TriangleMeshExplicit();
//	DelaunayTriangulationOSG* meshGenerator = new DelaunayTriangulationOSG();
//	meshGenerator->triangulate(pointCloud1, mesh);
//	cout << "Number of generated triangles: " << mesh->getSize() << endl;
//
//	/* visualize mesh */
//	OSGTriangleMeshVisualizer* meshVisualizer = new OSGTriangleMeshVisualizer();
//	meshVisualizer->addTriangleMesh(mesh);
//	meshVisualizer->visualize();

	/* clean up */
//	delete meshVisualizer;
//	delete meshGenerator;
	delete visualizer;
//	delete pointCloud1;
//	delete pointCloud2;

	return 0;
}

/* EOF */
