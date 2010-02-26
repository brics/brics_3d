/**
 * @file 
 * osg_delaunay.cpp
 *
 * @date: Nov 23, 2009
 * @author: sblume
 */

#include <util/DepthImageLoader.h>
#include <util/OSGPointCloudVisualizer.h>
#include <core/PointCloud3D.h>
#include <algorithm/DepthImageToPointCloudTransformation.h>



#include <osg/Group>
#include <osg/Geometry>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>

#include <osgUtil/DelaunayTriangulator>


#include <iostream>
#include <cstring>

using namespace std;
using namespace BRICS_3D;



int main(int argc, char **argv) {

	/* check argument */
	string filename;
	if (argc == 1) {
		cout << "Usage: " << argv[0] << " <filename>" << endl;

		char defaultFilename[255] = { BRICS_IMGAGES_DIR };
		strcat(defaultFilename, "/zcam_param1c.pgm\0");
		filename = defaultFilename;

		cout << "Trying to get default file: " << filename << endl;
	} else if (argc == 2) {
		filename = argv[1];
		cout << filename << endl;
	} else {
		cerr << "Usage: " << argv[0] << " <filename>" << endl;
		return -1;
	}

	/* get depth image*/
	IplImage* depthImage;
	DepthImageLoader *depthImgageLoader = new DepthImageLoader();
	depthImage = depthImgageLoader->loadDepthImage(filename);
	depthImgageLoader->displayDepthImage();

	/* convert to point cloud */
	PointCloud3D *pointCloud = new PointCloud3D();
	DepthImageToPointCloudTransformation *img2cloudTramsformer = new DepthImageToPointCloudTransformation();
	img2cloudTramsformer->transformDepthImageToPointCloud(depthImage, pointCloud, 0);
	cout << "Size of point cloud: " << pointCloud->getSize() << endl;

	/* visualize point cloud */
	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
	//visualizer->visualizePointCloud(pointCloud);

	// create a square area
	osg::Vec3Array* points = new osg::Vec3Array;
//	points->push_back(osg::Vec3(-1, -1, 0));
//	points->push_back(osg::Vec3(-1,  1, 0));
//	points->push_back(osg::Vec3( 1, -1, 0));

//	points->push_back(osg::Vec3( 0, 0, 0));
//	points->push_back(osg::Vec3( 0, 1, 0));
//	points->push_back(osg::Vec3( 1, 0, 0));
//	points->push_back(osg::Vec3( 1, 0, 0));
//	points->push_back(osg::Vec3( 0, 0, 2));
//	points->push_back(osg::Vec3( 0, 1, 2));
//	points->push_back(osg::Vec3( 1, 0, 2));



	for(unsigned int i = 0; i < pointCloud->getSize(); ++i) {
		points->push_back(osg::Vec3((float) ((*pointCloud->getPointCloud())[i].getX()),
				(float) ((*pointCloud->getPointCloud())[i].getY()),
				(float) ((*pointCloud->getPointCloud())[i].getZ()))); //NOTE: possible BUG fixed here
	}

	// create triangulator and set the points as the area
	osg::ref_ptr<osgUtil::DelaunayTriangulator> trig = new osgUtil::DelaunayTriangulator();
	trig->setInputPointArray(points);

	// create a triangular constraint
	osg::Vec3Array* bounds = new osg::Vec3Array;
//	bounds->push_back(osg::Vec3(-0.5f, -0.5f, 0));
//	bounds->push_back(osg::Vec3(-0.5f, 0.5f, 0));
//	bounds->push_back(osg::Vec3(0.5f, 0.5f, 0));
	bounds->push_back(osg::Vec3(-10.5f, -10.5f, 11.0));
	bounds->push_back(osg::Vec3(-10.5f, 10.5f, 10.0));
	bounds->push_back(osg::Vec3(10.5f, 10.5f, 0));
//	bounds->push_back(osg::Vec3(0.5f, 0.5f, 0));
	osg::ref_ptr<osgUtil::DelaunayConstraint> constraint = new osgUtil::DelaunayConstraint;
	constraint->setVertexArray(bounds);
	constraint->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,3) );

	// add constraint to triangulator
	trig->addInputConstraint(constraint.get());

	trig->triangulate();

	// remove triangle from mesh
	//trig->removeInternalTriangles(constraint.get());

	// put triangulated mesh into OSG geometry
	osg::Geometry* gm = new osg::Geometry;
	gm->setVertexArray(points);
	gm->addPrimitiveSet(trig->getTriangles());
	osg::Vec4Array* colors = new osg::Vec4Array(1);
	colors->push_back(osg::Vec4(0,1,0,0.3));
	gm->setColorArray(colors);
	gm->setColorBinding(osg::Geometry::BIND_OVERALL);

	//create geometry and add it to scene graph
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(gm);


	osg::Group* group = new osg::Group;
	group->addChild(visualizer->createPointCloudNode(pointCloud));
	group->addChild(geode);

	osgViewer::Viewer viewer;
	viewer.setSceneData(group);
	viewer.run();
	//viewer.setSceneData(visualizer->createPointCloudNode(pointCloud));


//	GetMatrixNode()->addChild(geode);
//
//	// use EdgeCollector to re-extract triangles from mesh
//	osgUtil::EdgeCollector ec;
//	ec.setGeometry(gm->asGeometry());

	// loop through each triangle and print its vertex values
//	BOOST_FOREACH(osg::ref_ptr<osgUtil::EdgeCollector::Triangle> tri, ec._triangleSet) {
//	   std::cout << "TRIANGLE Points: ";
//	   std::cout << tri->_p1->_vertex<< ", ";
//	   std::cout << tri->_p2->_vertex<< ", ";
//	   std::cout << tri->_p3->_vertex << "\n";
//	}

	return 0;
}

/* EOF */
