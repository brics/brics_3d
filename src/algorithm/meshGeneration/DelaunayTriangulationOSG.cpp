/**
 * @file 
 * DelaunayTriangulationOSG.cpp
 *
 * @date: Feb 26, 2010
 * @author: sblume
 */

#include "DelaunayTriangulationOSG.h"
#include "core/TriangleMeshImplicit.h"

#include <osg/Group>
#include <osg/Geometry>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgUtil/EdgeCollector>
#include <osgUtil/DelaunayTriangulator>

#include <boost/foreach.hpp>

namespace BRICS_3D {

DelaunayTriangulationOSG::DelaunayTriangulationOSG() {


}

DelaunayTriangulationOSG::~DelaunayTriangulationOSG() {

}

void DelaunayTriangulationOSG::triangulate(PointCloud3D* pointCloud, ITriangleMesh* mesh, axis ignore) {

//	if (mesh != 0) {
//		delete mesh;
//	}
//	mesh = new TriangleMeshImplicit();

	osg::Vec3Array* points = new osg::Vec3Array;
	for(unsigned int i = 0; i < pointCloud->getSize(); ++i) {
		points->push_back(osg::Vec3((float) ((*pointCloud->getPointCloud())[i].getX()),
				(float) ((*pointCloud->getPointCloud())[i].getY()),
				(float) ((*pointCloud->getPointCloud())[i].getZ()))); //NOTE: possible BUG fixed here
	}

	/* create triangulator and set the points as the area */
	osg::ref_ptr<osgUtil::DelaunayTriangulator> trig = new osgUtil::DelaunayTriangulator();
	trig->setInputPointArray(points);

	/* create a triangular constraint */
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

	/* add constraint to triangulator */
	//trig->addInputConstraint(constraint.get());

	trig->triangulate();

	/* remove triangles from mesh */
	//trig->removeInternalTriangles(constraint.get());


//	osg::Drawable  trig->getTriangles();

	osg::Geometry* gm = new osg::Geometry; //TODO better access of triangles
	gm->setVertexArray(points);
	gm->addPrimitiveSet(trig->getTriangles());
	osg::Vec4Array* colors = new osg::Vec4Array(1);
	colors->push_back(osg::Vec4(0,1,0,0.3));
	gm->setColorArray(colors);
	gm->setColorBinding(osg::Geometry::BIND_OVERALL);

	/* use EdgeCollector to re-extract triangles from mesh */
	osgUtil::EdgeCollector ec;
	ec.setGeometry(gm->asGeometry());
//	std::iterator<osgUtil::EdgeCollector> iter = ec._triangleSet.iterator;

	/* loop through each triangle and print its vertex values */
//	osg::ref_ptr<osgUtil::EdgeCollector::Triangle> triangle;
//	for (int i = 0; i < 3; ++i) {
//		triangle = (*ec._triangleSet)[i];
//		std::cout << "TRIANGLE Points: ";
//		std::cout << triangle->_p1->_vertex<< ", ";
//		std::cout << triangle->_p2->_vertex<< ", ";
//		std::cout << triangle->_p3->_vertex << "\n";
//	}
	Point3D tmpVertex1;
	Point3D tmpVertex2;
	Point3D tmpVertex3;
	BOOST_FOREACH(osg::ref_ptr<osgUtil::EdgeCollector::Triangle> tri, ec._triangleSet) {
//	   std::cout << "TRIANGLE Points: ";
//	   std::cout << tri->_p1->_vertex._v[0] << " ";
//	   std::cout << tri->_p1->_vertex._v[1] << " ";
//	   std::cout << tri->_p1->_vertex._v[2] << ", ";
	   tmpVertex1 = Point3D(tri->_p1->_vertex._v[0], tri->_p1->_vertex._v[1], tri->_p1->_vertex._v[2]);

//	   std::cout << tri->_p2->_vertex._v[0] << " ";
//	   std::cout << tri->_p2->_vertex._v[1] << " ";
//	   std::cout << tri->_p2->_vertex._v[2] << ", ";
	   tmpVertex2 = Point3D(tri->_p2->_vertex._v[0], tri->_p2->_vertex._v[1], tri->_p2->_vertex._v[2]);

//	   std::cout << tri->_p3->_vertex._v[0] << " ";
//	   std::cout << tri->_p3->_vertex._v[1] << " ";
//	   std::cout << tri->_p3->_vertex._v[2] << " ";
	   tmpVertex3 = Point3D(tri->_p3->_vertex._v[0], tri->_p3->_vertex._v[1], tri->_p3->_vertex._v[2]);
//	   std::cout << tri->_p1->_vertex << ", ";
//	   std::cout << tri->_p2->_vertex << ", ";
//	   std::cout << tri->_p3->_vertex << "\n";
//	   std::cout << std::endl;

	   mesh->addTriangle(tmpVertex1, tmpVertex2, tmpVertex3);
	}
}

}

/* EOF */
