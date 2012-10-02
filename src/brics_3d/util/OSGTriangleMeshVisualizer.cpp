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

#include "OSGTriangleMeshVisualizer.h"
#include "core/TriangleMeshImplicit.h"

#include <osg/ShapeDrawable>
#include <osg/Point>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>

namespace brics_3d {

OSGTriangleMeshVisualizer::OSGTriangleMeshVisualizer() {
	rootGeode = new osg::Group();
	viewer.setSceneData(rootGeode);
	viewer.setUpViewInWindow(10, 10, 500, 500);
	//viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
}

OSGTriangleMeshVisualizer::~OSGTriangleMeshVisualizer() {
//	delete rootGeode;
}

void OSGTriangleMeshVisualizer::addTriangleMesh(ITriangleMesh* mesh) {
	rootGeode->addChild(createTriangleMeshNode(mesh));
}

void OSGTriangleMeshVisualizer::visualize() {
//	osg::Point* point=new osg::Point;
//	point->setSize(2.0f);
//	rootGeode->getOrCreateStateSet()->setAttribute(point);
	viewer.run(); // run();
}


osg::ref_ptr<osg::Node> OSGTriangleMeshVisualizer::createTriangleMeshNode(ITriangleMesh* mesh, float red, float green, float blue, float alpha) {
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::TriangleMesh> osgMesh = createTriangleMesh(mesh);
	osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(osgMesh);

	drawable->setColor(osg::Vec4(red, green, blue, alpha)); // TODO: changeable colors...
	geode->addDrawable(drawable);
	geode->setDataVariance(osg::Object::DYNAMIC);

//	osg::Group* group = new osg::Group;
	osg::ref_ptr<osg::Group> group = new osg::Group;
	group->addChild(geode);

	return group;
}

osg::ref_ptr<osg::TriangleMesh> OSGTriangleMeshVisualizer::createTriangleMesh(ITriangleMesh* mesh) {
	int nVertices;
	int nTriangles;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::IntArray> indices = new osg::IntArray;
	Point3D tmpVertex;

	//this is a smart converter... ;-)
	TriangleMeshImplicit* meshImplicit = dynamic_cast<TriangleMeshImplicit*>(mesh);
	if (meshImplicit != NULL) { //here we exploit knowledge about the implicit triangle representation

		nVertices = meshImplicit->getNumberOfVertices();
		nTriangles = meshImplicit->getSize(); //TODO: correct size
		vertices = new osg::Vec3Array(nVertices);
		indices = new osg::IntArray(nTriangles * 3);

		for (int i=0; i < nVertices; ++i) {
			tmpVertex = (*meshImplicit->getVertices())[i];
			(*vertices)[i] = osg::Vec3(
					//static_cast<value_type>(tmpVertex.getX()),
					tmpVertex.getX(), // TODO: cast ?
					tmpVertex.getY(),
					tmpVertex.getZ());
		}

		int iOld = 0;
		int iNew = 0;
		for (int i=0; i < nTriangles; ++i) {
			(*indices)[iNew++] = (*meshImplicit->getIndices())[iOld++];
			(*indices)[iNew++] = (*meshImplicit->getIndices())[iOld++];
			(*indices)[iNew++] = (*meshImplicit->getIndices())[iOld++];
		}

	} else { // here we go with the generic (possible less efficient) representation

		nTriangles = mesh->getSize();
		nVertices = (nTriangles * 3);
		vertices = new osg::Vec3Array(nVertices);
		indices = new osg::IntArray(nTriangles * 3);

		int indexCount = 0;
		for (int i =0 ; i < nTriangles; ++i) { // loop over all triangles
			for (int j = 0; j <= 2; ++j) { // loop over the three vertices per triangle
				tmpVertex = *mesh->getTriangleVertex(i,j);
						(*vertices)[indexCount] = osg::Vec3(
					//static_cast<value_type>(tmpVertex.getX()),
					tmpVertex.getX(), // TODO: cast?
					tmpVertex.getY(),
					tmpVertex.getZ());

				(*indices)[indexCount] = indexCount; //one-to-one mapping
				indexCount++;
			}
		}
	}

	osg::ref_ptr<osg::TriangleMesh> osgMesh = new osg::TriangleMesh();
//	std::cout << std::endl << *mesh;
	osgMesh->setVertices(vertices);
	osgMesh->setIndices(indices);
	return osgMesh;
}

}

/* EOF */
