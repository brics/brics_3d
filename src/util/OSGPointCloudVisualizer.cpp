/**
 * @file 
 * OSGPointCloudVisualizer.cpp
 *
 * @date: Nov 23, 2009
 * @author: sblume
 */

#ifdef BRICS_OSG_ENABLE

#include "OSGPointCloudVisualizer.h"
#include <osg/Point>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>

namespace BRICS_3D {

OSGPointCloudVisualizer::OSGPointCloudVisualizer() {
	rootGeode = new osg::Group();
	viewer.setSceneData(rootGeode);
	viewer.setUpViewInWindow(10, 10, 500, 500);
	//viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
}

OSGPointCloudVisualizer::~OSGPointCloudVisualizer() {

}

struct DrawCallback: public osg::Drawable::DrawCallback {

	DrawCallback() :
		_firstTime(true) {
	}

	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const {
		osg::State& state = *renderInfo.getState();

		if (!_firstTime) {
			_previousModelViewMatrix = _currentModelViewMatrix;
			_currentModelViewMatrix = state.getModelViewMatrix();
			_inverseModelViewMatrix.invert(_currentModelViewMatrix);

			osg::Matrix T(_previousModelViewMatrix * _inverseModelViewMatrix);

			osg::Geometry * geometry = dynamic_cast<osg::Geometry*> (const_cast<osg::Drawable*> (drawable));
			osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*> (geometry->getVertexArray());
			for (unsigned int i = 0; i + 1 < vertices->size(); i += 2) {
				(*vertices)[i + 1] = (*vertices)[i] * T;
			}
		} else {
			_currentModelViewMatrix = state.getModelViewMatrix();
		}

		_firstTime = false;

		drawable->drawImplementation(renderInfo);
	}

	mutable bool _firstTime;
	mutable osg::Matrix _currentModelViewMatrix;
	mutable osg::Matrix _inverseModelViewMatrix;
	mutable osg::Matrix _previousModelViewMatrix;
};

void OSGPointCloudVisualizer::addPointCloud(PointCloud3D* pointCloud, float red, float green, float blue, float alpha) {
	rootGeode->addChild(createPointCloudNode(pointCloud, red, green, blue, alpha));
}

void OSGPointCloudVisualizer::addColoredPointCloud(ColoredPointCloud3D* pointCloud, float alpha) {
	rootGeode->addChild(createColoredPointCloudNode(pointCloud, alpha));
}

void OSGPointCloudVisualizer::visualizePointCloud(PointCloud3D *pointCloud, float red, float green, float blue, float alpha)
{
	osg::Point* point=new osg::Point;
	point->setSize(2.0f);
	rootGeode->getOrCreateStateSet()->setAttribute(point);

	rootGeode->addChild(createPointCloudNode(pointCloud, red, green, blue, alpha));
	viewer.run(); // run();
}

void OSGPointCloudVisualizer::visualizeColoredPointCloud(ColoredPointCloud3D *pointCloud, float alpha)
{
	osg::Point* point=new osg::Point;
	point->setSize(2.0f);
	rootGeode->getOrCreateStateSet()->setAttribute(point);

	rootGeode->addChild(createColoredPointCloudNode(pointCloud, alpha));
	viewer.run(); // run();
}


osg::Node* OSGPointCloudVisualizer::createPointCloudNode(PointCloud3D* pointCloud, float red, float green, float blue, float alpha) {

	unsigned int targetNumVertices = 10000; //maximal points per geode

	osg::Geode* geode = new osg::Geode;
	osg::Geometry* geometry = new osg::Geometry;

	osg::Vec3Array* vertices = new osg::Vec3Array;
	//osg::Vec3Array* normals = new osg::Vec3Array;
	//osg::Vec4ubArray* colours = new osg::Vec4ubArray; //every point has color
	osg::Vec4Array* colours = new osg::Vec4Array(1); //all point have same color
	(*colours)[0].set(red, green, blue, alpha); //set colours (r,g,b,a)

	vertices->reserve(targetNumVertices);
	//normals->reserve(targetNumVertices);
	colours->reserve(targetNumVertices);

	//feed point cloud into osg "geode(s)"
	unsigned int j = 0;
	unsigned int i = 0;
	for (i = 0; i < pointCloud->getSize(); ++i, j += 2) {
//	for (i = 0; i < 60032; ++i, j += 2) { //TODO: strange limit on a windows machine; higher results in crash

		osg::Vec3 tmpPoint;
		tmpPoint.set((float) ((*pointCloud->getPointCloud())[i].getX()), (float) ((*pointCloud->getPointCloud())[i].getY()),
				(float) ((*pointCloud->getPointCloud())[i].getZ()));
		vertices->push_back(tmpPoint);

		/*
		 * If geode gets bigger than 10000 (targetNumVertices) points than create a new child node.
		 * This is necessary to improve the performance due to graphics adapter internals.
		 */
		if (vertices->size() >= targetNumVertices) {
			// finishing setting up the current geometry and add it to the geode.
			geometry->setUseDisplayList(true);
			geometry->setUseVertexBufferObjects(true);
			geometry->setVertexArray(vertices);
			//geometry->setNormalArray(normals);
			//geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
			geometry->setColorArray(colours);
			geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

			geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices->size())); //GL_POINTS

			geode->addDrawable(geometry);

			// allocate a new geometry
			geometry = new osg::Geometry;

			vertices = new osg::Vec3Array;
			//normals = new osg::Vec3Array;
			//colours = new osg::Vec4ubArray;

			vertices->reserve(targetNumVertices);
			//normals->reserve(targetNumVertices);
			//colours->reserve(targetNumVertices);

		}

	}

	geometry->setUseDisplayList(true);
	geometry->setVertexArray(vertices);
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices->size()));
	geometry->setColorArray(colours);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	geometry->setDrawCallback(new DrawCallback);

	geode->addDrawable(geometry);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::Group* group = new osg::Group;
	group->addChild(geode);

	return group;
}

osg::Node* OSGPointCloudVisualizer::createColoredPointCloudNode(ColoredPointCloud3D* pointCloud, float alpha) {

	float red = 0.0;
	float green = 0.0;
	float blue = 0.0;

	unsigned int targetNumVertices = 10000; //maximal points per geode

	osg::Geode* geode = new osg::Geode;
	osg::Geometry* geometry = new osg::Geometry;

	osg::Vec3Array* vertices = new osg::Vec3Array;
	//osg::Vec3Array* normals = new osg::Vec3Array;
	osg::Vec4ubArray* colours = new osg::Vec4ubArray; //every point has color
//	osg::Vec4Array* colours = new osg::Vec4Array(1); //all point have same color
//	(*colours)[0].set(red, green, blue, alpha); //set colours (r,g,b,a)

	vertices->reserve(targetNumVertices);
	//normals->reserve(targetNumVertices);
	colours->reserve(targetNumVertices);

	//feed point cloud into osg "geode(s)"
	unsigned int j = 0;
	unsigned int i = 0;
	for (i = 0; i < pointCloud->getSize(); ++i, j += 2) {

		osg::Vec3 tmpPoint;
		tmpPoint.set((float) ((*pointCloud->getPointCloud())[i].getX()), (float) ((*pointCloud->getPointCloud())[i].getY()),
				(float) ((*pointCloud->getPointCloud())[i].getZ()));
		vertices->push_back(tmpPoint);

		osg::Vec4ub tmpColor;
		red = (*pointCloud->getPointCloud())[i].red;
		green = (*pointCloud->getPointCloud())[i].green;
		blue = (*pointCloud->getPointCloud())[i].blue;
		tmpColor.set(red, green, blue, alpha);
		colours->push_back(tmpColor);

		/*
		 * If geode gets bigger than 10000 (targetNumVertices) points than create a new child node.
		 * This is necessary to improve the performance due to graphics adapter internals.
		 */
		if (vertices->size() >= targetNumVertices) {
			// finishing setting up the current geometry and add it to the geode.
			geometry->setUseDisplayList(true);
			geometry->setUseVertexBufferObjects(true);
			geometry->setVertexArray(vertices);
			//geometry->setNormalArray(normals);
			//geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
			geometry->setColorArray(colours);
			geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

			geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices->size())); //GL_POINTS

			geode->addDrawable(geometry);

			// allocate a new geometry
			geometry = new osg::Geometry;

			vertices = new osg::Vec3Array;
			//normals = new osg::Vec3Array;
			colours = new osg::Vec4ubArray;

			vertices->reserve(targetNumVertices);
			//normals->reserve(targetNumVertices);
			colours->reserve(targetNumVertices);

		}

	}

	geometry->setUseDisplayList(true);
	geometry->setVertexArray(vertices);
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices->size()));
	geometry->setColorArray(colours);
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->setDrawCallback(new DrawCallback);

	geode->addDrawable(geometry);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::Group* group = new osg::Group;
	group->addChild(geode);

	return group;
}

}

#endif //BRICS_OSG_ENABLE

/* EOF */
