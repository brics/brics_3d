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

#include "OSGVisualizer.h"
#include "core/Logger.h"

namespace BRICS_3D {

namespace RSG {

/* OSG Helper classes  */
class OSGOperationAdd : public osg::Operation {
public:

	OSGOperationAdd(OSGVisualizer* obj, osg::ref_ptr<osg::Node> node, osg::ref_ptr<osg::Group> parent = 0) : osg::Operation() {
		this->obj = obj;
		this->node = node;
		this->parent = parent;
	}

	void operator()(osg::Object*) {
		if (parent)
			parent->addChild(node);
		else
			obj->rootGeode->addChild(node);
	}

	OSGVisualizer* obj; //volatile?
	osg::ref_ptr<osg::Node> node;
	osg::ref_ptr<osg::Group> parent;
};

class OSGOperationRemove : public osg::Operation {
public:

	OSGOperationRemove(OSGVisualizer* obj, osg::ref_ptr<osg::Node> node) : osg::Operation() {
		this->obj = obj;
		this->node = node;
	}

	void operator()(osg::Object*) {
		/* loop over all parents and delete _this_ node in ther childrens list */

		if (node->getNumParents() == 0) { // oops we are trying to delete the root node, but this on has no parents...
			LOG(WARNING) << "The root node cannot be deleted.";
			return;
		} else {

			/*
			 * so we found the handle to the current node; now we will invoke
			 * the according delete function for every parent
			 */
			while (node->getNumParents() > 0) { //NOTE: node->getNumberOfParents() will decrease within every iteration...
				unsigned int i = 0;
				osg::ref_ptr<osg::Node> parentNode = node->getParent(i);
				osg::ref_ptr<osg::Group> parentGroup = parentNode->asGroup();
				if (parentGroup != 0 ) {
					parentGroup->removeChild(node);
				} else {
					assert(false); // actually parents need to be groups otherwise sth. really went wrong
				}
			}
		}

	}

	OSGVisualizer* obj;
	osg::ref_ptr<osg::Node> node;
};

class OSGOperationUpdateTransform : public osg::Operation {
public:

	OSGOperationUpdateTransform(OSGVisualizer* obj, osg::ref_ptr<osg::MatrixTransform> node, osg::Matrixd transformData) : osg::Operation() {
		this->obj = obj;
		this->node = node;
		this->transformData = transformData;
	}

	void operator()(osg::Object*) {
		node->setMatrix(transformData);
	}

	OSGVisualizer* obj; //volatile?
	osg::ref_ptr<osg::MatrixTransform> node;
	osg::Matrixd transformData;
};

OSGVisualizer::OSGVisualizer() {
	frameAxisVisualisationScale = 0.1;//1.0;
	init();
}

OSGVisualizer::~OSGVisualizer() {

}

void OSGVisualizer::init() {
	rootGeode = new osg::Group();

	osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc();
	blendFunc->setFunction( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	osg::StateSet* stateset = rootGeode->getOrCreateStateSet();
	stateset->setAttributeAndModes( blendFunc );
	rootGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN ); // this one igth be computaionally expensive as it causes reordering of the scene

	/* optionally add some fram indication */
	rootGeode->addChild(createFrameAxis(frameAxisVisualisationScale));

	idLookUpTable.insert(std::make_pair(getRootId(), rootGeode));
    thread = new boost::thread(boost::bind(&OSGVisualizer::threadFunction, this, this));
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

bool OSGVisualizer::addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) {
	LOG(DEBUG) << "OSGVisualizer: adding node";
	osg::ref_ptr<osg::Node> node = findNodeRecerence(parentId);
	osg::ref_ptr<osg::Group> parentGroup = 0;
	if (node != 0) {
		parentGroup = node->asGroup();
	}
	if (parentGroup != 0) {
		osg::ref_ptr<osg::Node> newNode = new osg::Node();
		viewer.addUpdateOperation(new OSGOperationAdd(this, newNode, parentGroup));
		idLookUpTable.insert(std::make_pair(assignedId, newNode));
		return true;
	}
	LOG(ERROR) << "OSGVisualizer: Parent with ID " << parentId << " is not a group. Cannot add a new node as a child of it.";
	return false;
}

bool OSGVisualizer::addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) {
	LOG(DEBUG) << "OSGVisualizer: adding group";

	osg::ref_ptr<osg::Node> node = findNodeRecerence(parentId);
	osg::ref_ptr<osg::Group> parentGroup = 0;
	if (node != 0) {
		parentGroup = node->asGroup();
	}
	if (parentGroup != 0) {
		osg::ref_ptr<osg::Group> newGroup = new osg::Group();
		viewer.addUpdateOperation(new OSGOperationAdd(this, newGroup, parentGroup));
		idLookUpTable.insert(std::make_pair(assignedId, newGroup));
		return true;
	}
	LOG(ERROR) << "OSGVisualizer: Parent with ID " << parentId << " is not a group. Cannot add a new group as a child of it.";
	return false;
}

bool OSGVisualizer::addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	LOG(DEBUG) << "OSGVisualizer: adding transform node";

	osg::ref_ptr<osg::Node> node = findNodeRecerence(parentId);
	osg::ref_ptr<osg::Group> parentGroup = 0;
	if (node != 0) {
		parentGroup = node->asGroup();
	}
	if (parentGroup != 0) {
		osg::ref_ptr<osg::MatrixTransform> newTransformNode = new osg::MatrixTransform();
		osg::Matrixd transformMatrix;
		transformMatrix.set(transform->getRawData());
		newTransformNode->setMatrix(transformMatrix);
		newTransformNode->addChild(createFrameAxis(frameAxisVisualisationScale)); //optionally for visualization
		viewer.addUpdateOperation(new OSGOperationAdd(this, newTransformNode, parentGroup));
		idLookUpTable.insert(std::make_pair(assignedId, newTransformNode));
		return true;
	}
	LOG(ERROR) << "OSGVisualizer: Parent with ID " << parentId << " is not a group. Cannot add a new transform as a child of it.";
	return false;
}

bool OSGVisualizer::addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp) {
	LOG(DEBUG) << "OSGVisualizer: adding geode";

	osg::ref_ptr<osg::Node> node = findNodeRecerence(parentId);
	osg::ref_ptr<osg::Group> parentGroup = 0;
	if (node != 0) {
		parentGroup = node->asGroup();
	}
	if (parentGroup != 0) {
		float red = (rand()%100)/100.0; // some randomized colors to better distinguish multible point clouds
		float green = (rand()%100)/100.0;
		float blue = (rand()%100)/100.0;
		float alpha = 0.7;

		RSG::PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pointCloud(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
		pointCloud = boost::dynamic_pointer_cast<PointCloud<BRICS_3D::PointCloud3D> >(shape);
		RSG::Box::BoxPtr box(new RSG::Box());
		box =  boost::dynamic_pointer_cast<RSG::Box>(shape);
		RSG::Cylinder::CylinderPtr cylinder(new RSG::Cylinder());
		cylinder =  boost::dynamic_pointer_cast<RSG::Cylinder>(shape);
		if(pointCloud !=0) {
			LOG(DEBUG) << "                 -> Adding a new point cloud.";
			osg::ref_ptr<osg::Node> pointCloudNode = OSGPointCloudVisualizer::createPointCloudNode(pointCloud->data.get(), red, green, blue, alpha);
			viewer.addUpdateOperation(new OSGOperationAdd(this, pointCloudNode, parentGroup));
			idLookUpTable.insert(std::make_pair(assignedId, pointCloudNode));
			return true;
		} else if (box !=0) {
			LOG(DEBUG) << "                 -> Adding a new box.";
			osg::ref_ptr<osg::Box> boxData = new osg::Box();
			boxData->setHalfLengths(osg::Vec3(box->getSizeX()/2.0, box->getSizeY()/2.0,box->getSizeZ()/2.0));
			osg::ref_ptr<osg::Geode> geode = new osg::Geode;
			osg::ref_ptr<osg::ShapeDrawable> geometry = new osg::ShapeDrawable();
			geometry->setShape(boxData);
			geometry->setColor(osg::Vec4(red, green, blue, alpha/2.0));
			geode->addDrawable(geometry);
			geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN ); // might comp. expensive

			viewer.addUpdateOperation(new OSGOperationAdd(this, geode, parentGroup));

		} else if (cylinder !=0) {
			LOG(DEBUG) << "                 -> Adding a new cylinder.";
			osg::ref_ptr<osg::Cylinder> cylinderData = new osg::Cylinder();
			cylinderData->setRadius(static_cast<float>(cylinder->getRadius()));
			cylinderData->setHeight(static_cast<float>(cylinder->getHeight()));
			osg::ref_ptr<osg::Geode> geode = new osg::Geode;
			osg::ref_ptr<osg::ShapeDrawable> geometry = new osg::ShapeDrawable();
			geometry->setShape(cylinderData);
			geometry->setColor(osg::Vec4(red, green, blue, alpha/2.0));
			geode->addDrawable(geometry);
			geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN ); // might comp. expensive

			viewer.addUpdateOperation(new OSGOperationAdd(this, geode, parentGroup));
//		} else if (...) { //more shapes & geometries to come...
//
		} else {
			LOG(WARNING) << "               -> Unsupported geometry type. Cannot add this.";
			return false;
		}
		return true;
	}
	LOG(ERROR) << "OSGVisualizer: Parent with ID " << parentId << " is not a group. Cannot add a new geometric node as a child of it.";
	return false;
}


bool OSGVisualizer::setNodeAttributes(unsigned int id, vector<Attribute> newAttributes) {
	LOG(DEBUG) << "OSGVisualizer: setting attributes";
	return true;
}

bool OSGVisualizer::setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	LOG(DEBUG) << "OSGVisualizer: updating transform data";

	osg::ref_ptr<osg::Node> node = findNodeRecerence(id);
	osg::ref_ptr<osg::Transform> transformNode;
	osg::ref_ptr<osg::MatrixTransform> matrixTransformNode;
	if (node != 0) {
		transformNode = node->asTransform();
		matrixTransformNode = transformNode->asMatrixTransform();
	}
	if (matrixTransformNode != 0) {
		osg::Matrixd transformMatrix;
		transformMatrix.set(transform->getRawData());
		viewer.addUpdateOperation(new OSGOperationUpdateTransform(this, matrixTransformNode, transformMatrix));
		return true;
	}
	LOG(ERROR) << "OSGVisualizer: Node with ID " << id << " is not a transform node. Cannot add a new transform data.";
	return false;

	return true;
}

bool OSGVisualizer::deleteNode(unsigned int id) {
	LOG(DEBUG) << "OSGVisualizer: deleting node";

	osg::ref_ptr<osg::Node> node = findNodeRecerence(id);
	if (node != 0) {
		viewer.addUpdateOperation(new OSGOperationRemove(this, node));
		idLookUpTable.erase(id);
		return true;
	}
	return false;
}

bool OSGVisualizer::addParent(unsigned int id, unsigned int parentId) {
	LOG(DEBUG) << "OSGVisualizer: adding new parent ";
	return true;
}

bool OSGVisualizer::done() {
	return viewer.done();
}

void OSGVisualizer::threadFunction(OSGVisualizer* obj) {
	LOG(INFO) << "OSGVisualizer: Starting visualization.";
	boost::this_thread::sleep(boost::posix_time::milliseconds(100)); // Avoid race condition
	viewer.setSceneData(rootGeode);
	viewer.setUpViewInWindow(10, 10, 500, 500);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

	//The viewer.run() method starts the threads and the traversals.
	viewer.run();
	LOG(INFO) << "OSGVisualizer: Done.";
}

unsigned int OSGVisualizer::getRootId() {
	return 1u; //TODO ask the observed scene (for now: 1u is the same as the gerenerated on in the SimpleIDGenerator)
}

osg::ref_ptr<osg::Node> OSGVisualizer::findNodeRecerence(unsigned int id) {
	nodeIterator = idLookUpTable.find(id);
	if (nodeIterator != idLookUpTable.end()) { //TODO multiple IDs?
		osg::ref_ptr<osg::Node> tmpNodeHandle = nodeIterator->second;
		if(tmpNodeHandle !=0) {
			return nodeIterator->second;
		}
		LOG(WARNING) << "OSGVisualizer: ID " << id << " seems to be orphaned. Possibly its node has been deleted earlier.";
	}
	LOG(WARNING) << "OSGVisualizer: Scene graph does not contain a node with ID " << id;
	return 0;
}

osg::ref_ptr<osg::Node> OSGVisualizer::createFrameAxis(double axisLength) {
	osg::ref_ptr<osg::Group> frameNode = new osg::Group();
	double height = axisLength;
    double radius = height*0.04;
    float alpha = 0.4;

    osg::MatrixTransform* zmt = new osg::MatrixTransform();

    frameNode->addChild(zmt);
    osg::ShapeDrawable *zShape = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f,0.0f,height/2),radius,height));

    zShape->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, alpha));
    osg::Geode *z = new osg::Geode;
    z->addDrawable(zShape);
    zmt->addChild(z);

    osg::MatrixTransform* mt = new osg::MatrixTransform();
    frameNode->addChild(mt);

    //osg::Matrix xMatrix = osg::Matrix::rotate(-osg::PI_2, 0.0, 1.0, 0.0);
    osg::Matrix xMatrix = osg::Matrix::rotate(osg::PI_2, 0.0, 1.0, 0.0);
    mt->setMatrix(xMatrix);


    osg::ShapeDrawable *xShape = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f,0.0f,height/2),radius,height));
    xShape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, alpha));
    osg::Geode *x = new osg::Geode;
    x->addDrawable(xShape);
    mt->addChild(x);


    osg::MatrixTransform *yMt = new osg::MatrixTransform();
    frameNode->addChild(yMt);
    //osg::Matrix yMatrix = osg::Matrix::rotate(osg::PI_2, 1.0, 0.0, 0.0);
    osg::Matrix yMatrix = osg::Matrix::rotate(-osg::PI_2, 1.0, 0.0, 0.0);
    yMt->setMatrix(yMatrix);

    osg::ShapeDrawable *yShape = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f,0.0f,height/2),radius,height));
    yShape->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, alpha));
    osg::Geode *y = new osg::Geode;
    y->addDrawable(yShape);
    yMt->addChild(y);

    frameNode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN ); // might comp. expensive

    return frameNode;
}


}  // namespace RSG

}  // namespace BRICS_3D

/* EOF */
