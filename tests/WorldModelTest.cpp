/**
 * @file 
 * WorldModelTest.cpp
 *
 * @date: Oct 19, 2011
 * @author: sblume
 */

#include "WorldModelTest.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( WorldModelTest );

void WorldModelTest::setUp() {

}

void WorldModelTest::tearDown() {

}

void WorldModelTest::testConstructor() {
	WorldModel* myWorldModel = 0;

	myWorldModel = new WorldModel();
	CPPUNIT_ASSERT(myWorldModel != 0);

	delete myWorldModel;
}

void WorldModelTest::testSimpleHanoiUseCase() {
	///[world_model_hanoi_example]
	WorldModel* myWM = new WorldModel();
	Timer* timer = new Timer();

	/* Add predefined area */
	SceneObject* targetArea = new SceneObject();
	Shape::ShapePtr targetShape(new Cylinder(0.5,0)); // radius and height in [m]
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr initPose(new HomogeneousMatrix44()); 	// here we use the identity matrix as it is the default constructor
	targetArea->shape = targetShape;
	targetArea->transform = initPose;
	targetArea->parentId =  myWM->getRootNodeId(); // hook in after root node
	targetArea->attributes.push_back(Attribute("shapeType","Cylinder")); //here we choose some meaning full tags; theses tags will help us to formulate queries later
	targetArea->attributes.push_back(Attribute("taskType","targetArea"));
	Id targetAreaID; //Stores the ID assigned by the world model, so we can later reference to the object.
	myWM->addSceneObject(*targetArea, targetAreaID);

	/* Move the added object a little bit */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr tmpTransformation(new brics_3d::HomogeneousMatrix44(0,0,0,  //Rotation coefficients
	                                                                                                  0,0,0,
	                                                                                                  0,0,0,
	                                                                                                  2,3,0)); //Translation coefficients
	TimeStamp currentTimeStamp(timer->getCurrentTime());
	myWM->insertTransform(targetAreaID, tmpTransformation);


	/* Start perception */
	myWM->initPerception();
	myWM->runPerception(); //starts some thread

	/* Pose a query formed by Attributes. Please note that multiple Attributes are logically connected via AND */
	vector<Attribute> queryArributes;
	queryArributes.push_back(Attribute("shapeType","Box"));
	queryArributes.push_back(Attribute("color","green"));
	vector<SceneObject> resultObjects;

	myWM->getSceneObjects(queryArributes, resultObjects);

	/* Browse the results */
	cout << resultObjects.size() << " green Boxes found." << endl;
	for(unsigned int i = 0; i < resultObjects.size() ; ++i) { // just loop over all objects
	    cout << "ID: " << resultObjects[i].id << endl;
	    cout << "TF: " << resultObjects[i].transform << endl;
	    for(unsigned int j = 0; j < resultObjects[i].attributes.size() ; ++j) {
	        cout << "Attributes: " << resultObjects[i].attributes[j].key << " " << resultObjects[i].attributes[j].value << endl;
	    }
	}

	if (resultObjects.size() >= 1) {
		Id intrestingID;
		intrestingID = resultObjects[0].id;

		/* See if object's pose changes */
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform;
		for(int i=0; i < 10 ; ++i) {
			myWM->getCurrentTransform(intrestingID, transform);
			cout << "TF: " << transform;
			sleep(1); //[s]
		}
	}

	/* Clean up */
	myWM->stopPerception();
	delete myWM;
	delete timer;
	///[world_model_hanoi_example]
}

void WorldModelTest::testTowerOfHanoi() {

	WorldModel myWM;
	SceneObject tmpSceneObject;
	const double* matrixPtr;

	Shape::ShapePtr targetShape(new Box(0.2, 0.40, 0.30)); // radius and height in [m] TODO: boost::units ?
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             4,5,6)); 						//Translation coefficients
	tmpSceneObject.shape = targetShape;
	tmpSceneObject.transform = transform456;
	tmpSceneObject.parentId =  myWM.getRootNodeId(); // hook in after root node
	tmpSceneObject.attributes.clear();
	tmpSceneObject.attributes.push_back(Attribute("shapeType","Box"));
	tmpSceneObject.attributes.push_back(Attribute("taskType","targetArea"));
	tmpSceneObject.attributes.push_back(Attribute("name","startArea"));

	Id goalId;
	myWM.addSceneObject(tmpSceneObject, goalId);

	vector<Attribute> queryArributes;
	queryArributes.push_back(Attribute("name","startArea"));
	vector<SceneObject> resultObjects;

	myWM.getSceneObjects(queryArributes, resultObjects);

	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultObjects.size()));

	matrixPtr = resultObjects[0].transform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[14], maxTolerance);



}

void WorldModelTest::testFMPCUseCase() {
	/*
	 * Example usage for the Fast Model Predictive Controller, which
	 * acts as a _user_ od the world model.
	 *
	 * Essentially the world model will store a box as a virtual fence
	 * and spheres as virtual obstacles. All geometries have a z value
	 * of zero, because the controllers handles constraints in 2D space.
	 *
	 */

	/*
	 * Setup of the world model.
	 */
	WorldModel* wmHandle = new WorldModel();

	std::vector<rsg::Attribute> attributes;
	attributes.clear();
	attributes.push_back(rsg::Attribute("taskType", "scene_objecs"));
	rsg::Id sceneObjectsId;
	wmHandle->scene.addGroup(wmHandle->getRootNodeId(), sceneObjectsId, attributes);

	/* Box for "virtual fence" */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "box_tf"));
	rsg::Id boxTfId;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform120(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,0)); 						// Translation coefficients
	wmHandle->scene.addTransformNode(sceneObjectsId, boxTfId, attributes, transform120, wmHandle->now());

	attributes.clear();
	attributes.push_back(rsg::Attribute("shape", "Box"));
	attributes.push_back(rsg::Attribute("name", "virtual_fence")); // this name serves as a conventions here
	rsg::Box::BoxPtr box( new rsg::Box(1,2,0));
	rsg::Id boxId;
	wmHandle->scene.addGeometricNode(boxTfId, boxId, attributes, box, wmHandle->now());

	/* A sphere used as obstacle  */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "shpere_1_tf"));
	rsg::Id sphere1TfId;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformShpere1(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0.5,1,0)); 						// Translation coefficients
	wmHandle->scene.addTransformNode(sceneObjectsId, sphere1TfId, attributes, transformShpere1, wmHandle->now());

	attributes.clear();
	attributes.push_back(rsg::Attribute("shape", "Sphere"));
	attributes.push_back(rsg::Attribute("name", "sphere_1"));
	rsg::Sphere::SpherePtr sphere1( new rsg::Sphere(20, Units::CentiMeter));
	rsg::Id sphere1Id;
	wmHandle->scene.addGeometricNode(sphere1TfId, sphere1Id, attributes, sphere1, wmHandle->now());

	/* A another sphere used as obstacle  */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "shpere_2_tf"));
	rsg::Id sphere2TfId;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformShpere2(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0.8,1.5,0)); 						// Translation coefficients
	wmHandle->scene.addTransformNode(sceneObjectsId, sphere2TfId, attributes, transformShpere2, wmHandle->now());

	attributes.clear();
	attributes.push_back(rsg::Attribute("shape", "Sphere"));
	attributes.push_back(rsg::Attribute("name", "sphere_2"));
	rsg::Sphere::SpherePtr sphere2( new rsg::Sphere(10, Units::CentiMeter));
	rsg::Id sphere2Id;
	wmHandle->scene.addGeometricNode(sphere2TfId, sphere2Id, attributes, sphere2, wmHandle->now());


	/*
	 * Sample queries by the FMPC.
	 */
	vector<Attribute> queryAttributes;
	vector<Id> resultIds;

	/* Get data and pose of the box */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(Attribute("name", "virtual_fence"));
	wmHandle->scene.getNodes(queryAttributes, resultIds);

	for(vector<Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
		TimeStamp creationTime;
		Shape::ShapePtr shape;
		wmHandle->scene.getGeometry(*it, shape, creationTime);
		Box::BoxPtr resultBox = boost::dynamic_pointer_cast<Box>(shape);
		if (resultBox != 0) {
			LOG(INFO) << "Box (x,y,z) = " << resultBox->getSizeX() << " "
					<< resultBox->getSizeY() << " "
					<< resultBox->getSizeZ();
		}
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr boxPose;
		wmHandle->scene.getTransformForNode(*it, wmHandle->getRootNodeId(), creationTime, boxPose);
		LOG(INFO) << "Pose of box is = " << std::endl << *boxPose;
	}

	/* Get data of obstacles (spheres) */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(Attribute("shape", "Sphere"));
	wmHandle->scene.getNodes(queryAttributes, resultIds);

	for(vector<Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
		TimeStamp creationTime;
		Shape::ShapePtr shape;
		wmHandle->scene.getGeometry(*it, shape, creationTime);
		Sphere::SpherePtr resultSphere = boost::dynamic_pointer_cast<Sphere>(shape);
		if (resultSphere != 0) {
			LOG(INFO) << "Sphere (r) = " << resultSphere->getRadius();
		}
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr spherePose;
		wmHandle->scene.getTransformForNode(*it, wmHandle->getRootNodeId(), creationTime, spherePose);
		LOG(INFO) << "Pose of sphere is = " << std::endl << *spherePose;
	}

	/*
	 * Sample updates to the world model.
	 */

	/* update the pose of the second sphere */
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformShpere2Update(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0.9,1.55,0)); 						// Translation coefficients
	wmHandle->scene.setTransform(sphere2TfId, transformShpere2Update, wmHandle->now());

	/* update fence geometry be creation of a new geometry */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(Attribute("name", "virtual_fence"));
	wmHandle->scene.getNodes(queryAttributes, resultIds);
	for(vector<Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // delete all node  with "name", "virtual_fence"
		wmHandle->scene.deleteNode(*it);
	}
	attributes.clear();
	attributes.push_back(rsg::Attribute("shape", "Box"));
	attributes.push_back(rsg::Attribute("name", "virtual_fence"));
	rsg::Box::BoxPtr newBox(new rsg::Box(1.5,2.5,0));
	rsg::Id newBoxId;
	wmHandle->scene.addGeometricNode(boxTfId, newBoxId, attributes, newBox, wmHandle->now());

	/*
	 * Query again
	 */

	/* Get data and pose of the box */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(Attribute("name", "virtual_fence"));
	wmHandle->scene.getNodes(queryAttributes, resultIds);

	for(vector<Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
		TimeStamp creationTime;
		Shape::ShapePtr shape;
		wmHandle->scene.getGeometry(*it, shape, creationTime);
		Box::BoxPtr resultBox = boost::dynamic_pointer_cast<Box>(shape);
		if (resultBox != 0) {
			LOG(INFO) << "Box (x,y,z) = " << resultBox->getSizeX() << " "
					<< resultBox->getSizeY() << " "
					<< resultBox->getSizeZ();
		}
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr boxPose;
		wmHandle->scene.getTransformForNode(*it, wmHandle->getRootNodeId(), creationTime, boxPose);
		LOG(INFO) << "Pose of box is = " << std::endl << *boxPose;
	}

	/* Get data of obstacles (spheres) */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(Attribute("shape", "Sphere"));
	wmHandle->scene.getNodes(queryAttributes, resultIds);

	for(vector<Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
		TimeStamp creationTime;
		Shape::ShapePtr shape;
		wmHandle->scene.getGeometry(*it, shape, creationTime);
		Sphere::SpherePtr resultSphere = boost::dynamic_pointer_cast<Sphere>(shape);
		if (resultSphere != 0) {
			LOG(INFO) << "Sphere (r) = " << resultSphere->getRadius();
		}
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr spherePose;
		wmHandle->scene.getTransformForNode(*it, wmHandle->getRootNodeId(), creationTime, spherePose);
		LOG(INFO) << "Pose of sphere is = " << std::endl << *spherePose;
	}

	/*
	 * Cleaning up.
	 */
	delete wmHandle;
}

void WorldModelTest::testMovingSceneObject() {
	WorldModel myWM;
	SceneObject tmpSceneObject;
	SceneObject tmpSceneObject2;
	const double* matrixPtr;
	double x1 = 1.1;
	double y1 = 2.2;
	double z1 = 3.3;

	Shape::ShapePtr targetShape(new Box());
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform1(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             x1,y1,z1)); 						//Translation coefficients
	tmpSceneObject.shape = targetShape;
	tmpSceneObject.transform = transform1;
	tmpSceneObject.parentId =  myWM.getRootNodeId(); // hook in after root node
	tmpSceneObject.attributes.clear();
	tmpSceneObject.attributes.push_back(Attribute("taskType","sceneObject"));

	Id sceneobjectId;
	myWM.addSceneObject(tmpSceneObject, sceneobjectId);

	vector<Attribute> queryArributes;
	queryArributes.push_back(Attribute("taskType","sceneObject"));
	vector<SceneObject> resultObjects;

	myWM.getSceneObjects(queryArributes, resultObjects);

	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultObjects.size()));
	matrixPtr = resultObjects[0].transform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(x1, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(y1, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(z1, matrixPtr[14], maxTolerance);

	double x2 = 1.2;
	double y2 = 2.3;
	double z2 = 3.4;

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform2(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             x2, y2, z2)); 						//Translation coefficients

	myWM.insertTransform(sceneobjectId, transform2);

	queryArributes.clear();
	queryArributes.push_back(Attribute("taskType","sceneObject"));
	resultObjects.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultObjects.size()));

	myWM.getSceneObjects(queryArributes, resultObjects);

	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultObjects.size()));
	matrixPtr = resultObjects[0].transform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(x2, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(y2, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(z2, matrixPtr[14], maxTolerance);

	/* Add a second object */
	double x3 = 1.3;
	double y3 = 2.4;
	double z3 = 3.5;

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform3(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             x3,y3,z3)); 						//Translation coefficients
	tmpSceneObject2.shape = targetShape;
	tmpSceneObject2.transform = transform3;
	tmpSceneObject2.parentId =  myWM.getRootNodeId(); // hook in after root node
	tmpSceneObject2.attributes.clear();
	tmpSceneObject2.attributes.push_back(Attribute("taskType","sceneObject"));

	Id sceneobject2Id;
	myWM.addSceneObject(tmpSceneObject2, sceneobject2Id);

	/* query for both */
	queryArributes.clear();
	queryArributes.push_back(Attribute("taskType","sceneObject"));
	resultObjects.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultObjects.size()));

	myWM.getSceneObjects(queryArributes, resultObjects);

	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultObjects.size()));
	matrixPtr = resultObjects[0].transform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(x2, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(y2, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(z2, matrixPtr[14], maxTolerance);

	matrixPtr = resultObjects[1].transform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(x3, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(y3, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(z3, matrixPtr[14], maxTolerance);

	/* Update both scene objets*/
	double x4 = 10.3;
	double y4 = 20.4;
	double z4 = 30.5;

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform4(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             x4,y4,z4)); 						//Translation coefficients

	double x5 = 100.3;
	double y5 = 200.4;
	double z5 = 300.5;

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform5(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             x5,y5,z5)); 						//Translation coefficients

	myWM.insertTransform(sceneobjectId, transform4);
	myWM.insertTransform(sceneobject2Id, transform5);

	/* query for both */
	queryArributes.clear();
	queryArributes.push_back(Attribute("taskType","sceneObject"));
	resultObjects.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultObjects.size()));

	myWM.getSceneObjects(queryArributes, resultObjects);

	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultObjects.size()));
	matrixPtr = resultObjects[0].transform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(x4, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(y4, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(z4, matrixPtr[14], maxTolerance);

	matrixPtr = resultObjects[1].transform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(x5, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(y5, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(z5, matrixPtr[14], maxTolerance);


}

}  // namespace unitTests

/* EOF */
