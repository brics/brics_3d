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
	WorldModel* myWM = new WorldModel();
	Timer* timer = new Timer();

	/* Add predefined area */
	SceneObject* targetArea = new SceneObject();
	Shape* targetShape = new Cylinder();//(0.5,0); // radius and height in [m] TODO: boost::units ?
	targetArea->shape = targetShape;
	targetArea->transform = new HomogeneousMatrix44(); // here: identity matrix
	targetArea->parentId =  myWM->getRootNodeId(); // hook in after root node
	targetArea->attributes.push_back(Attribute("shapeType","Cylinder"));
	targetArea->attributes.push_back(Attribute("taskType","targetArea"));
	uint targetAreaID; //Stores the ID assigned by the world modle, so we can later reference to the object.
	myWM->addSceneObject(*targetArea, &targetAreaID);

	/* Move the added object */
	IHomogeneousMatrix44* tmpTrasformation = new HomogeneousMatrix44(0,0,0,  //Rotation coefficients
	                                                                 0,0,0,
	                                                                 0,0,0,
	                                                                 2,3,0); //Translation coefficients
	TimeStamp currentTimeStamp;
	currentTimeStamp.timeStamp = timer->getCurrentTime();
//	myWM->setTransform(targetAreaID, tmpTransform, currentTimeStamp); // FIXME how much RSG should be exposed?


	/* Start perception */
	myWM->initPerception();
	myWM->runPerception(); //starts some thread

	/* Pose a query */
	vector<Attribute> queryArributes;
	queryArributes.push_back(Attribute("shapeType","Box"));
	queryArributes.push_back(Attribute("color","green"));
	vector<SceneObject> resultObjects;

	myWM->getSceneObjects(queryArributes, &resultObjects);

	/* Browse the results */
	cout << resultObjects.size() << " green Boxes found." << endl;
	for(unsigned int i = 0; i < resultObjects.size() ; ++i) {
	    cout << "ID: " << resultObjects[i].id << endl;
	    cout << "TF: " << resultObjects[i].transform << endl;
	    for(unsigned int j = 0; j < resultObjects[i].attributes.size() ; ++j) {
	        cout << "Attributes: " << resultObjects[i].attributes[j].key << " " << resultObjects[i].attributes[j].value << endl;
	    }
	}

	if (resultObjects.size() >= 1) {
		uint intrestingID;
		intrestingID = resultObjects[0].id;

		/* See if object's pose changes */
		HomogeneousMatrix44 transform;
		for(int i=0; i < 10 ; ++i) {
			myWM->getCurrentTransform(intrestingID, &transform);
			cout << "TF: " << transform;
			sleep(1); //[s]
		}
	}

	/* Clean up */
	myWM->stopPerception();
	delete myWM;
	delete timer;

	CPPUNIT_FAIL("TODO: Implement.");
}

}  // namespace unitTests

/* EOF */
