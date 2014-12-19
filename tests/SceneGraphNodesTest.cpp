/**
 * @file 
 * SceneGraphNodesTest.cpp
 *
 * @date: Oct 20, 2011
 * @author: sblume
 */

#include "SceneGraphNodesTest.h"
#include <stdexcept>
#include "brics_3d/core/Logger.h"
#include "brics_3d/worldModel/sceneGraph/UuidGenerator.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( SceneGraphNodesTest );

void SceneGraphNodesTest::setUp() {

}

void SceneGraphNodesTest::tearDown() {

}

void SceneGraphNodesTest::testNode() {

	Node testNode1;
	Node* testNode2 = new Node();

	Id defaultId = 0;
	Id testId = 42;

	CPPUNIT_ASSERT_EQUAL(defaultId, testNode1.getId());
	CPPUNIT_ASSERT_EQUAL(defaultId, testNode2->getId());

	testNode1.setId(testId);
	CPPUNIT_ASSERT_EQUAL(testId, testNode1.getId());

	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(testNode1.getAttributes().size())); // Attributes should be empty...
	vector<Attribute> testAttributes;
	testAttributes.push_back(Attribute("myName","testNode1"));
	testNode1.setAttributes(testAttributes);
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(testNode1.getAttributes().size())); // Now there should be one

//	testNode2->setParents()


	delete testNode2;
}

void SceneGraphNodesTest::testGroup() {
	Group root;
	Node::NodePtr child1(new Node());
	Node::NodePtr child2(new Node());

	const Id rootId = 1;
	const Id child1testId = 42;
	const Id child2testId = 123;

	root.setId(rootId);
	child1->setId(child1testId);
	child2->setId(child2testId);


	CPPUNIT_ASSERT_EQUAL(rootId, root.getId()); // preconditions
	CPPUNIT_ASSERT_EQUAL(child1testId, child1->getId());
	CPPUNIT_ASSERT_EQUAL(child2testId, child2->getId());
	CPPUNIT_ASSERT_EQUAL(0u, root.getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(0u, root.getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, child1->getNumberOfParents());

	/* create parent-child relationship */
	root.addChild(child1);

	CPPUNIT_ASSERT_EQUAL(1u, root.getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(0u, root.getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, child1->getNumberOfParents());

	CPPUNIT_ASSERT_EQUAL(rootId, child1->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(child1testId, root.getChild(0)->getId());

	/* create another parent-child relationship */
	root.insertChild(child2, 0u); //insert on first place

	CPPUNIT_ASSERT_EQUAL(2u, root.getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(0u, root.getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, child1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, child2->getNumberOfParents());

	CPPUNIT_ASSERT_EQUAL(rootId, child1->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(rootId, child2->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(child1testId, root.getChild(1)->getId());
	CPPUNIT_ASSERT_EQUAL(child2testId, root.getChild(0)->getId());

	/* delete the children */
	root.removeChild(child1);
	CPPUNIT_ASSERT_EQUAL(0u, child1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, root.getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(child2testId, root.getChild(0)->getId());

	root.removeChild(child2);
	CPPUNIT_ASSERT_EQUAL(0u, child2->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, root.getNumberOfChildren());

//	CPPUNIT_ASSERT_THROW(child1->getParent(1000)->getId(), out_of_range);
//	CPPUNIT_ASSERT_THROW(root.getChild(1000)->getId(), out_of_range);

	/* set up some graph again */
	root.addChild(child1);
	root.addChild(child2);

	CPPUNIT_ASSERT_EQUAL(1u, child1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, child2->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(2u, root.getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(child1testId, root.getChild(0)->getId());
	CPPUNIT_ASSERT_EQUAL(child2testId, root.getChild(1)->getId());

	CPPUNIT_ASSERT_EQUAL(0u, root.getChildIndex(child1));
	CPPUNIT_ASSERT_EQUAL(0u, root.getChildIndex(child1.get()));

	CPPUNIT_ASSERT_EQUAL(1u, root.getChildIndex(child2));
	CPPUNIT_ASSERT_EQUAL(1u, root.getChildIndex(child2.get()));


}

void SceneGraphNodesTest::testTransform() {

	Group::GroupPtr root(new Group);
	rsg::Transform::TransformPtr transform1 (new rsg::Transform);

	const Id rootId = 1;
	const Id transform1Id = 2;

	root->setId(rootId);
	transform1->setId(transform1Id);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions
	CPPUNIT_ASSERT_EQUAL(transform1Id, transform1->getId());

	root->addChild(transform1);

	CPPUNIT_ASSERT_EQUAL(1u, root->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(0u, root->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, transform1->getNumberOfParents());

	CPPUNIT_ASSERT_EQUAL(rootId, transform1->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(transform1Id, root->getChild(0)->getId());

	/* set transform */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); //Translation coefficients
	transform1->insertTransform(transform123, TimeStamp(1.0));

	/* getTransform */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;
	CPPUNIT_ASSERT(resultTransform == 0);
	resultTransform = transform1->getTransform(TimeStamp(1.0));
	CPPUNIT_ASSERT(resultTransform != 0);
	CPPUNIT_ASSERT(resultTransform = transform123);


	matrixPtr = resultTransform->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);



	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform654(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             6,5,4)); //Translation coefficients
	transform1->insertTransform(transform654, TimeStamp(1.0));

	/* getTransform */
	resultTransform.reset();
	CPPUNIT_ASSERT(resultTransform == 0);
	resultTransform = transform1->getTransform(TimeStamp(1.0));
	CPPUNIT_ASSERT(resultTransform != 0);
	CPPUNIT_ASSERT(resultTransform = transform654);


	matrixPtr = resultTransform->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

}

void SceneGraphNodesTest::testTemporalTransform() {
	Group::GroupPtr root(new Group);
	rsg::Transform::TransformPtr transform1 (new rsg::Transform);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;

	const Id rootId = 1;
	const Id transform1Id = 2;

	root->setId(rootId);
	transform1->setId(transform1Id);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions
	CPPUNIT_ASSERT_EQUAL(transform1Id, transform1->getId());

	root->addChild(transform1);


	/* set transform */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform234(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             2,3,4)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform345(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             3,4,5)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             4,5,6)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform567(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             5,6,7)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform678(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             6,7,8)); //Translation coefficients

	transform1->setMaxHistoryDuration(TimeStamp(10.0)); //this is just an arbitrary number!!!

	CPPUNIT_ASSERT_EQUAL(0u, transform1->getCurrentHistoryLenght());
	transform1->insertTransform(transform123, TimeStamp(0));
	CPPUNIT_ASSERT_EQUAL(1u, transform1->getCurrentHistoryLenght());

	/* get latest transform and check ist content */
	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);


	transform1->insertTransform(transform234, TimeStamp(5));
	CPPUNIT_ASSERT_EQUAL(2u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);


	transform1->insertTransform(transform345, TimeStamp(10));
	CPPUNIT_ASSERT_EQUAL(3u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[14], maxTolerance);


	transform1->insertTransform(transform456, TimeStamp(15)); // first shuld be now being deleted
	CPPUNIT_ASSERT_EQUAL(3u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[14], maxTolerance);


	transform1->insertTransform(transform567, TimeStamp(20));
	CPPUNIT_ASSERT_EQUAL(3u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[14], maxTolerance);


	transform1->insertTransform(transform678, TimeStamp(50)); // only this shoulbe present now
	CPPUNIT_ASSERT_EQUAL(1u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[14], maxTolerance);

	/*
	 * try to insert completely outdated data
	 */
	transform1->insertTransform(transform567, TimeStamp(0.0)); // really old data as 50 is the current latest one
	CPPUNIT_ASSERT_EQUAL(1u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[14], maxTolerance);


	/*
	 * try to insert in decending order (latest should stay the same)
	 */
	transform1->insertTransform(transform567, TimeStamp(49.0));
	CPPUNIT_ASSERT_EQUAL(2u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[14], maxTolerance);


	transform1->insertTransform(transform456, TimeStamp(48.0));
	CPPUNIT_ASSERT_EQUAL(3u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[14], maxTolerance);


	transform1->insertTransform(transform345, TimeStamp(47.0));
	CPPUNIT_ASSERT_EQUAL(4u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[14], maxTolerance);


	/*again feed with newer data */
	transform1->insertTransform(transform345, TimeStamp(58.999));
	CPPUNIT_ASSERT_EQUAL(3u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[14], maxTolerance);

	CPPUNIT_ASSERT(transform1->insertTransform(transform234, TimeStamp(59)));
	CPPUNIT_ASSERT_EQUAL(4u, transform1->getCurrentHistoryLenght());

	CPPUNIT_ASSERT(!transform1->insertTransform(transform234, TimeStamp(59))); // insertion at the same timestamp should be ignored and return false.
	CPPUNIT_ASSERT_EQUAL(4u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);


	transform1->insertTransform(transform123, TimeStamp(59.001)); //this should delete the transform with timestamp 49
	CPPUNIT_ASSERT_EQUAL(4u, transform1->getCurrentHistoryLenght());

	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);


}

void SceneGraphNodesTest::testTemporalTransformAccess() {
	Group::GroupPtr root(new Group);
	rsg::Transform::TransformPtr transform1 (new rsg::Transform);
	rsg::Transform::TransformPtr transform2 (new rsg::Transform);
	rsg::Transform::TransformPtr transform3 (new rsg::Transform);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;

	const Id rootId = 1;
	const Id transform1Id = 2;
	const Id transform2Id = 3;
	const Id transform3Id = 4;

	root->setId(rootId);
	transform1->setId(transform1Id);
	transform2->setId(transform2Id);
	transform3->setId(transform3Id);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions
	CPPUNIT_ASSERT_EQUAL(transform1Id, transform1->getId());
	CPPUNIT_ASSERT_EQUAL(transform2Id, transform2->getId());
	CPPUNIT_ASSERT_EQUAL(transform3Id, transform3->getId());

	root->addChild(transform1);
	root->addChild(transform2);

	/* set transforms */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform234(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             2,3,4)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform345(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             3,4,5)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             4,5,6)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform567(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             5,6,7)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform678(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             6,7,8)); //Translation coefficients

	/* Some simple tests for time stamps */
	TimeStamp t1(0.0);
	CPPUNIT_ASSERT(t1 == t1);
	CPPUNIT_ASSERT(TimeStamp(0.0) == t1);
	CPPUNIT_ASSERT(TimeStamp(0.0) == TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) > TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) >= TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(0.0) < TimeStamp(1.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) >= TimeStamp(0.0));

	CPPUNIT_ASSERT_EQUAL(0u, transform1->getCurrentHistoryLenght());
	transform1->setMaxHistoryDuration(TimeStamp(10.0)); //this is just an arbitrary number!!!
	CPPUNIT_ASSERT(transform1->getMaxHistoryDuration() == TimeStamp(10.0));
	CPPUNIT_ASSERT(transform1->getUpdateCount() == 0u);

	transform1->insertTransform(transform123, TimeStamp(0.0));
	CPPUNIT_ASSERT(transform1->getOldestTimeStamp() == TimeStamp(0.0));
	CPPUNIT_ASSERT(transform1->getLatestTimeStamp() == TimeStamp(0.0));
	CPPUNIT_ASSERT(transform1->getUpdateCount() == 1u);

	transform1->insertTransform(transform234, TimeStamp(1.0));
	CPPUNIT_ASSERT(transform1->getOldestTimeStamp() == TimeStamp(0.0));
	CPPUNIT_ASSERT(transform1->getLatestTimeStamp() == TimeStamp(1.0));
	CPPUNIT_ASSERT(transform1->getUpdateCount() == 2u);

	transform1->insertTransform(transform456, TimeStamp(3.0)); // input shall not be ordered
	CPPUNIT_ASSERT(transform1->getOldestTimeStamp() == TimeStamp(0.0));
	CPPUNIT_ASSERT(transform1->getLatestTimeStamp() == TimeStamp(3.0));
	CPPUNIT_ASSERT(transform1->getUpdateCount() ==3u);

	transform1->insertTransform(transform345, TimeStamp(2.0)); // input shall not be ordered
	CPPUNIT_ASSERT(transform1->getOldestTimeStamp() == TimeStamp(0.0));
	CPPUNIT_ASSERT(transform1->getLatestTimeStamp() == TimeStamp(3.0));
	CPPUNIT_ASSERT(transform1->getUpdateCount() == 4u);

	transform1->insertTransform(transform567, TimeStamp(4.0));
	CPPUNIT_ASSERT(transform1->getOldestTimeStamp() == TimeStamp(0.0));
	CPPUNIT_ASSERT(transform1->getLatestTimeStamp() == TimeStamp(4.0));
	CPPUNIT_ASSERT(transform1->getUpdateCount() == 5u);

	CPPUNIT_ASSERT_EQUAL(5u, transform1->getCurrentHistoryLenght());

	/* get latest */
	resultTransform = transform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[14], maxTolerance);

	/* get with existing time stamps */
	resultTransform = transform1->getTransform(TimeStamp(0.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	resultTransform = transform1->getTransform(TimeStamp(1.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	resultTransform = transform1->getTransform(TimeStamp(2.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[14], maxTolerance);

	resultTransform = transform1->getTransform(TimeStamp(3.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[14], maxTolerance);

	resultTransform = transform1->getTransform(TimeStamp(4.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[14], maxTolerance);


	/* get with interpolated time stamps */
	resultTransform = transform1->getTransform(TimeStamp(0.1));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);


	resultTransform = transform1->getTransform(TimeStamp(0.5));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	resultTransform = transform1->getTransform(TimeStamp(0.9));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	resultTransform = transform1->getTransform(TimeStamp(1.4));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	resultTransform = transform1->getTransform(TimeStamp(1.5));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[14], maxTolerance);

	resultTransform = transform1->getTransform(TimeStamp(1.8));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[14], maxTolerance);

	resultTransform = transform1->getTransform(TimeStamp(2.3));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[14], maxTolerance);

	/* tests with an empty history */
	CPPUNIT_ASSERT_EQUAL(0u, transform2->getCurrentHistoryLenght());
	CPPUNIT_ASSERT(transform2->getLatestTransform() == 0);
	CPPUNIT_ASSERT(transform2->getTransform(TimeStamp(0.0)) == 0);
	CPPUNIT_ASSERT(transform2->getLatestTimeStamp() == TimeStamp(0.0));
	CPPUNIT_ASSERT(transform2->getOldestTimeStamp() == TimeStamp(0.0));


	/*
	 * Test with real time.
	 */
	Timer timer;

	CPPUNIT_ASSERT_EQUAL(0u, transform3->getCurrentHistoryLenght());
	transform3->insertTransform(transform123, TimeStamp(timer.getCurrentTime()));
	CPPUNIT_ASSERT_EQUAL(1u, transform3->getCurrentHistoryLenght());
	transform3->insertTransform(transform234, TimeStamp(timer.getCurrentTime()));
	CPPUNIT_ASSERT_EQUAL(2u, transform3->getCurrentHistoryLenght());

	/* oldest */
	resultTransform = transform3->getTransform(TimeStamp(0.0));
	CPPUNIT_ASSERT(resultTransform != 0);
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	/* latest */
	resultTransform = transform3->getTransform(TimeStamp(timer.getCurrentTime()));
	CPPUNIT_ASSERT(resultTransform != 0);
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

}

void SceneGraphNodesTest::testUncertainTransform() {
	Group::GroupPtr root(new Group);
	rsg::UncertainTransform::UncertainTransformPtr uncertainTransform1 (new rsg::UncertainTransform);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;
	ITransformUncertainty::ITransformUncertaintyPtr resultUncertainty;

	const Id rootId = 1;
	const Id uncertainTransform1Id = 2;

	root->setId(rootId);
	uncertainTransform1->setId(uncertainTransform1Id);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions
	CPPUNIT_ASSERT_EQUAL(uncertainTransform1Id, uncertainTransform1->getId());

	root->addChild(uncertainTransform1);

	/* check parent-child relation */
	CPPUNIT_ASSERT_EQUAL(1u, root->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(0u, root->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, uncertainTransform1->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getNumberOfParents());

	/* test data */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform234(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             2,3,4)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform345(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             3,4,5)); //Translation coefficients

	ITransformUncertainty::ITransformUncertaintyPtr uncertainty123(new CovarianceMatrix66(0.91,0.92,0.93, 0.1,0.2,0.3));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty234(new CovarianceMatrix66(0.81,0.82,0.83, 0.4,0.5,0.6));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty345(new CovarianceMatrix66(0.71,0.72,0.73, 0.7,0.8,0.9));

	/* Some checks for the covariance representation */
	CPPUNIT_ASSERT_EQUAL(36, uncertainty123->getDimension());
	CPPUNIT_ASSERT_EQUAL(6, uncertainty123->getRowDimension());
	CPPUNIT_ASSERT_EQUAL(6, uncertainty123->getColumnDimension());

	matrixPtr = uncertainty123->getRawData();
	for (int i = 0; i < uncertainty123->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.91, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.92, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.93, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.1,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.3,  matrixPtr[35], maxTolerance);


//	LOG(ERROR) << "Covariance matrices: ";
//	LOG(ERROR) << *uncertainty123;
//	LOG(ERROR) << *uncertainty234;
//	LOG(ERROR) << *uncertainty345;

	uncertainTransform1->setMaxHistoryDuration(TimeStamp(10.0)); //this is just an arbitrary number!!!

	/* insertion */
	CPPUNIT_ASSERT_EQUAL(0u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(0u, uncertainTransform1->getCurrentUncertaintyHistoryLength());
	CPPUNIT_ASSERT(uncertainTransform1->getTransformUncertainty(TimeStamp(0.0)) == 0); //nullptr
	CPPUNIT_ASSERT(uncertainTransform1->getLatestTransformUncertainty() == 0); //nullptr
	CPPUNIT_ASSERT(uncertainTransform1->getLatestTransformUncertaintyTimeStamp() == TimeStamp(0.0));
	CPPUNIT_ASSERT(uncertainTransform1->getOldestTransformUncertaintyTimeStamp() == TimeStamp(0.0));
	uncertainTransform1->insertTransform(transform123, uncertainty123, TimeStamp(0));
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	/* get latest transform + uncetainty and check ist content */
	resultTransform = uncertainTransform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	resultUncertainty = uncertainTransform1->getLatestTransformUncertainty();
	matrixPtr = resultUncertainty->getRawData();
	for (int i = 0; i < resultUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.91, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.92, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.93, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.1,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.3,  matrixPtr[35], maxTolerance);

	/*
	 * second insertion
	 */
	uncertainTransform1->insertTransform(transform234, uncertainty234, TimeStamp(5.0));
	CPPUNIT_ASSERT_EQUAL(2u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(2u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	/* get latest transform + uncetainty and check its content */
	resultTransform = uncertainTransform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	resultUncertainty = uncertainTransform1->getLatestTransformUncertainty();
	matrixPtr = resultUncertainty->getRawData();
	for (int i = 0; i < resultUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.81, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.82, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.83, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.4,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.6,  matrixPtr[35], maxTolerance);

	/*
	 * third insertion beyond time limit -> first element in each cache should get deleted
	 */
	uncertainTransform1->insertTransform(transform345, uncertainty345, TimeStamp(11.0));
	CPPUNIT_ASSERT_EQUAL(2u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(2u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	/* get latest transform + uncetainty and check its content */
	resultTransform = uncertainTransform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[14], maxTolerance);

	resultUncertainty = uncertainTransform1->getLatestTransformUncertainty();
	matrixPtr = resultUncertainty->getRawData();
	for (int i = 0; i < resultUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.71, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.72, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.73, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.7,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.8,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.9,  matrixPtr[35], maxTolerance);

	/* get transform @ T= 5.0 + uncetainty and check its content */
	resultTransform = uncertainTransform1->getTransform(TimeStamp(5.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(5.0));
	matrixPtr = resultUncertainty->getRawData();
	for (int i = 0; i < resultUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.81, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.82, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.83, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.4,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.6,  matrixPtr[35], maxTolerance);

}

void SceneGraphNodesTest::testTemporalUncertainTransform() {
	Group::GroupPtr root(new Group);
	rsg::UncertainTransform::UncertainTransformPtr uncertainTransform1 (new rsg::UncertainTransform);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;
	ITransformUncertainty::ITransformUncertaintyPtr resultUncertainty;

	const Id rootId = 1;
	const Id uncertainTransform1Id = 2;

	root->setId(rootId);
	uncertainTransform1->setId(uncertainTransform1Id);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions
	CPPUNIT_ASSERT_EQUAL(uncertainTransform1Id, uncertainTransform1->getId());

	root->addChild(uncertainTransform1);

	/* test data */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform234(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             2,3,4)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform345(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             3,4,5)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             4,5,6)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform567(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             5,6,7)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform678(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             6,7,8)); //Translation coefficients

	ITransformUncertainty::ITransformUncertaintyPtr uncertainty123(new CovarianceMatrix66(0.91,0.92,0.93, 0.1,0.2,0.3));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty234(new CovarianceMatrix66(0.81,0.82,0.83, 0.4,0.5,0.6));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty345(new CovarianceMatrix66(0.71,0.72,0.73, 0.7,0.8,0.9));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty456(new CovarianceMatrix66(0.61,0.62,0.63, 0.7,0.8,0.9));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty567(new CovarianceMatrix66(0.51,0.52,0.53, 1.0,1.0,1.0));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty678(new CovarianceMatrix66(0.41,0.42,0.43, 1.0,1.0,1.0));


	uncertainTransform1->setMaxHistoryDuration(TimeStamp(10.0)); //this is just an arbitrary number!!!

	/*
	 * un-ordered insertion
	 */
	CPPUNIT_ASSERT_EQUAL(0u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(0u, uncertainTransform1->getCurrentUncertaintyHistoryLength());
	CPPUNIT_ASSERT(uncertainTransform1->insertTransform(transform123, uncertainty123, TimeStamp(0)));
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	CPPUNIT_ASSERT(uncertainTransform1->insertTransform(transform234, uncertainty234, TimeStamp(7.0)));
	CPPUNIT_ASSERT_EQUAL(2u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(2u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	CPPUNIT_ASSERT(uncertainTransform1->insertTransform(transform345, uncertainty345, TimeStamp(5.0)));
	CPPUNIT_ASSERT_EQUAL(3u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(3u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	CPPUNIT_ASSERT(!uncertainTransform1->insertTransform(transform345, uncertainty345, TimeStamp(5.0))); //no double insertions
	CPPUNIT_ASSERT_EQUAL(3u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(3u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	CPPUNIT_ASSERT(!uncertainTransform1->insertTransform(transform345, TimeStamp(5.0))); //no double insertions
	CPPUNIT_ASSERT_EQUAL(3u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(3u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	/* quick test for latest (at T= 7.0)*/
	resultTransform = uncertainTransform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getLatestTransformUncertainty();
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.81, matrixPtr[0],  maxTolerance);

	/* quick test for oldest (at T= 0.0)*/
	resultTransform = uncertainTransform1->getTransform(TimeStamp(0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(0));
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.91, matrixPtr[0],  maxTolerance);

	/* quick test for oldest (interpolated at T= 2.0) */
	resultTransform = uncertainTransform1->getTransform(TimeStamp(2.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(2.0));
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.91, matrixPtr[0],  maxTolerance);

	/* quick test for T=5.0 (interpolated at T= 4.0)  */
	resultTransform = uncertainTransform1->getTransform(TimeStamp(4.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(4.0));
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.71, matrixPtr[0],  maxTolerance);

	CPPUNIT_ASSERT(uncertainTransform1->getOldestTimeStamp() == uncertainTransform1->getOldestTransformUncertaintyTimeStamp());

	/* insertion that will delet oldest */
	uncertainTransform1->insertTransform(transform456, uncertainty456, TimeStamp(11.0));
	CPPUNIT_ASSERT_EQUAL(3u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(3u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	/* quick test for latest (at T= 11.0)*/
	resultTransform = uncertainTransform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getLatestTransformUncertainty();
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.61, matrixPtr[0],  maxTolerance);

	/* quick test for oldest (at T= 5.0)*/
	resultTransform = uncertainTransform1->getTransform(TimeStamp(0.5));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(0.5));
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.71, matrixPtr[0],  maxTolerance);

	/* quick test for T=0 -> should be redirected to T=5 as there are no older elemnts than 1.0 */
	resultTransform = uncertainTransform1->getTransform(TimeStamp(0.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(0.0));
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.71, matrixPtr[0],  maxTolerance);

	CPPUNIT_ASSERT(uncertainTransform1->getOldestTimeStamp() == uncertainTransform1->getOldestTransformUncertaintyTimeStamp());

	/* insertion that will delete all others  */
	uncertainTransform1->insertTransform(transform567, uncertainty567, TimeStamp(30.0));
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentUncertaintyHistoryLength());

	/* quick test for latest (at T= 30.0)*/
	resultTransform = uncertainTransform1->getLatestTransform();
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getLatestTransformUncertainty();
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.51, matrixPtr[0],  maxTolerance);

	CPPUNIT_ASSERT(uncertainTransform1->getOldestTimeStamp() == uncertainTransform1->getOldestTransformUncertaintyTimeStamp());

	/* flush the caches */
	uncertainTransform1->deleteOutdatedTransforms(TimeStamp(50.0));
	CPPUNIT_ASSERT_EQUAL(0u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(0u, uncertainTransform1->getCurrentUncertaintyHistoryLength());
	CPPUNIT_ASSERT(uncertainTransform1->getOldestTimeStamp() == uncertainTransform1->getOldestTransformUncertaintyTimeStamp());

	/*
	 * insertions with intermixed invocationds of transforms and transforms + uncertainty
	 */
	uncertainTransform1->insertTransform(transform123, uncertainty123, TimeStamp(100.0));
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentUncertaintyHistoryLength());
	CPPUNIT_ASSERT(uncertainTransform1->getOldestTimeStamp() == uncertainTransform1->getOldestTransformUncertaintyTimeStamp());

	uncertainTransform1->insertTransform(transform234, TimeStamp(102.0));
	CPPUNIT_ASSERT_EQUAL(2u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentUncertaintyHistoryLength());
	CPPUNIT_ASSERT(uncertainTransform1->getOldestTimeStamp() == uncertainTransform1->getOldestTransformUncertaintyTimeStamp());

	uncertainTransform1->insertTransform(transform345, uncertainty345, TimeStamp(105.0));
	CPPUNIT_ASSERT_EQUAL(3u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(2u, uncertainTransform1->getCurrentUncertaintyHistoryLength());
	CPPUNIT_ASSERT(uncertainTransform1->getOldestTimeStamp() == uncertainTransform1->getOldestTransformUncertaintyTimeStamp());

	/* quick test at T= 100.0*/
	resultTransform = uncertainTransform1->getTransform(TimeStamp(100.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(100.0));
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.91, matrixPtr[0],  maxTolerance);

	/* quick test at T= 102.0*/
	resultTransform = uncertainTransform1->getTransform(TimeStamp(102.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(102.0));
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.91, matrixPtr[0],  maxTolerance); // redirected from T=100.0

	/* quick test at T= 105.0 */
	resultTransform = uncertainTransform1->getTransform(TimeStamp(105.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(105.0));
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.71, matrixPtr[0],  maxTolerance);

	uncertainTransform1->insertTransform(transform456, TimeStamp(120.0)); //this one unsyncs the caches (start + endpoint)
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(2u, uncertainTransform1->getCurrentUncertaintyHistoryLength());
	CPPUNIT_ASSERT(uncertainTransform1->getOldestTimeStamp() != uncertainTransform1->getOldestTransformUncertaintyTimeStamp());

	/* quick test at T= 120.0 */
	resultTransform = uncertainTransform1->getTransform(TimeStamp(120.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(120.0)); // redirects to 105.0
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.71, matrixPtr[0],  maxTolerance);

	uncertainTransform1->insertTransform(transform567, uncertainty567, TimeStamp(150.0)); //this one re-syncs the durations (start + endpoint)
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentHistoryLenght());
	CPPUNIT_ASSERT_EQUAL(1u, uncertainTransform1->getCurrentUncertaintyHistoryLength());
	CPPUNIT_ASSERT(uncertainTransform1->getOldestTimeStamp() == uncertainTransform1->getOldestTransformUncertaintyTimeStamp());

	/* quick test at T= 150.0 */
	resultTransform = uncertainTransform1->getTransform(TimeStamp(150.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[12], maxTolerance);
	resultUncertainty = uncertainTransform1->getTransformUncertainty(TimeStamp(150.0));
	matrixPtr = resultUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.51, matrixPtr[0],  maxTolerance);

}

void SceneGraphNodesTest::testGeometricNode() {
	/* Graph structure: (remember: nodes can only serve as leaves)
	 *       root(tf)
	 *        |
	 *      geode1
	 */
	const Id rootId = 0;
	const Id geode1Id = 1;


	rsg::Transform::TransformPtr root(new rsg::Transform());
	root->setId(rootId);
	GeometricNode::GeometricNodePtr geode1(new GeometricNode());
	geode1->setId(geode1Id);

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform654(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             6,5,4)); 						//Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;
	root->insertTransform(transform654, TimeStamp(1.0));
	Box::BoxPtr box1(new Box(2,3,4));
	geode1->setShape(box1);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions:
	CPPUNIT_ASSERT_EQUAL(geode1Id, geode1->getId());

	/* setup scenegraph */
	root->addChild(geode1);

	CPPUNIT_ASSERT_EQUAL(1u, root->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(1u, geode1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(rootId, geode1->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(geode1Id, root->getChild(0)->getId());

	Box::BoxPtr tmpBox1;
	CPPUNIT_ASSERT(tmpBox1 == 0);
	tmpBox1 = boost::dynamic_pointer_cast<Box>(geode1->getShape());
	CPPUNIT_ASSERT(tmpBox1 != 0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2, tmpBox1->getSizeX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3, tmpBox1->getSizeY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4, tmpBox1->getSizeZ(), maxTolerance);

	Cylinder::CylinderPtr cylinder1(new Cylinder(0.2,0.1));
	Cylinder::CylinderPtr cylinder2(new Cylinder());
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.1, cylinder1->getHeight(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2, cylinder1->getRadius(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, cylinder2->getHeight(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, cylinder2->getRadius(), maxTolerance);
	geode1->setShape(cylinder1);

	Sphere::SpherePtr sphere0(new Sphere());
	Sphere::SpherePtr sphere1(new Sphere(1));
	Sphere::SpherePtr sphere2(new Sphere(200, Units::CentiMeter));
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, sphere0->getRadius(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, sphere1->getRadius(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, sphere2->getRadius(), maxTolerance);
	sphere0->setRadius(4);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, sphere0->getRadius(), maxTolerance);
	sphere0->setRadius(4, Units::KiloMeter);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4000.0, sphere0->getRadius(), maxTolerance);

	Cylinder::CylinderPtr tmpCylinder1;
	tmpBox1.reset();
	CPPUNIT_ASSERT(tmpCylinder1 == 0);
	CPPUNIT_ASSERT(tmpBox1 == 0);
	tmpBox1 = boost::dynamic_pointer_cast<Box>(geode1->getShape());
	CPPUNIT_ASSERT(tmpBox1 == 0);
	tmpCylinder1 = boost::dynamic_pointer_cast<Cylinder>(geode1->getShape());
	CPPUNIT_ASSERT(tmpCylinder1 != 0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2, tmpCylinder1->getRadius(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.1, tmpCylinder1->getHeight(), maxTolerance);

	geode1->setShape(sphere2);
	Sphere::SpherePtr tmpSphere1;
	tmpSphere1 = boost::dynamic_pointer_cast<Sphere>(geode1->getShape());
	CPPUNIT_ASSERT(tmpSphere1 != 0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, tmpSphere1->getRadius(), maxTolerance);



	/*
	 * check traversals
	 */
	resultTransform = getGlobalTransform(geode1, TimeStamp(1.0));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);




	/* traverse from root */
//	cout << "Traversing with GeometricNode:" << endl;
	IdCollector* idCollector = new IdCollector();
	root->accept(idCollector); // traverse the graph downwards from root with the visitor

	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	CPPUNIT_ASSERT_EQUAL(rootId, idCollector->collectedIDs[0]); //Remember: we have depth-first-search
	CPPUNIT_ASSERT_EQUAL(geode1Id, idCollector->collectedIDs[1]);

	delete idCollector;

}

void SceneGraphNodesTest::testConnection() {
	/* Graph structure:
	 *       root
	 *    ----+-------
	 *    |          |
	 *   node1     node2
	 */
	const Id rootId = 1;
	const Id node1Id = 2;
	const Id node2Id = 3;

	rsg::Group::GroupPtr root(new rsg::Group());
	root->setId(rootId);
	Node::NodePtr node1(new Node());
	node1->setId(node1Id);
	Node::NodePtr node2(new Node());
	node2->setId(node2Id);
	CPPUNIT_ASSERT(node1->getId() == node1Id);
	CPPUNIT_ASSERT(node2->getId() == node2Id);

	root->addChild(node1);
	root->addChild(node2);
	CPPUNIT_ASSERT_EQUAL(2u, root->getNumberOfChildren());

	Connection::ConnectionPtr conn1(new Connection());
	const Id conn1Id = 4;
	conn1->setId(conn1Id);
	CPPUNIT_ASSERT(conn1->getId() == conn1Id);
	CPPUNIT_ASSERT_EQUAL(0u, conn1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, conn1->getNumberOfSourceNodes());
	CPPUNIT_ASSERT_EQUAL(0u, conn1->getNumberOfTargetNodes());

	/* Graph structure:
	 *           root
	 *    --------+---------
	 *    |       |        |
	 *   node1  conn1    node2
	 */

	root->addChild(conn1);
	CPPUNIT_ASSERT_EQUAL(3u, root->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(1u, conn1->getNumberOfParents());

	/* Graph structure:
	 *           root
	 *    ----------+----------
	 *    |        |          |
	 *   node1...>conn1....>node2
	 */

	conn1->addSourceNode(node1.get());
	conn1->addTargetNode(node2.get());
	CPPUNIT_ASSERT_EQUAL(1u, conn1->getNumberOfSourceNodes());
	CPPUNIT_ASSERT_EQUAL(1u, conn1->getNumberOfTargetNodes());

	CPPUNIT_ASSERT(conn1->getSourceNode(0)->getId() == node1Id);
	CPPUNIT_ASSERT(conn1->getTargetNode(0)->getId() == node2Id);

}

void SceneGraphNodesTest::testOwnership() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *             root
	 *              |
	 *        ------+-----
	 *        |          |
	 *      group1     group2
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *            group3
	 */

	/*
	 * here we use an inner scope to completely pass by the ownership
	 * to the scenegraph. "Normal" shared pointers would "hold" the reference for the
	 * duration of the test...
	 */
	Group::GroupPtr root(new Group());
	boost::weak_ptr<Group> testWeakPtr;

	const Id group1Id = 1;
	const Id group2Id = 2;
	const Id group3Id = 3;

	{
		Group::GroupPtr group1(new Group());
		group1->setId(group1Id);
		Group::GroupPtr group2(new Group());
		group2->setId(group2Id);
		Group::GroupPtr group3(new Group());
		group3->setId(group3Id);

		root->addChild(group1);
		root->addChild(group2);
		group1->addChild(group3);
		group2->addChild(group3);

		Group::GroupPtr test(new Group());
		testWeakPtr = test;
//		boost::weak_ptr<Group> tmptWeakPtr(test);
//		testWeakPtr = tmptWeakPtr;
		CPPUNIT_ASSERT(testWeakPtr.lock() != 0);

	} // let the local shared pointer go out of scope...
	CPPUNIT_ASSERT(testWeakPtr.lock() == 0);

	CPPUNIT_ASSERT_EQUAL(2u, root->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(2u, boost::dynamic_pointer_cast<Group>(root->getChild(0))->getChild(0)->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(group1Id, root->getChild(0)->getId());
	CPPUNIT_ASSERT_EQUAL(group2Id, root->getChild(1)->getId());
	CPPUNIT_ASSERT_EQUAL(group3Id, boost::dynamic_pointer_cast<Group>(root->getChild(0))->getChild(0)->getId());
	CPPUNIT_ASSERT_EQUAL(group3Id, boost::dynamic_pointer_cast<Group>(root->getChild(1))->getChild(0)->getId());

	root->removeChildren(0);

	/* now it should be:
	 *             root
	 *              |
	 *              +-----
	 *                   |
	 *                 group2
	 *                   |
	 *               ----+
	 *               |
	 *            group3
	 */

	CPPUNIT_ASSERT_EQUAL(1u, root->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(1u, boost::dynamic_pointer_cast<Group>(root->getChild(0))->getChild(0)->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(group2Id, root->getChild(0)->getId());
	CPPUNIT_ASSERT_EQUAL(group3Id, boost::dynamic_pointer_cast<Group>(root->getChild(0))->getChild(0)->getId());

}

void SceneGraphNodesTest::testSimpleGraph() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *             root
	 *              |
	 *        ------+-------
	 *        |             |
	 *      group1        group2
	 *        |             |
	 *    ----+----  -------+-------
	 *    |       |  |      |      |
	 *   node3    node4  node5   group6
	 */

	const Id rootId = 0;
	const Id  group1Id = 1;
	const Id  group2Id = 2;
	const Id  node3Id = 3;
	const Id  node4Id = 4;
	const Id  node5Id = 5;
	const Id  group6Id = 6;

	Group::GroupPtr root(new Group());
	root->setId(rootId);
	Group::GroupPtr group1(new Group());
	group1->setId(group1Id);
	Group::GroupPtr group2(new Group());
	group2->setId(group2Id);

	Node::NodePtr node3(new Node());
	node3->setId(node3Id);
	Node::NodePtr node4(new Node());
	node4->setId(node4Id);
	Node::NodePtr node5(new Node());
	node5->setId(node5Id);

	Group::GroupPtr group6(new Group());
	group6->setId(group6Id);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions:
	CPPUNIT_ASSERT_EQUAL(group1Id, group1->getId());
	CPPUNIT_ASSERT_EQUAL(group2Id, group2->getId());
	CPPUNIT_ASSERT_EQUAL(node3Id, node3->getId());
	CPPUNIT_ASSERT_EQUAL(node4Id, node4->getId());
	CPPUNIT_ASSERT_EQUAL(node5Id, node5->getId());
	CPPUNIT_ASSERT_EQUAL(group6Id, group6->getId());

	/* set up graph */
	root->addChild(group1);
	root->addChild(group2);

	group1->addChild(node3);
	group1->addChild(node4);

	group2->addChild(node4); //here we brake the "tree" structure to create a real "graph"
	group2->addChild(node5);
	group2->addChild(group6); //just to test heterogeneous nodes.

	/*
	 * now check if every node(group) has its correct place in the graph
	 */
	/* root */
	CPPUNIT_ASSERT_EQUAL(0u, root->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(2u, root->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(group1Id, root->getChild(0)->getId());
	CPPUNIT_ASSERT_EQUAL(group2Id, root->getChild(1)->getId());

	/* group1 */
	CPPUNIT_ASSERT_EQUAL(1u, group1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(rootId, group1->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(2u, group1->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(node3Id, group1->getChild(0)->getId());
	CPPUNIT_ASSERT_EQUAL(node4Id, group1->getChild(1)->getId());

	/* group2 */
	CPPUNIT_ASSERT_EQUAL(1u, group2->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(rootId, group2->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(3u, group2->getNumberOfChildren());
	CPPUNIT_ASSERT_EQUAL(node4Id, group2->getChild(0)->getId());
	CPPUNIT_ASSERT_EQUAL(node5Id, group2->getChild(1)->getId());
	CPPUNIT_ASSERT_EQUAL(group6Id, group2->getChild(2)->getId());

	/* node3 */
	CPPUNIT_ASSERT_EQUAL(1u, node3->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(group1Id, node3->getParent(0)->getId());

	/* node4 */
	CPPUNIT_ASSERT_EQUAL(2u, node4->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(group1Id, node4->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(group2Id, node4->getParent(1)->getId());

	/* node5 */
	CPPUNIT_ASSERT_EQUAL(1u, node5->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(group2Id, node5->getParent(0)->getId());

	/* group6 */
	CPPUNIT_ASSERT_EQUAL(1u, group6->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(group2Id, group6->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(0u, group6->getNumberOfChildren());

	/*
	 * check if we can get some leave-to-root paths
	 */
	CPPUNIT_ASSERT_EQUAL(rootId, node3->getParent(0)->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(rootId, node4->getParent(0)->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(rootId, node4->getParent(1)->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(rootId, node5->getParent(0)->getParent(0)->getId());
	CPPUNIT_ASSERT_EQUAL(rootId, group6->getParent(0)->getParent(0)->getId());

	/*
	 * check if we can get some root-to-leave paths
	 */
	CPPUNIT_ASSERT_EQUAL(node3Id, boost::dynamic_pointer_cast<Group>(root->getChild(0))->getChild(0)->getId());
	CPPUNIT_ASSERT_EQUAL(node4Id, boost::dynamic_pointer_cast<Group>(root->getChild(0))->getChild(1)->getId());

	CPPUNIT_ASSERT_EQUAL(node4Id, boost::dynamic_pointer_cast<Group>(root->getChild(1))->getChild(0)->getId());
	CPPUNIT_ASSERT_EQUAL(node5Id, boost::dynamic_pointer_cast<Group>(root->getChild(1))->getChild(1)->getId());
	CPPUNIT_ASSERT_EQUAL(group6Id, boost::dynamic_pointer_cast<Group>(root->getChild(1))->getChild(2)->getId());
	CPPUNIT_ASSERT_EQUAL(0u, boost::dynamic_pointer_cast<Group>(boost::dynamic_pointer_cast<Group>(root->getChild(1))->getChild(2))->getNumberOfChildren());


}

void SceneGraphNodesTest::testSimpleVisitor() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+----------
	 *        |          |         |
	 *      group1     group2    node3
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *            node4
	 */
	const Id rootId = 0;
	const Id group1Id = 1;
	const Id group2Id = 2;
	const Id node3Id = 3;
	const Id node4Id = 4;

	Group::GroupPtr root(new Group());
	root->setId(rootId);
	Group::GroupPtr group1(new Group());
	group1->setId(group1Id);
	Group::GroupPtr group2(new Group());
	group2->setId(group2Id);

//	Node::NodePtr node3(new Node());
	CustomNode::CustomNodePtr node3(new CustomNode()); //should be handeled polymorph as a regular node
	node3->setId(node3Id);
	Node::NodePtr node4(new Node());
	node4->setId(node4Id);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions:
	CPPUNIT_ASSERT_EQUAL(group1Id, group1->getId());
	CPPUNIT_ASSERT_EQUAL(group2Id, group2->getId());
	CPPUNIT_ASSERT_EQUAL(node3Id, node3->getId());
	CPPUNIT_ASSERT_EQUAL(node4Id, node4->getId());

	/* setup scenegraph */
	root->addChild(group1);
	root->addChild(group2);
	root->addChild(node3);
	group1->addChild(node4);
	group2->addChild(node4);

	IdCollector* idCollector = new IdCollector();
//	cout << "testSimpleVisitor:" << endl;

	/* traverse from root */
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	CPPUNIT_ASSERT(idCollector->getDirection() == INodeVisitor::downwards);

	/* traverse from root */
	root->accept(idCollector); // traverse the graph downwards from root with the visitor

	CPPUNIT_ASSERT_EQUAL(6u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	CPPUNIT_ASSERT_EQUAL(rootId, idCollector->collectedIDs[0]); //Remember: we have depth-first-search
	CPPUNIT_ASSERT_EQUAL(group1Id, idCollector->collectedIDs[1]);
	CPPUNIT_ASSERT_EQUAL(node4Id, idCollector->collectedIDs[2]);
	CPPUNIT_ASSERT_EQUAL(group2Id, idCollector->collectedIDs[3]);
	CPPUNIT_ASSERT_EQUAL(node4Id, idCollector->collectedIDs[4]);
	CPPUNIT_ASSERT_EQUAL(node3Id, idCollector->collectedIDs[5]);

	/* traverse from group2 */
	idCollector->collectedIDs.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	CPPUNIT_ASSERT(idCollector->getDirection() == INodeVisitor::downwards);

	group2->accept(idCollector);

	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	CPPUNIT_ASSERT_EQUAL(group2Id, idCollector->collectedIDs[3]);
	CPPUNIT_ASSERT_EQUAL(node4Id, idCollector->collectedIDs[4]);

	/* traverse from upwards from group2 */
	idCollector->collectedIDs.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	idCollector->setDirection(INodeVisitor::upwards);
	CPPUNIT_ASSERT(idCollector->getDirection() == INodeVisitor::upwards);

	group2->accept(idCollector);

	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	CPPUNIT_ASSERT_EQUAL(group2Id, idCollector->collectedIDs[0]);
	CPPUNIT_ASSERT_EQUAL(rootId, idCollector->collectedIDs[1]);

	/* traverse from upwards from node4 */
	idCollector->collectedIDs.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	idCollector->setDirection(INodeVisitor::upwards);
	CPPUNIT_ASSERT(idCollector->getDirection() == INodeVisitor::upwards);

	node4->accept(idCollector);

	CPPUNIT_ASSERT_EQUAL(5u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	CPPUNIT_ASSERT_EQUAL(node4Id, idCollector->collectedIDs[0]);
	CPPUNIT_ASSERT_EQUAL(group1Id, idCollector->collectedIDs[1]);
	CPPUNIT_ASSERT_EQUAL(rootId, idCollector->collectedIDs[2]);
	CPPUNIT_ASSERT_EQUAL(group2Id, idCollector->collectedIDs[3]);
	CPPUNIT_ASSERT_EQUAL(rootId, idCollector->collectedIDs[4]);

	delete idCollector;
}

void SceneGraphNodesTest::testPathCollectorVisitor() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+----------
	 *        |          |         |
	 *      group1     group2    node3
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *            node4
	 */
	const Id rootId = 0;
	const Id group1Id = 1;
	const Id group2Id = 2;
	const Id node3Id = 3;
	const Id node4Id = 4;

	Group::GroupPtr root(new Group());
	root->setId(rootId);
	Group::GroupPtr group1(new Group());
	group1->setId(group1Id);
	Group::GroupPtr group2(new Group());
	group2->setId(group2Id);

	Node::NodePtr node3(new Node());
	node3->setId(node3Id);
	Node::NodePtr node4(new Node());
	node4->setId(node4Id);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions:
	CPPUNIT_ASSERT_EQUAL(group1Id, group1->getId());
	CPPUNIT_ASSERT_EQUAL(group2Id, group2->getId());
	CPPUNIT_ASSERT_EQUAL(node3Id, node3->getId());
	CPPUNIT_ASSERT_EQUAL(node4Id, node4->getId());

	/* setup scenegraph */
	root->addChild(group1);
	root->addChild(group2);
	root->addChild(node3);
	group1->addChild(node4);
	group2->addChild(node4);

	PathCollector* pathCollector = new PathCollector();

	/* root */
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(pathCollector->getNodePaths().size()));

	root->accept(pathCollector);

	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(pathCollector->getNodePaths().size()));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(pathCollector->getNodePaths()[0].size()));

	/* group2 */
	pathCollector->reset();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(pathCollector->getNodePaths().size()));

	group2->accept(pathCollector);

	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(pathCollector->getNodePaths().size()));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(pathCollector->getNodePaths()[0].size()));
	CPPUNIT_ASSERT_EQUAL(rootId, (*pathCollector->getNodePaths()[0][0]).getId());

	/* node4 */
	pathCollector->reset();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(pathCollector->getNodePaths().size()));

	node4->accept(pathCollector);

	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(pathCollector->getNodePaths().size()));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(pathCollector->getNodePaths()[0].size()));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(pathCollector->getNodePaths()[1].size()));

	CPPUNIT_ASSERT_EQUAL(rootId, (*pathCollector->getNodePaths()[0][0]).getId()); // 1. path
	CPPUNIT_ASSERT_EQUAL(group1Id, (*pathCollector->getNodePaths()[0][1]).getId());

	CPPUNIT_ASSERT_EQUAL(rootId, (*pathCollector->getNodePaths()[1][0]).getId()); // 2. path
	CPPUNIT_ASSERT_EQUAL(group2Id, (*pathCollector->getNodePaths()[1][1]).getId());

	delete pathCollector;
}

void SceneGraphNodesTest::testTransformVisitor() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *            root
	 *              |
	 *        ------+-----
	 *        |          |
	 *       tf1        tf2
	 *        |          |
	 *       tf3        group4
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *            node5
	 */
	const Id rootId = 0;
	const Id tf1Id = 1;
	const Id tf2Id = 2;
	const Id tf3Id = 3;
	const Id group4Id = 4;
	const Id node5Id = 5;

	Group::GroupPtr root(new Group());
	root->setId(rootId);
	rsg::Transform::TransformPtr tf1(new rsg::Transform());
	tf1->setId(tf1Id);
	rsg::Transform::TransformPtr tf2(new rsg::Transform());
	tf2->setId(tf2Id);
	rsg::Transform::TransformPtr tf3(new rsg::Transform());
	tf3->setId(tf3Id);
	Group::GroupPtr group4(new Group());
	group4->setId(group4Id);
	Node::NodePtr node5(new Node());
	node5->setId(node5Id);

	TimeStamp dummyTime(1.0);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						//Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform654(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             6,5,4)); 						//Translation coefficients

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform789(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             7,8,9)); 						//Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;

	tf1->insertTransform(transform123, dummyTime + TimeStamp(1, Units::MilliSecond));
	tf2->insertTransform(transform654, dummyTime + TimeStamp(2, Units::MilliSecond));
	tf3->insertTransform(transform789, dummyTime + TimeStamp(3, Units::MilliSecond));

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions:
	CPPUNIT_ASSERT_EQUAL(tf1Id, tf1->getId());
	CPPUNIT_ASSERT_EQUAL(tf2Id, tf2->getId());
	CPPUNIT_ASSERT_EQUAL(tf3Id, tf3->getId());
	CPPUNIT_ASSERT_EQUAL(group4Id, group4->getId());
	CPPUNIT_ASSERT_EQUAL(node5Id, node5->getId());

	/* setup scenegraph */
	root->addChild(tf1);
	root->addChild(tf2);
	tf1->addChild(tf3);
	tf3->addChild(node5);
	tf2->addChild(group4);
	group4->addChild(node5);


	IdCollector* idCollector = new IdCollector();

//	cout << "Graph with transforms" << endl;
	root->accept(idCollector);

	CPPUNIT_ASSERT_EQUAL(7u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	CPPUNIT_ASSERT_EQUAL(rootId, idCollector->collectedIDs[0]); //Remember: we have depth-first-search
	CPPUNIT_ASSERT_EQUAL(tf1Id, idCollector->collectedIDs[1]);
	CPPUNIT_ASSERT_EQUAL(tf3Id, idCollector->collectedIDs[2]);
	CPPUNIT_ASSERT_EQUAL(node5Id, idCollector->collectedIDs[3]);
	CPPUNIT_ASSERT_EQUAL(tf2Id, idCollector->collectedIDs[4]);
	CPPUNIT_ASSERT_EQUAL(group4Id, idCollector->collectedIDs[5]);
	CPPUNIT_ASSERT_EQUAL(node5Id, idCollector->collectedIDs[6]);

	PathCollector* pathCollector = new PathCollector();
	node5->accept(pathCollector);

	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(pathCollector->getNodePaths().size()));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(pathCollector->getNodePaths()[0].size()));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(pathCollector->getNodePaths()[1].size()));

	CPPUNIT_ASSERT_EQUAL(rootId, (*pathCollector->getNodePaths()[0][0]).getId()); // 1. path
	CPPUNIT_ASSERT_EQUAL(tf1Id, (*pathCollector->getNodePaths()[0][1]).getId());
	CPPUNIT_ASSERT_EQUAL(tf3Id, (*pathCollector->getNodePaths()[0][2]).getId());

	CPPUNIT_ASSERT_EQUAL(rootId, (*pathCollector->getNodePaths()[1][0]).getId()); // 2. path
	CPPUNIT_ASSERT_EQUAL(tf2Id, (*pathCollector->getNodePaths()[1][1]).getId());
	CPPUNIT_ASSERT_EQUAL(group4Id, (*pathCollector->getNodePaths()[1][2]).getId());

	resultTransform = getGlobalTransformAlongPath(pathCollector->getNodePaths()[0], dummyTime);
//	cout << (*resultTransform) << endl;

	matrixPtr = resultTransform->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(12.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

	resultTransform = getGlobalTransformAlongPath(pathCollector->getNodePaths()[1], dummyTime);
//	cout << (*resultTransform) << endl;

	matrixPtr = resultTransform->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);


	/*
	 * Here we update one transform and look what will happen...
	 */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr someUpdate(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             -100,-100,-100)); 						//Translation coefficients
	tf1->insertTransform(someUpdate, dummyTime + TimeStamp(4, Units::MilliSecond));
	pathCollector->reset();
	node5->accept(pathCollector);


	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(pathCollector->getNodePaths().size()));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(pathCollector->getNodePaths()[0].size()));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(pathCollector->getNodePaths()[1].size()));

	CPPUNIT_ASSERT_EQUAL(rootId, (*pathCollector->getNodePaths()[0][0]).getId()); // 1. path
	CPPUNIT_ASSERT_EQUAL(tf1Id, (*pathCollector->getNodePaths()[0][1]).getId());
	CPPUNIT_ASSERT_EQUAL(tf3Id, (*pathCollector->getNodePaths()[0][2]).getId());

	CPPUNIT_ASSERT_EQUAL(rootId, (*pathCollector->getNodePaths()[1][0]).getId()); // 2. path
	CPPUNIT_ASSERT_EQUAL(tf2Id, (*pathCollector->getNodePaths()[1][1]).getId());
	CPPUNIT_ASSERT_EQUAL(group4Id, (*pathCollector->getNodePaths()[1][2]).getId());

	resultTransform = getGlobalTransformAlongPath(pathCollector->getNodePaths()[0], dummyTime + TimeStamp(4, Units::MilliSecond));
//	cout << (*resultTransform) << endl;

	matrixPtr = resultTransform->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-93.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-92.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-91.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

	resultTransform = getGlobalTransformAlongPath(pathCollector->getNodePaths()[1], dummyTime + TimeStamp(4, Units::MilliSecond));
//	cout << (*resultTransform) << endl;

	matrixPtr = resultTransform->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

	delete pathCollector;
	delete idCollector;
}

void SceneGraphNodesTest::testUncertainTransformVisitor() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *            root
	 *              |
	 *        ------+-----
	 *        |          |
	 *       tf1        utf2
	 *        |          |
	 *       utf3        group4
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *            node5
	 */
	const Id rootId = 0;
	const Id tf1Id = 1;
	const Id utf2Id = 2;
	const Id utf3Id = 3;
	const Id group4Id = 4;
	const Id node5Id = 5;

	Group::GroupPtr root(new Group());
	root->setId(rootId);
	rsg::Transform::TransformPtr tf1(new rsg::Transform());
	tf1->setId(tf1Id);
	rsg::UncertainTransform::UncertainTransformPtr utf2(new rsg::UncertainTransform());
	utf2->setId(utf2Id);
	rsg::UncertainTransform::UncertainTransformPtr utf3(new rsg::UncertainTransform());
	utf3->setId(utf3Id);
	Group::GroupPtr group4(new Group());
	group4->setId(group4Id);
	Node::NodePtr node5(new Node());
	node5->setId(node5Id);

	TimeStamp dummyTime(1.0);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						//Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform654(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             6,5,4)); 						//Translation coefficients

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform789(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             7,8,9)); 						//Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;

	ITransformUncertainty::ITransformUncertaintyPtr uncertainty234(new CovarianceMatrix66(0.81,0.82,0.83, 0.4,0.5,0.6));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty345(new CovarianceMatrix66(0.71,0.72,0.73, 0.7,0.8,0.9));


	tf1->insertTransform(transform123, dummyTime);
	utf2->insertTransform(transform654, uncertainty234, dummyTime);
	utf3->insertTransform(transform789, uncertainty345, dummyTime);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions:
	CPPUNIT_ASSERT_EQUAL(tf1Id, tf1->getId());
	CPPUNIT_ASSERT_EQUAL(utf2Id, utf2->getId());
	CPPUNIT_ASSERT_EQUAL(utf3Id, utf3->getId());
	CPPUNIT_ASSERT_EQUAL(group4Id, group4->getId());
	CPPUNIT_ASSERT_EQUAL(node5Id, node5->getId());

	/* setup scenegraph */
	root->addChild(tf1);
	root->addChild(utf2);
	tf1->addChild(utf3);
	utf3->addChild(node5);
	utf2->addChild(group4);
	group4->addChild(node5);


	IdCollector* idCollector = new IdCollector();

//	cout << "Graph with transforms" << endl;
	root->accept(idCollector);

	CPPUNIT_ASSERT_EQUAL(7u, static_cast<unsigned int>(idCollector->collectedIDs.size()));
	CPPUNIT_ASSERT_EQUAL(rootId, idCollector->collectedIDs[0]); //Remember: we have depth-first-search
	CPPUNIT_ASSERT_EQUAL(tf1Id, idCollector->collectedIDs[1]);
	CPPUNIT_ASSERT_EQUAL(utf3Id, idCollector->collectedIDs[2]);
	CPPUNIT_ASSERT_EQUAL(node5Id, idCollector->collectedIDs[3]);
	CPPUNIT_ASSERT_EQUAL(utf2Id, idCollector->collectedIDs[4]);
	CPPUNIT_ASSERT_EQUAL(group4Id, idCollector->collectedIDs[5]);
	CPPUNIT_ASSERT_EQUAL(node5Id, idCollector->collectedIDs[6]);

	PathCollector* pathCollector = new PathCollector();
	node5->accept(pathCollector);

	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(pathCollector->getNodePaths().size()));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(pathCollector->getNodePaths()[0].size()));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(pathCollector->getNodePaths()[1].size()));

	CPPUNIT_ASSERT_EQUAL(rootId, (*pathCollector->getNodePaths()[0][0]).getId()); // 1. path
	CPPUNIT_ASSERT_EQUAL(tf1Id, (*pathCollector->getNodePaths()[0][1]).getId());
	CPPUNIT_ASSERT_EQUAL(utf3Id, (*pathCollector->getNodePaths()[0][2]).getId());

	CPPUNIT_ASSERT_EQUAL(rootId, (*pathCollector->getNodePaths()[1][0]).getId()); // 2. path
	CPPUNIT_ASSERT_EQUAL(utf2Id, (*pathCollector->getNodePaths()[1][1]).getId());
	CPPUNIT_ASSERT_EQUAL(group4Id, (*pathCollector->getNodePaths()[1][2]).getId());

	resultTransform = getGlobalTransformAlongPath(pathCollector->getNodePaths()[0], dummyTime);
//	cout << (*resultTransform) << endl;

	matrixPtr = resultTransform->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(12.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

	resultTransform = getGlobalTransformAlongPath(pathCollector->getNodePaths()[1], dummyTime);
//	cout << (*resultTransform) << endl;

	matrixPtr = resultTransform->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

    //TODO test update of uncertain transform


}

void SceneGraphNodesTest::testGlobalTransformCalculation() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *            root(tf)
	 *              |
	 *        ------+-----
	 *        |          |
	 *       tf1        tf2
	 *        |          |
	 *       tf3        group4
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *            node5
	 */
	const Id rootId = 0;
	const Id tf1Id = 1;
	const Id tf2Id = 2;
	const Id tf3Id = 3;
	const Id group4Id = 4;
	const Id node5Id = 5;

	rsg::Transform::TransformPtr root(new rsg::Transform());
	root->setId(rootId);
	rsg::Transform::TransformPtr tf1(new rsg::Transform());
	tf1->setId(tf1Id);
	rsg::Transform::TransformPtr tf2(new rsg::Transform());
	tf2->setId(tf2Id);
	rsg::Transform::TransformPtr tf3(new rsg::Transform());
	tf3->setId(tf3Id);
	Group::GroupPtr group4(new Group());
	group4->setId(group4Id);
	Node::NodePtr node5(new Node());
	node5->setId(node5Id);

	TimeStamp dummyTime(1.0);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform001(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0,0,-1));
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						//Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform654(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             6,5,4)); 						//Translation coefficients

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform789(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             7,8,9)); 						//Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;

	root->insertTransform(transform001, dummyTime +  TimeStamp(1, Units::MilliSecond));
	tf1->insertTransform(transform123, dummyTime +  TimeStamp(2, Units::MilliSecond));
	tf2->insertTransform(transform654, dummyTime +  TimeStamp(3, Units::MilliSecond));
	tf3->insertTransform(transform789, dummyTime +  TimeStamp(4, Units::MilliSecond));

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions:
	CPPUNIT_ASSERT_EQUAL(tf1Id, tf1->getId());
	CPPUNIT_ASSERT_EQUAL(tf2Id, tf2->getId());
	CPPUNIT_ASSERT_EQUAL(tf3Id, tf3->getId());
	CPPUNIT_ASSERT_EQUAL(group4Id, group4->getId());
	CPPUNIT_ASSERT_EQUAL(node5Id, node5->getId());

	/* setup scenegraph */
	root->addChild(tf1);
	root->addChild(tf2);
	tf1->addChild(tf3);
	tf3->addChild(node5);
	tf2->addChild(group4);
	group4->addChild(node5);


	resultTransform = getGlobalTransform(root, dummyTime); //There is actually an offeste in time but this should be ok for the cache.
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(tf1, dummyTime);
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(tf3, dummyTime);
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(11.0, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(node5, dummyTime);
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(11.0, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(tf2, dummyTime);
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(group4, dummyTime);
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr someUpdate(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             100,100,99));
	root->insertTransform(someUpdate, dummyTime + TimeStamp(5, Units::MilliSecond));

	resultTransform = getGlobalTransform(root, dummyTime + TimeStamp(5, Units::MilliSecond));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(100.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(100.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(99, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(tf1, dummyTime + TimeStamp(5, Units::MilliSecond));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(101.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(102.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(102.0, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(tf3, dummyTime + TimeStamp(5, Units::MilliSecond));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(108.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(110.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(111.0, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(node5, dummyTime + TimeStamp(5, Units::MilliSecond));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(108.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(110.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(111.0, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(tf2, dummyTime + TimeStamp(5, Units::MilliSecond));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(106.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(105.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(103.0, matrixPtr[14], maxTolerance);

	resultTransform = getGlobalTransform(group4, dummyTime + TimeStamp(5, Units::MilliSecond));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(106.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(105.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(103.0, matrixPtr[14], maxTolerance);

	tf3->removeChildren(0); //make a tree

	resultTransform = getGlobalTransform(node5, dummyTime + TimeStamp(5, Units::MilliSecond));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(106.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(105.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(103.0, matrixPtr[14], maxTolerance);

}

void SceneGraphNodesTest::testAttributeFinder() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+----------
	 *        |          |         |
	 *       tf1        group2    node3
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *            geode4
	 */
	const Id rootId = 0;
	const Id tf1Id = 1;
	const Id group2Id = 2;
	const Id node3Id = 3;
	const Id geode4Id = 4;

	Attribute testAttribute1("name","test1");
	Attribute testAttribute2("name","test1");
	Attribute testAttribute3("name","test2");
	Attribute testAttribute4("noName","test1");
	Attribute testAttribute5;
	const Attribute testAttribute6("name","const1");
	cout << testAttribute1 << endl;

	CPPUNIT_ASSERT(testAttribute1 == testAttribute2);
	CPPUNIT_ASSERT(!(testAttribute1 == testAttribute3));
	CPPUNIT_ASSERT(!(testAttribute1 == testAttribute4));

	CPPUNIT_ASSERT(!(testAttribute1 != testAttribute2));
	CPPUNIT_ASSERT(testAttribute1 != testAttribute3);
	CPPUNIT_ASSERT(testAttribute1 != testAttribute4);
	CPPUNIT_ASSERT(testAttribute1 != testAttribute5);
	CPPUNIT_ASSERT(testAttribute5 != testAttribute6);



	CPPUNIT_ASSERT(testAttribute1 == Attribute("name","test1"));

	vector<Attribute> tmpAttributes;
	tmpAttributes.push_back(Attribute("taskType","targetArea"));
	tmpAttributes.push_back(Attribute("name","Goal Area"));

	CPPUNIT_ASSERT((attributeListContainsAttribute(tmpAttributes, Attribute("name","Goal Area"))) == true );
	CPPUNIT_ASSERT((attributeListContainsAttribute(tmpAttributes, Attribute("taskType","targetArea"))) == true );
	CPPUNIT_ASSERT((attributeListContainsAttribute(tmpAttributes, Attribute("name123","Goal Area"))) == false );


	Group::GroupPtr root(new Group());
	root->setId(rootId);
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","root"));
	root->setAttributes(tmpAttributes);
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(root->getAttributes().size()));
	CPPUNIT_ASSERT(root->getAttributes()[0] == Attribute("name","root"));

	rsg::Transform::TransformPtr tf1(new rsg::Transform());
	tf1->setId(tf1Id);
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("frameID","base_link"));
	tf1->setAttributes(tmpAttributes);
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(tf1->getAttributes().size()));
	CPPUNIT_ASSERT(tf1->getAttributes()[0] == Attribute("frameID","base_link"));

	Group::GroupPtr group2(new Group());
	group2->setId(group2Id);
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("taskType","targetArea"));
	tmpAttributes.push_back(Attribute("name","Goal Area"));
	group2->setAttributes(tmpAttributes);
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(group2->getAttributes().size()));
	CPPUNIT_ASSERT(group2->getAttributes()[0] == Attribute("taskType","targetArea"));
	CPPUNIT_ASSERT(group2->getAttributes()[1] == Attribute("name","Goal Area"));

	Node::NodePtr node3(new Node());
	node3->setId(node3Id);
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(node3->getAttributes().size()));

	GeometricNode::GeometricNodePtr geode4(new GeometricNode());
	geode4->setId(geode4Id);
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("shapeType","Cylinder"));
	tmpAttributes.push_back(Attribute("taskType","targetArea"));
	geode4->setAttributes(tmpAttributes);
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(geode4->getAttributes().size()));
	CPPUNIT_ASSERT(geode4->getAttributes()[0] == Attribute("shapeType","Cylinder"));
	CPPUNIT_ASSERT(geode4->getAttributes()[1] == Attribute("taskType","targetArea"));

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions:
	CPPUNIT_ASSERT_EQUAL(tf1Id, tf1->getId());
	CPPUNIT_ASSERT_EQUAL(group2Id, group2->getId());
	CPPUNIT_ASSERT_EQUAL(node3Id, node3->getId());
	CPPUNIT_ASSERT_EQUAL(geode4Id, geode4->getId());

	/* setup scenegraph */
	root->addChild(tf1);
	root->addChild(group2);
	root->addChild(node3);
	tf1->addChild(geode4);
	group2->addChild(geode4);

	/*
	 * query for certain attributes
	 */
	AttributeFinder* attributeFinder = new AttributeFinder();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","root"));
	attributeFinder->setQueryAttributes(tmpAttributes);
	root->accept(attributeFinder);
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[0]).getAttributes()[0] == Attribute("name", "root"));
	CPPUNIT_ASSERT_EQUAL(rootId, (*attributeFinder->getMatchingNodes()[0]).getId());

	attributeFinder->reset();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("frameID","base_link"));
	attributeFinder->setQueryAttributes(tmpAttributes);
	root->accept(attributeFinder);
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[0]).getAttributes()[0] == Attribute("frameID","base_link"));
	CPPUNIT_ASSERT_EQUAL(tf1Id, (*attributeFinder->getMatchingNodes()[0]).getId());

	attributeFinder->reset();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("taskType","targetArea"));
	attributeFinder->setQueryAttributes(tmpAttributes);
	root->accept(attributeFinder);
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[0]).getAttributes()[1] == Attribute("taskType","targetArea"));
	CPPUNIT_ASSERT_EQUAL(geode4Id, (*attributeFinder->getMatchingNodes()[0]).getId());
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[1]).getAttributes()[0] == Attribute("taskType","targetArea"));
	CPPUNIT_ASSERT_EQUAL(group2Id, (*attributeFinder->getMatchingNodes()[1]).getId());

	attributeFinder->reset();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","Goal Area"));
	attributeFinder->setQueryAttributes(tmpAttributes);
	root->accept(attributeFinder);
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[0]).getAttributes()[1] == Attribute("name","Goal Area"));
	CPPUNIT_ASSERT_EQUAL(group2Id, (*attributeFinder->getMatchingNodes()[0]).getId());

	attributeFinder->reset();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("shapeType","Cylinder"));
	attributeFinder->setQueryAttributes(tmpAttributes);
	root->accept(attributeFinder);
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[0]).getAttributes()[0] == Attribute("shapeType","Cylinder"));
	CPPUNIT_ASSERT_EQUAL(geode4Id, (*attributeFinder->getMatchingNodes()[0]).getId());

	attributeFinder->reset();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("justSomething","Cylinder"));
	attributeFinder->setQueryAttributes(tmpAttributes);
	root->accept(attributeFinder);
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));


	/* test AND */
	attributeFinder->reset();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("taskType","targetArea"));
	tmpAttributes.push_back(Attribute("name","Goal Area"));
	attributeFinder->setQueryAttributes(tmpAttributes);
	root->accept(attributeFinder);
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[0]).getAttributes()[0] == Attribute("taskType","targetArea"));
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[0]).getAttributes()[1] == Attribute("name","Goal Area"));
	CPPUNIT_ASSERT_EQUAL(group2Id, (*attributeFinder->getMatchingNodes()[0]).getId());


	attributeFinder->reset();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("shapeType","Cylinder"));
	tmpAttributes.push_back(Attribute("taskType","targetArea"));
	attributeFinder->setQueryAttributes(tmpAttributes);
	root->accept(attributeFinder);
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(attributeFinder->getMatchingNodes().size()));
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[0]).getAttributes()[0] == Attribute("shapeType","Cylinder"));
	CPPUNIT_ASSERT((*attributeFinder->getMatchingNodes()[0]).getAttributes()[1] == Attribute("taskType","targetArea"));
	CPPUNIT_ASSERT_EQUAL(geode4Id, (*attributeFinder->getMatchingNodes()[0]).getId());

	delete attributeFinder;


}

void SceneGraphNodesTest::testOutdatedDataDeleter() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+----------
	 *        |          |         |
	 *       tf1        tf2       tf3
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *             tf4
	 */
	Id rootId = 0;
	Id tf1Id = 1;
	Id tf2Id = 2;
	Id tf3Id = 3;
	Id tf4Id = 4;

	Group::GroupPtr root(new Group());
	root->setId(rootId);

	rsg::Transform::TransformPtr tf1(new rsg::Transform());
	tf1->setId(tf1Id);
	rsg::Transform::TransformPtr tf2(new rsg::Transform());
	tf2->setId(tf2Id);
	rsg::Transform::TransformPtr tf3(new rsg::Transform());
	tf3->setId(tf3Id);
	rsg::Transform::TransformPtr tf4(new rsg::Transform());
	tf4->setId(tf4Id);

	root->addChild(tf1);
	root->addChild(tf2);
	root->addChild(tf3);
	tf1->addChild(tf4);
	tf2->addChild(tf4);

	/* check that graph actually is as we expect */
	CPPUNIT_ASSERT_EQUAL(0u, root->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(3u, root->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(1u, tf1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, tf1->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(1u, tf2->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, tf2->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(1u, tf3->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf3->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(2u, tf4->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf4->getNumberOfChildren());

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); //Translation coefficients

	/* configure cache size */
	tf1->setMaxHistoryDuration(TimeStamp(10.0));
	tf2->setMaxHistoryDuration(TimeStamp(10.0));
	tf3->setMaxHistoryDuration(TimeStamp(10.0));
	tf4->setMaxHistoryDuration(TimeStamp(10.0));


	/* update tf data such that all have 2 elements in history cache exept for tf3 => tf3 will b deleted */
	tf1->insertTransform(transform123, TimeStamp(0.0));
	tf1->insertTransform(transform123, TimeStamp(5.0));

	tf2->insertTransform(transform123, TimeStamp(0.0));
	tf2->insertTransform(transform123, TimeStamp(5.0));

	tf3->insertTransform(transform123, TimeStamp(0.0));

	tf4->insertTransform(transform123, TimeStamp(0.0));
	tf4->insertTransform(transform123, TimeStamp(5.0));

	OutdatedDataDeleter* deleter = new OutdatedDataDeleter();
	deleter->setPerformAutomaticHistoryUpdates(false);
	CPPUNIT_ASSERT(deleter->getPerformAutomaticHistoryUpdates() == false);
	deleter->setMinHistoryLength(2u);
	CPPUNIT_ASSERT_EQUAL(2u, deleter->getMinHistoryLength());
	root->accept(deleter);

	/* check if tf3 is deleted */
	CPPUNIT_ASSERT_EQUAL(0u, root->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(2u, root->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(1u, tf1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, tf1->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(1u, tf2->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, tf2->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(0u, tf3->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf3->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(2u, tf4->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf4->getNumberOfChildren());

	/* trigger deletion of tf2 as cach will only have 1 element */
	tf2->insertTransform(transform123, TimeStamp(20.0));
	root->accept(deleter);

	/* check if tf2 is deleted */
	CPPUNIT_ASSERT_EQUAL(0u, root->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, root->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(1u, tf1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(1u, tf1->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(0u, tf2->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf2->getNumberOfChildren()); //1?

	CPPUNIT_ASSERT_EQUAL(0u, tf3->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf3->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(1u, tf4->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf4->getNumberOfChildren());

	/* trigger deletion of tf1 as cache will only have 1 element */
	tf1->insertTransform(transform123, TimeStamp(20.0));
	root->accept(deleter);

	CPPUNIT_ASSERT_EQUAL(0u, root->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, root->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(0u, tf1->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf1->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(0u, tf2->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf2->getNumberOfChildren()); //1?

	CPPUNIT_ASSERT_EQUAL(0u, tf3->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf3->getNumberOfChildren());

	CPPUNIT_ASSERT_EQUAL(0u, tf4->getNumberOfParents());
	CPPUNIT_ASSERT_EQUAL(0u, tf4->getNumberOfChildren());

	delete deleter;
}

void SceneGraphNodesTest::testIdGenerator(){
	IIdGenerator* idGenerator = new SimpleIdGenerator();

	Id rootId = 1u;
	Id id2 = 2u;
	Id id3 = 3u;
	Id id4 = 4u;
	Id id5 = 5u;
	Id id6 = 6u;
	Id id7 = 7u;
	Id id9 = 9u;

	CPPUNIT_ASSERT_EQUAL(rootId, idGenerator->getRootId());
	CPPUNIT_ASSERT_EQUAL(id2, idGenerator->getNextValidId());
	CPPUNIT_ASSERT_EQUAL(rootId, idGenerator->getRootId());
	CPPUNIT_ASSERT_EQUAL(id3, idGenerator->getNextValidId());
	CPPUNIT_ASSERT_EQUAL(id4, idGenerator->getNextValidId());
	CPPUNIT_ASSERT_EQUAL(id5, idGenerator->getNextValidId());
	CPPUNIT_ASSERT_EQUAL(id6, idGenerator->getNextValidId());

	CPPUNIT_ASSERT(!idGenerator->removeIdFromPool(1u));
	CPPUNIT_ASSERT(!idGenerator->removeIdFromPool(2u));
	CPPUNIT_ASSERT(!idGenerator->removeIdFromPool(3u));
	CPPUNIT_ASSERT(!idGenerator->removeIdFromPool(4u));
	CPPUNIT_ASSERT(!idGenerator->removeIdFromPool(5u));
	CPPUNIT_ASSERT(!idGenerator->removeIdFromPool(6u));

	CPPUNIT_ASSERT_EQUAL(id7, idGenerator->getNextValidId());

	CPPUNIT_ASSERT(!idGenerator->removeIdFromPool(7u));
	CPPUNIT_ASSERT(idGenerator->removeIdFromPool(8u)); // this should work

	CPPUNIT_ASSERT_EQUAL(id9, idGenerator->getNextValidId());
	CPPUNIT_ASSERT(!idGenerator->removeIdFromPool(9u));

	CPPUNIT_ASSERT(idGenerator->removeIdFromPool(123u)); // this should work too

	delete idGenerator;
}

void SceneGraphNodesTest::testSceneGraphFacade(){
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+----------
	 *        |          |         |
	 *       tf1        group2    node3
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *            geode4
	 */

	/* will be assigned later */
	Id rootId = 0;
	Id tf1Id = 0;
	Id group2Id = 0;
	Id node3Id = 0;
	Id geode4Id = 0;
	const Id invalidId = 100000000;

	SceneGraphFacade scene(new UuidGenerator(1u));// provide a fixed root ID
	vector<Attribute> tmpAttributes;
	vector<Id> resultParentIds;
	vector<Id> resultChildIds;

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						//Translation coefficients
	TimeStamp dummyTime(20);

	/* test root */
	Id expecetedRootId = 1u;
	CPPUNIT_ASSERT_EQUAL(expecetedRootId, scene.getRootId()); //assumption: uses SimpleIdGenerator
	rootId = scene.getRootId();

	resultParentIds.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()) );
	CPPUNIT_ASSERT(scene.getNodeParents(scene.getRootId(), resultParentIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()) );
	CPPUNIT_ASSERT(!scene.getNodeParents(invalidId, resultParentIds));

	tmpAttributes.clear();
	CPPUNIT_ASSERT(scene.getNodeAttributes(rootId, tmpAttributes));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(tmpAttributes.size()));
	CPPUNIT_ASSERT(!scene.getNodeAttributes(invalidId, tmpAttributes));

	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","root"));
	CPPUNIT_ASSERT(scene.setNodeAttributes(rootId, tmpAttributes));

	tmpAttributes.clear();
	CPPUNIT_ASSERT(scene.getNodeAttributes(rootId, tmpAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(tmpAttributes.size()));
	CPPUNIT_ASSERT(tmpAttributes[0] == Attribute("name","root"));
	CPPUNIT_ASSERT(!scene.setNodeAttributes(invalidId, tmpAttributes));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(rootId, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT(!scene.getGroupChildren(invalidId, resultChildIds));


	/* tf1Id */
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","tf"));
	CPPUNIT_ASSERT(tf1Id == 0);
	CPPUNIT_ASSERT(scene.addTransformNode(rootId, tf1Id, tmpAttributes, transform123, dummyTime + TimeStamp(0, Units::MilliSecond)));
	CPPUNIT_ASSERT(tf1Id != 0);
	CPPUNIT_ASSERT(!scene.addTransformNode(invalidId, tf1Id, tmpAttributes, transform123, dummyTime + TimeStamp(2, Units::MilliSecond)));

	resultParentIds.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()) );
	CPPUNIT_ASSERT(scene.getNodeParents(tf1Id, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT_EQUAL(rootId, static_cast<Id>(resultParentIds[0]));
	CPPUNIT_ASSERT(!scene.getNodeParents(invalidId, resultParentIds));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(rootId, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT_EQUAL(tf1Id, static_cast<Id>(resultChildIds[0]));


	/* group2Id */
	tmpAttributes.clear();
	CPPUNIT_ASSERT(group2Id == 0);
	CPPUNIT_ASSERT(scene.addGroup(rootId, group2Id, tmpAttributes));
	CPPUNIT_ASSERT(group2Id != 0);

	Id tmpId = group2Id;
	resultParentIds.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()) );
	CPPUNIT_ASSERT(scene.getNodeParents(group2Id, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT_EQUAL(rootId, static_cast<Id>(resultParentIds[0]));
	CPPUNIT_ASSERT(!scene.addGroup(invalidId, group2Id, tmpAttributes));
	CPPUNIT_ASSERT_EQUAL(tmpId, group2Id); // check if there is a side effect

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(rootId, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT_EQUAL(tf1Id, static_cast<Id>(resultChildIds[0]));
	CPPUNIT_ASSERT_EQUAL(group2Id, static_cast<Id>(resultChildIds[1]));

	/* node3Id */
	tmpAttributes.clear();
	CPPUNIT_ASSERT(node3Id == 0);
	CPPUNIT_ASSERT(scene.addNode(rootId, node3Id, tmpAttributes));
	CPPUNIT_ASSERT(node3Id != 0);

	resultParentIds.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()) );
	CPPUNIT_ASSERT(scene.getNodeParents(node3Id, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT_EQUAL(rootId, static_cast<Id>(resultParentIds[0]));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(rootId, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT_EQUAL(tf1Id, static_cast<Id>(resultChildIds[0]));
	CPPUNIT_ASSERT_EQUAL(group2Id, static_cast<Id>(resultChildIds[1]));
	CPPUNIT_ASSERT_EQUAL(node3Id, static_cast<Id>(resultChildIds[2]));

	CPPUNIT_ASSERT(!scene.addNode(invalidId, node3Id, tmpAttributes));

	/*geode4Id */
	tmpAttributes.clear();
	CPPUNIT_ASSERT(geode4Id == 0);
	Cylinder::CylinderPtr cylinder1(new Cylinder(0.2,0.1));
	tmpAttributes.push_back(Attribute("shapeType","Cylinder"));
	CPPUNIT_ASSERT(scene.addGeometricNode(tf1Id, geode4Id, tmpAttributes, cylinder1, dummyTime));
	CPPUNIT_ASSERT(geode4Id != 0);

	resultParentIds.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()) );
	CPPUNIT_ASSERT(scene.getNodeParents(geode4Id, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT_EQUAL(tf1Id, static_cast<Id>(resultParentIds[0]));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(tf1Id, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT_EQUAL(geode4Id, static_cast<Id>(resultChildIds[0]));

	CPPUNIT_ASSERT(!scene.getGroupChildren(geode4Id, resultChildIds));
	CPPUNIT_ASSERT(!scene.addGeometricNode(invalidId, geode4Id, tmpAttributes, cylinder1, dummyTime));

	/* make it a graph */
	CPPUNIT_ASSERT(scene.addParent(geode4Id, group2Id));

	resultParentIds.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()) );
	CPPUNIT_ASSERT(scene.getNodeParents(geode4Id, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT_EQUAL(tf1Id, static_cast<Id>(resultParentIds[0]));
	CPPUNIT_ASSERT_EQUAL(group2Id, static_cast<Id>(resultParentIds[1]));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(tf1Id, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT_EQUAL(geode4Id, static_cast<Id>(resultChildIds[0]));
	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group2Id, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT_EQUAL(geode4Id, static_cast<Id>(resultChildIds[0]));

	CPPUNIT_ASSERT(!scene.getGroupChildren(node3Id, resultChildIds));

	/* get transform data */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;
	CPPUNIT_ASSERT(scene.getTransform(tf1Id, dummyTime, resultTransform));

	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	CPPUNIT_ASSERT(!scene.getTransform(invalidId, dummyTime, resultTransform));
	CPPUNIT_ASSERT(!scene.getTransform(rootId, dummyTime, resultTransform));
	CPPUNIT_ASSERT(!scene.getTransform(group2Id, dummyTime, resultTransform));
	CPPUNIT_ASSERT(!scene.getTransform(node3Id, dummyTime, resultTransform));
	CPPUNIT_ASSERT(!scene.getTransform(geode4Id, dummyTime, resultTransform));

	/* get geometry data */
	Shape::ShapePtr resultShape;
	CPPUNIT_ASSERT(scene.getGeometry(geode4Id, resultShape, dummyTime));
	Cylinder::CylinderPtr resultCylinder;
	resultCylinder = boost::dynamic_pointer_cast<Cylinder>(resultShape);
	CPPUNIT_ASSERT(resultCylinder != 0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2, resultCylinder->getRadius(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.1, resultCylinder->getHeight(), maxTolerance);

	CPPUNIT_ASSERT(!scene.getGeometry(invalidId, resultShape, dummyTime));
	CPPUNIT_ASSERT(!scene.getGeometry(rootId, resultShape, dummyTime));
	CPPUNIT_ASSERT(!scene.getGeometry(tf1Id, resultShape, dummyTime));
	CPPUNIT_ASSERT(!scene.getGeometry(group2Id, resultShape, dummyTime));
	CPPUNIT_ASSERT(!scene.getGeometry(node3Id, resultShape, dummyTime));

	/* set new tf data */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             4,5,6)); 						//Translation coefficients


	CPPUNIT_ASSERT(scene.setTransform(tf1Id, transform456, dummyTime  + TimeStamp(3, Units::MilliSecond)));

	CPPUNIT_ASSERT(scene.getTransform(tf1Id, dummyTime  + TimeStamp(3, Units::MilliSecond), resultTransform));
	matrixPtr = resultTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[14], maxTolerance);

	CPPUNIT_ASSERT(!scene.getTransform(invalidId, dummyTime, resultTransform));
	CPPUNIT_ASSERT(!scene.getTransform(rootId, dummyTime, resultTransform));
	CPPUNIT_ASSERT(!scene.getTransform(geode4Id, dummyTime, resultTransform));
	CPPUNIT_ASSERT(!scene.getTransform(group2Id, dummyTime, resultTransform));
	CPPUNIT_ASSERT(!scene.getTransform(node3Id, dummyTime, resultTransform));

	/* find nodes by attributes */
	vector<Id> resultIds;

	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","root"));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodes(tmpAttributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT_EQUAL(rootId, resultIds[0]);

	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","tf"));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodes(tmpAttributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT_EQUAL(tf1Id, resultIds[0]);

	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("shapeType","Cylinder"));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodes(tmpAttributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT_EQUAL(geode4Id, resultIds[0]);


	/*
	 * now we delete the nodes
	 */

	/* deletion of geode4: preconditions */
	resultParentIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(geode4Id, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultParentIds.size()));

	/* deletion of geode4: actual deletion */
	CPPUNIT_ASSERT(scene.deleteNode(geode4Id));

	/* after deletion:
	 *                 root
	 *                   |
	 *        -----------+----------
	 *        |          |         |
	 *       tf1        group2    node3
	 */

	/* deletion of geode4: postconditions */
	resultParentIds.clear();
	CPPUNIT_ASSERT(!scene.getNodeParents(geode4Id, resultParentIds)); // now it should fail is it is not present anymore...
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT(!scene.getNodeAttributes(geode4Id, tmpAttributes));
	CPPUNIT_ASSERT(!scene.setNodeAttributes(geode4Id, tmpAttributes));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(tf1Id, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultChildIds.size()));
	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group2Id, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultChildIds.size()));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(rootId, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT_EQUAL(tf1Id, static_cast<Id>(resultChildIds[0]));
	CPPUNIT_ASSERT_EQUAL(group2Id, static_cast<Id>(resultChildIds[1]));
	CPPUNIT_ASSERT_EQUAL(node3Id, static_cast<Id>(resultChildIds[2]));


	/* deletion of tf1: preconditions */
	resultParentIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(tf1Id, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultParentIds.size()));

	/* deletion of tf1: actual deletion */
	CPPUNIT_ASSERT(scene.deleteNode(tf1Id));

	/* after deletion:
	 *                 root
	 *                   |
	 *                   +----------
	 *                   |         |
	 *                group2    node3
	 */

	/* deletion of tf1: postconditions */
	resultParentIds.clear();
	CPPUNIT_ASSERT(!scene.getNodeParents(tf1Id, resultParentIds)); // now it should fail is it is not present anymore...
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT(!scene.getNodeAttributes(tf1Id, tmpAttributes));
	CPPUNIT_ASSERT(!scene.setNodeAttributes(tf1Id, tmpAttributes));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group2Id, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultChildIds.size()));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(rootId, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT_EQUAL(group2Id, static_cast<Id>(resultChildIds[0]));
	CPPUNIT_ASSERT_EQUAL(node3Id, static_cast<Id>(resultChildIds[1]));

	/* deletion of group2: preconditions */
	resultParentIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(group2Id, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultParentIds.size()));

	/* deletion of group2: actual deletion */
	CPPUNIT_ASSERT(scene.deleteNode(group2Id));

	/* after deletion:
	 *                 root
	 *                   |
	 *                   +----------
	 *                             |
	 *                           node3
	 */

	/* deletion of group2: postconditions */
	resultParentIds.clear();
	CPPUNIT_ASSERT(!scene.getNodeParents(group2Id, resultParentIds)); // now it should fail is it is not present anymore...
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT(!scene.getNodeAttributes(group2Id, tmpAttributes));
	CPPUNIT_ASSERT(!scene.setNodeAttributes(group2Id, tmpAttributes));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(rootId, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultChildIds.size()));
	CPPUNIT_ASSERT_EQUAL(node3Id, static_cast<Id>(resultChildIds[0]));


	/* deletion of node3: preconditions */
	resultParentIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(node3Id, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultParentIds.size()));

	/* deletion of node3: actual deletion */
	CPPUNIT_ASSERT(scene.deleteNode(node3Id));

	/* after deletion:
	 *                 root
	 */

	/* deletion of node3: postconditions */
	resultParentIds.clear();
	CPPUNIT_ASSERT(!scene.getNodeParents(node3Id, resultParentIds)); // now it should fail is it is not present anymore...
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT(!scene.getNodeAttributes(node3Id, tmpAttributes));
	CPPUNIT_ASSERT(!scene.setNodeAttributes(node3Id, tmpAttributes));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(rootId, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultChildIds.size()));

	/* trying to delete root */
	CPPUNIT_ASSERT(!scene.deleteNode(rootId));

	resultChildIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(rootId, resultChildIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultChildIds.size()));

}

void SceneGraphNodesTest::testSceneGraphFacadeTransforms() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *            root
	 *              |
	 *        ------+-----
	 *        |          |
	 *       tf1        tf2
	 *        |          |
	 *       tf3        group4
	 *        |          |
	 *        +----  ----+
	 *            |  |
	 *            node5
	 */
	Id tf1Id = 0;
	Id tf2Id = 0;
	Id tf3Id = 0;
	Id group4Id = 0;
	Id node5Id = 0;

	const double* desiredMatrixData;
	const double* resultMatrixData;

	TimeStamp dummyTime(1.0);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform001(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0,0,-1));
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						//Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform654(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             6,5,4)); 						//Translation coefficients

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform789(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             7,8,9)); 						//Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr expectedTransform(new HomogeneousMatrix44());
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identity(new HomogeneousMatrix44());

	SceneGraphFacade scene;
	vector<Attribute> tmpAttributes;

	CPPUNIT_ASSERT(scene.addTransformNode(scene.getRootId(), tf1Id, tmpAttributes, transform001, dummyTime));
	CPPUNIT_ASSERT(scene.addTransformNode(scene.getRootId(), tf2Id, tmpAttributes, transform123, dummyTime));
	CPPUNIT_ASSERT(scene.addTransformNode(tf1Id, tf3Id, tmpAttributes, transform654, dummyTime));
	CPPUNIT_ASSERT(scene.addGroup(tf2Id, group4Id, tmpAttributes));
	CPPUNIT_ASSERT(scene.addNode(tf3Id, node5Id, tmpAttributes));
	CPPUNIT_ASSERT(scene.addParent(node5Id, group4Id));

//	brics_3d::rsg::DotGraphGenerator dotGraphGenerator; //Debug
//	scene.executeGraphTraverser(&dotGraphGenerator);
//	cout << "testSceneGraphFacadeTransforms" << endl;
//	cout << dotGraphGenerator.getDotGraph();


	/*check root to tf1*/
	CPPUNIT_ASSERT(scene.getTransformForNode(tf1Id, scene.getRootId(), dummyTime, resultTransform));
	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = transform001->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/*check root to tf3*/
	CPPUNIT_ASSERT(scene.getTransformForNode(tf3Id, scene.getRootId(), dummyTime, resultTransform));
	*expectedTransform = *identity;
	*expectedTransform = *( (*expectedTransform) * (*transform001) );
	*expectedTransform = *( (*expectedTransform) * (*transform654) );

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/*check root to node5 */
	CPPUNIT_ASSERT(scene.getTransformForNode(node5Id, scene.getRootId(), dummyTime, resultTransform));
	*expectedTransform = *identity;
	*expectedTransform = *( (*expectedTransform) * (*transform001) );
	*expectedTransform = *( (*expectedTransform) * (*transform654) );

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/*check root to tf2 */
	CPPUNIT_ASSERT(scene.getTransformForNode(tf2Id, scene.getRootId(), dummyTime, resultTransform));
	*expectedTransform = *identity;
	*expectedTransform = *( (*expectedTransform) * (*transform123) );

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/*check root to group4 */
	CPPUNIT_ASSERT(scene.getTransformForNode(group4Id, scene.getRootId(), dummyTime, resultTransform));
	*expectedTransform = *identity;
	*expectedTransform = *( (*expectedTransform) * (*transform123) );

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/* tf1 to tf3 (tf3 expressed in tf1) */
	CPPUNIT_ASSERT(scene.getTransformForNode(tf3Id, tf1Id, dummyTime, resultTransform));
	*expectedTransform = *transform654;

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/* tf3 to tf1  (tf1 expressed in tf3) */
	CPPUNIT_ASSERT(scene.getTransformForNode(tf1Id, tf3Id, dummyTime, resultTransform));
	*expectedTransform = *transform654;
	expectedTransform->inverse();

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/* tf1 expressed in tf 2 */
	CPPUNIT_ASSERT(scene.getTransformForNode(tf1Id, tf2Id, dummyTime, resultTransform));
	*expectedTransform = *transform123;
	expectedTransform->inverse();
	*expectedTransform = *( (*expectedTransform) * (*transform001) );

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/* tf2 expressed in tf1 */
	CPPUNIT_ASSERT(scene.getTransformForNode(tf2Id, tf1Id, dummyTime, resultTransform));
	*expectedTransform = *transform001;
	expectedTransform->inverse();
	*expectedTransform = *( (*expectedTransform) * (*transform123) );

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/*
	 * check how if updates to the trasforms are accounted for
	 */
	TimeStamp dummyTime2 = dummyTime + TimeStamp(1, Units::MilliSecond);
	CPPUNIT_ASSERT(scene.setTransform(tf3Id, transform789, dummyTime2));

	/*check root to node5 (involves tf3) @ dummyTime */
	CPPUNIT_ASSERT(scene.getTransformForNode(node5Id, scene.getRootId(), dummyTime, resultTransform)); // query stll at dummy time -> result shoudlbe unschneged
	*expectedTransform = *identity;
	*expectedTransform = *( (*expectedTransform) * (*transform001) );
	*expectedTransform = *( (*expectedTransform) * (*transform654) );

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/*check root to node5 (involves tf3) @ dummyTime2 */
	CPPUNIT_ASSERT(scene.getTransformForNode(node5Id, scene.getRootId(), dummyTime2, resultTransform)); // query stll at dummy time -> result shoudlbe unschneged
	*expectedTransform = *identity;
	*expectedTransform = *( (*expectedTransform) * (*transform001) );
	*expectedTransform = *( (*expectedTransform) * (*transform789) ); // this is the update @ dummyTime2

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);


	CPPUNIT_ASSERT(!scene.setTransform(tf3Id, transform123, dummyTime2));

	/*check root to node5 (involved tf3) @ dummyTime */
	CPPUNIT_ASSERT(scene.getTransformForNode(node5Id, scene.getRootId(), dummyTime, resultTransform)); // query stll at dummy time -> result shoudlbe unschneged
	*expectedTransform = *identity;
	*expectedTransform = *( (*expectedTransform) * (*transform001) );
	*expectedTransform = *( (*expectedTransform) * (*transform654) );

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/*check root to node5 (involved tf3) @ dummyTime2 */
	CPPUNIT_ASSERT(scene.getTransformForNode(node5Id, scene.getRootId(), dummyTime2, resultTransform)); // query stll at dummy time -> result shoudlbe unschneged
	*expectedTransform = *identity;
	*expectedTransform = *( (*expectedTransform) * (*transform001) );
	*expectedTransform = *( (*expectedTransform) * (*transform789) );

	resultMatrixData = resultTransform->getRawData();
	desiredMatrixData = expectedTransform->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[12], resultMatrixData[12], maxTolerance); //check (just) translation
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[13], resultMatrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(desiredMatrixData[14], resultMatrixData[14], maxTolerance);

	/*
	 * check if double insertion at the same time stamp are prevented
	 */
}

void SceneGraphNodesTest::testPointCloud() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+----------
	 *        |                    |
	 *       pc1                  tf2
	 *                             |
	 *                            pc3
	 *
	 */
	Id rootId = 0;
	Id pc1Id = 1;
	Id tf2Id = 2;
	Id pc3Id = 3;

	Group::GroupPtr root(new Group());
	root->setId(rootId);

	/*
	 * the real data/ implementation
	 */
	brics_3d::PointCloud3D::PointCloud3DPtr pc1_data(new brics_3d::PointCloud3D());
	brics_3d::PointCloud3D::PointCloud3DPtr pc3_data(new brics_3d::PointCloud3D());

	/*SceneGraphNodesTest::testSceneGraphFacade
	 * generic data containers
	 */

	// NODE with raw pointer
//	brics_3d::PointCloud3D::PointCloud3DPtr pc1_test_data(new brics_3d::PointCloud3D());
//	rsg::PointCloud<brics_3d::PointCloud3D>* pc1_test = new rsg::PointCloud<brics_3d::PointCloud3D>();
//	pc1_test->data = pc1_test_data;
//	pc1_test->data->addPoint(Point3D(1,2,3));
	//std::cout << *(pc1_test->data);

	// NODE with manual boost pointer
	boost::shared_ptr<PointCloud<brics_3d::PointCloud3D> >  pc1(new rsg::PointCloud<brics_3d::PointCloud3D>());
	pc1->data = pc1_data;
	pc1->data->addPoint(Point3D(1,1,1));
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, (*pc1->data->getPointCloud())[0].getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, (*pc1->data->getPointCloud())[0].getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, (*pc1->data->getPointCloud())[0].getZ(), maxTolerance);
//	std::cout << *(pc1->data);

	// NODE with
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pc3(new rsg::PointCloud<brics_3d::PointCloud3D>());
	pc3->data = pc3_data;
	pc3->data->addPoint(Point3D(2,2,2));
	pc3->data->addPoint(Point3D(3,3,3));
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, (*pc3->data->getPointCloud())[0].getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, (*pc3->data->getPointCloud())[0].getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, (*pc3->data->getPointCloud())[0].getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, (*pc3->data->getPointCloud())[1].getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, (*pc3->data->getPointCloud())[1].getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, (*pc3->data->getPointCloud())[1].getZ(), maxTolerance);
//	std::cout << *(pc3->data);

	/*
	 * geometric nodes
	 */
	GeometricNode::GeometricNodePtr pcGeode1(new GeometricNode());
	pcGeode1->setId(pc1Id);
	GeometricNode::GeometricNodePtr pcGeode3(new GeometricNode());
	pcGeode1->setId(pc3Id);

	pcGeode1->setShape(pc1);
	pcGeode3->setShape(pc3);

	rsg::Transform::TransformPtr tf2(new rsg::Transform());
	tf2->setId(tf2Id);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr someTransform(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             10,10,10)); 						//Translation coefficients
	tf2->insertTransform(someTransform, TimeStamp(0.0));

	/*
	 * finally construct some scenegraph
	 */
	root->addChild(pcGeode1);
	root->addChild(tf2);
	tf2->addChild(pcGeode3);

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;
	resultTransform	= getGlobalTransform(pcGeode3, TimeStamp(0.0));
	//cout << *resultTransform;

	Shape::ShapePtr resultShape;
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr resultPointCloud;
	resultShape = pcGeode3->getShape();
	resultPointCloud = boost::dynamic_pointer_cast<rsg::PointCloud<brics_3d::PointCloud3D> >(resultShape);
	CPPUNIT_ASSERT(resultPointCloud != 0);
	CPPUNIT_ASSERT_EQUAL(2u, resultPointCloud->data->getSize());
	for (unsigned int index = 0; index < resultPointCloud->data->getSize(); ++index) {
		brics_3d::Point3D resultPoint;
		resultPoint = (*resultPointCloud->data->getPointCloud())[index];
		//cout << "raw Point value = " << resultPoint;
		resultPoint.homogeneousTransformation(resultTransform.get());
		//cout << "transform Point value = " << resultPoint << endl;
	}

}

void SceneGraphNodesTest::testUpdateObserver() {
	SceneGraphFacade scene;
	Id dymmyId = 0;
	Id tfId = 0;
	Id geodeId = 0;
	TimeStamp dummyTime(20);
	vector<Attribute> tmpAttributes;
	vector<Id> resultParentIds;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						//Translation coefficients

	Cylinder::CylinderPtr cylinder1(new Cylinder(0.2,0.1));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty123(new CovarianceMatrix66(0.91,0.92,0.93, 0.1,0.2,0.3));

	MyObserver testObserver;
	scene.setCallObserversEvenIfErrorsOccurred(false); // Turn off observer increments cause by wrong inserts (yes we try to insert wrong stuffer here...)


	CPPUNIT_ASSERT(scene.attachUpdateObserver(&testObserver) == true);

	CPPUNIT_ASSERT_EQUAL(0, testObserver.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setUncertainTransformCounter);

	CPPUNIT_ASSERT(scene.addNode(scene.getRootId(), dymmyId, tmpAttributes) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setUncertainTransformCounter);



	CPPUNIT_ASSERT(scene.addGroup(scene.getRootId(), dymmyId, tmpAttributes) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setUncertainTransformCounter);


	CPPUNIT_ASSERT(scene.addTransformNode(scene.getRootId(), tfId, tmpAttributes, transform123, dummyTime) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setUncertainTransformCounter);


	CPPUNIT_ASSERT(scene.addGeometricNode(scene.getRootId(), geodeId, tmpAttributes, cylinder1, dummyTime) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setUncertainTransformCounter);


	CPPUNIT_ASSERT(scene.setNodeAttributes(scene.getRootId(), tmpAttributes) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setUncertainTransformCounter);

	CPPUNIT_ASSERT(scene.setTransform(tfId, transform123, dummyTime) == false);
	CPPUNIT_ASSERT(scene.setTransform(tfId, transform123, dummyTime + TimeStamp(1, Units::MilliSecond)) == true); // NOTE: we cannot use the same time stamp
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setUncertainTransformCounter);

	CPPUNIT_ASSERT(scene.addUncertainTransformNode(scene.getRootId(), tfId, tmpAttributes, transform123, uncertainty123, dummyTime) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.setUncertainTransformCounter);

	CPPUNIT_ASSERT(scene.setUncertainTransform(tfId, transform123, uncertainty123, dummyTime) == false);
	CPPUNIT_ASSERT(scene.setUncertainTransform(tfId, transform123, uncertainty123, dummyTime + TimeStamp(1, Units::MilliSecond)) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setUncertainTransformCounter);

	CPPUNIT_ASSERT(scene.deleteNode(tfId) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setUncertainTransformCounter);


	CPPUNIT_ASSERT(scene.addParent(geodeId, scene.getRootId()) == true); //actually same relation twice
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setUncertainTransformCounter);


	CPPUNIT_ASSERT(scene.removeParent(geodeId, scene.getRootId()) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setUncertainTransformCounter);


	/* detach -> no further updates are expected */
	CPPUNIT_ASSERT(scene.detachUpdateObserver(&testObserver) == true);
	CPPUNIT_ASSERT(scene.addGroup(scene.getRootId(), dymmyId, tmpAttributes) == true);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addNodeCounter); //poscondition
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.removeParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, testObserver.setUncertainTransformCounter);


	CPPUNIT_ASSERT(scene.detachUpdateObserver(&testObserver) == false);

	DotVisualizer debugObserver(&scene);
	CPPUNIT_ASSERT(scene.attachUpdateObserver(&debugObserver) == true);
	//CPPUNIT_ASSERT(scene.addNode(scene.getRootId(), dymmyId, tmpAttributes) == true); //trigger an update
	CPPUNIT_ASSERT(scene.detachUpdateObserver(&debugObserver) == true);
}

void SceneGraphNodesTest::testDotGraphGenerator() {
	brics_3d::rsg::SceneGraphFacade scene;			// The 3D world model handle
	vector<Attribute> attributes;	// with this one we can attach attibutes / tags to the nodes

	/* Some node Ids we would like to remember */
	Id dummyId = 0;
	Id tfId = 0; // We will use this multible times
	Id pointCloudId = 0;
	Id sensorGroupId = 0;
	Id filteredGroupId = 0;
	Id sceneObjectGroupId = 0;
	Id robotGroupId = 0;
	Id tableGroupId = 0;
	Id plateGroupId = 0;
	Id boxGroupId1 = 0;
	Id boxGroupId2 = 0;
	Id geometryId = 0;

	/* Some (dummy) data to be use within the scenegraph */
	brics_3d::rsg::TimeStamp dummyTime(0);
	brics_3d::rsg::TimeStamp t1(1.0);
	brics_3d::rsg::TimeStamp t2(2.0);
	brics_3d::rsg::TimeStamp t3(3.0);
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						// Translation coefficients

	brics_3d::PointCloud3D::PointCloud3DPtr dummyPointCloud(new brics_3d::PointCloud3D());		// An empty BRICS_3D point cloud as dummy
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr dummyPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>()); // Containter used within scenegraph
	dummyPointCloudContainer->data=dummyPointCloud;		// Fill container with the point cloud data

	brics_3d::rsg::Box::BoxPtr dummyBox(new brics_3d::rsg::Box());

	/*
	 * Set up the scenegraph
	 */

	/* nodes for raw data from sensor */
	attributes.clear();
	attributes.push_back(Attribute("name","sensor_tf"));
	scene.addTransformNode(scene.getRootId(), tfId, attributes, transform123, dummyTime);

	attributes.clear();
	attributes.push_back(Attribute("name","sensor"));
	scene.addGroup(tfId, sensorGroupId, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","pc1_tf"));
	scene.addTransformNode(sensorGroupId, tfId, attributes, transform123, t1);

	attributes.clear();
	attributes.push_back(Attribute("name","point_cloud_1"));
	scene.addGeometricNode(tfId, pointCloudId, attributes, dummyPointCloudContainer, t1);

	attributes.clear();
	attributes.push_back(Attribute("name","pc2_tf"));
	scene.addTransformNode(sensorGroupId, tfId, attributes, transform123, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","point_cloud_2"));
	scene.addGeometricNode(tfId, pointCloudId, attributes, dummyPointCloudContainer, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","pc3_tf"));
	scene.addTransformNode(sensorGroupId, tfId, attributes, transform123, t3);

	attributes.clear();
	attributes.push_back(Attribute("name","point_cloud_3"));
	scene.addGeometricNode(tfId, pointCloudId, attributes, dummyPointCloudContainer, t3);

	/* nodes for processed (filtered) data */
	attributes.clear();
	attributes.push_back(Attribute("name","filtered_tf"));
	scene.addTransformNode(scene.getRootId(), tfId, attributes, transform123, dummyTime);

	attributes.clear();
	attributes.push_back(Attribute("name","filtered_data"));
	scene.addGroup(tfId, filteredGroupId, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","pc1_filterd_tf"));
	scene.addTransformNode(filteredGroupId, tfId, attributes, transform123, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","pc_filterd_1"));
	scene.addGeometricNode(tfId, pointCloudId, attributes, dummyPointCloudContainer, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","pc2_filtered_tf"));
	scene.addTransformNode(filteredGroupId, tfId, attributes, transform123, t3);

	attributes.clear();
	attributes.push_back(Attribute("name","pc_filtered_2"));
	scene.addGeometricNode(tfId, pointCloudId, attributes, dummyPointCloudContainer, t3);

	/* nodes for hight level scene objects */
	attributes.clear();
	attributes.push_back(Attribute("name","scene_objects_tf"));
	scene.addTransformNode(scene.getRootId(), tfId, attributes, transform123, dummyTime);

	attributes.clear();
	attributes.push_back(Attribute("name","scene_objects"));
	attributes.push_back(Attribute("note","high level objects"));
	scene.addGroup(tfId, sceneObjectGroupId, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","table_tf"));
	scene.addTransformNode(sceneObjectGroupId, tfId, attributes, transform123, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","Table"));
	scene.addGroup(tfId, tableGroupId, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","table_to_plate_tf"));
	scene.addTransformNode(tableGroupId, tfId, attributes, transform123, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","Plate"));
	scene.addGroup(tfId, plateGroupId, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","table_to_box_tf"));
	scene.addTransformNode(plateGroupId, tfId, attributes, transform123, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","Box"));
	attributes.push_back(Attribute("color","red"));
	scene.addGroup(tfId, boxGroupId1, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","table_to_leg1_tf"));
	scene.addTransformNode(tableGroupId, tfId, attributes, transform123, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","Leg1"));
	scene.addGroup(tfId, plateGroupId, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","table_to_leg2_tf"));
	scene.addTransformNode(tableGroupId, tfId, attributes, transform123, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","Leg2"));
	scene.addGroup(tfId, plateGroupId, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","cylinder_tf"));
	scene.addTransformNode(sceneObjectGroupId, tfId, attributes, transform123, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","Cylinder"));
	attributes.push_back(Attribute("color","blue"));
	scene.addGroup(tfId, dummyId, attributes);


	attributes.clear();
	attributes.push_back(Attribute("name","box_tf"));
	scene.addTransformNode(sceneObjectGroupId, tfId, attributes, transform123, t2);

	attributes.clear();
	attributes.push_back(Attribute("name","Box"));
	attributes.push_back(Attribute("color","red"));
	scene.addGroup(tfId, boxGroupId1, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","Geometry"));
	scene.addGeometricNode(boxGroupId1, geometryId, attributes, dummyBox, dummyTime);

	attributes.clear();
	attributes.push_back(Attribute("name","box_tf1"));
	scene.addTransformNode(sceneObjectGroupId, tfId, attributes, transform123, t1);

	attributes.clear();
	attributes.push_back(Attribute("name","Box"));
	attributes.push_back(Attribute("color","green"));
	scene.addGroup(tfId, boxGroupId2, attributes);

	/* reference to the same geometry */
	scene.addParent(geometryId, boxGroupId2);

	/* multible obeservations to the same object */
	attributes.clear();
	attributes.push_back(Attribute("name","box_tf2"));
	scene.addTransformNode(sceneObjectGroupId, tfId, attributes, transform123, t2);
	scene.addParent(boxGroupId2, tfId);

	attributes.clear();
	attributes.push_back(Attribute("name","box_tf3"));
	scene.addTransformNode(sceneObjectGroupId, tfId, attributes, transform123, t3);
	scene.addParent(boxGroupId2, tfId);

	/* finally create some robot representation */
	attributes.clear();
	attributes.push_back(Attribute("name","robot_tf"));
	scene.addTransformNode(scene.getRootId(), tfId, attributes, transform123, dummyTime);

	attributes.clear();
	attributes.push_back(Attribute("name","robot"));
	scene.addGroup(tfId, robotGroupId, attributes);

	/* Visualize the structure */
	brics_3d::rsg::DotGraphGenerator dotGraphGenerator;
	string resultString;
	resultString = dotGraphGenerator.getDotGraph();

//	cout << "Dot graph: " << endl << resultString << endl;

	int expectedStringSizeForEmptyGraph = 14;
	CPPUNIT_ASSERT_EQUAL(resultString.compare(""), expectedStringSizeForEmptyGraph);

	scene.executeGraphTraverser(&dotGraphGenerator, scene.getRootId());
	resultString = dotGraphGenerator.getDotGraph();

//	cout << "Dot graph: " << endl << resultString << endl;
	CPPUNIT_ASSERT(resultString.compare("") != expectedStringSizeForEmptyGraph);

}

void SceneGraphNodesTest::testPointIterator() {
	PointCloud3D::PointCloud3DPtr cloud1(new PointCloud3D());
	PointCloud3D::PointCloud3DPtr cloud2(new PointCloud3D());
	PointCloud3D::PointCloud3DPtr cloud3(new PointCloud3D());
	PointCloud3D::PointCloud3DPtr cloudAggregated (new PointCloud3D());
	cloud1->addPoint(Point3D(1,2,3));
	cloud1->addPoint(Point3D(4,5,6));
	cloud1->addPoint(Point3D(7,8,9));

	cloud2->addPoint(Point3D(10,11,12));
	cloud2->addPoint(Point3D(13,14,15));

	cloud3->addPoint(Point3D(20,21,22));
	cloud3->addPoint(Point3D(23,24,25));
	cloud3->addPoint(Point3D(26,27,28));


	cloudAggregated->addPoint(Point3D(1,2,3));
	cloudAggregated->addPoint(Point3D(4,5,6));
	cloudAggregated->addPoint(Point3D(7,8,9));

	cloudAggregated->addPoint(Point3D(110,111,112));
	cloudAggregated->addPoint(Point3D(113,114,115));

	cloudAggregated->addPoint(Point3D(2020,2021,2022));
	cloudAggregated->addPoint(Point3D(2023,2024,2025));
	cloudAggregated->addPoint(Point3D(2026,2027,2028));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr shift100 (new HomogeneousMatrix44 (1,0,0, 0,1,0, 0,0,1, 100, 100, 100));
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr shift2000(new HomogeneousMatrix44 (1,0,0, 0,1,0, 0,0,1, 2000, 2000, 2000));

	int count=0;

	CPPUNIT_ASSERT_EQUAL(3u, cloud1->getSize());
	CPPUNIT_ASSERT_EQUAL(2u, cloud2->getSize());
	CPPUNIT_ASSERT_EQUAL(3u, cloud3->getSize());

	PointCloud3DIterator* it = new PointCloud3DIterator();
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identity(new HomogeneousMatrix44());
	it->insert(cloud1, identity);

	count = 0;
	for (it->begin(); !it->end(); it->next()){
		it->getX();
		it->getY();
		it->getZ();
//		Point3D* tmpPoint = it->getRawData();
//		std::cout << "transformed: ("<< it->getX() << "," << it->getY() << "," << it->getZ() << ")" << std::endl;
//		std::cout << "RAW        : ("<< tmpPoint->getX() << "," << tmpPoint->getY() << "," << tmpPoint->getZ() << ")" << std::endl;


		Point3D resultPoint = (*cloudAggregated->getPointCloud())[count];
		CPPUNIT_ASSERT_DOUBLES_EQUAL(resultPoint.getX(), it->getX(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL(resultPoint.getY(), it->getY(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL(resultPoint.getZ(), it->getZ(), maxTolerance);


		count++;
	}
	CPPUNIT_ASSERT_EQUAL(3, count);


	it->insert(cloud2, shift100);

	count = 0;
//	std::cout << "Iterator data:" << std::endl;
	for (it->begin(); !it->end(); it->next()){
		it->getX();
		it->getY();
		it->getZ();
//		Point3D* tmpPoint = it->getRawData();
//		std::cout << "transformed: ("<< it->getX() << "," << it->getY() << "," << it->getZ() << ")" << std::endl;
//		std::cout << "RAW        : ("<< tmpPoint->getX() << "," << tmpPoint->getY() << "," << tmpPoint->getZ() << ")" << std::endl;

		Point3D resultPoint = (*cloudAggregated->getPointCloud())[count];
//		CPPUNIT_ASSERT_DOUBLES_EQUAL(resultPoint.getX(), it->getX(), maxTolerance); // hard to test as there is no dereministic order in the map iteration
//		CPPUNIT_ASSERT_DOUBLES_EQUAL(resultPoint.getY(), it->getY(), maxTolerance);
//		CPPUNIT_ASSERT_DOUBLES_EQUAL(resultPoint.getZ(), it->getZ(), maxTolerance);

		count++;
	}
	CPPUNIT_ASSERT_EQUAL(5, count);

	it->insert(cloud3, shift2000);

	count = 0;
//	std::cout << "Iterator data:" << std::endl;
	for (it->begin(); !it->end(); it->next()) {
		it->getX();
		it->getY();
		it->getZ();
//		Point3D* tmpPoint = it->getRawData();
//		std::cout << "transformed: ("<< it->getX() << "," << it->getY() << "," << it->getZ() << ")" << std::endl;
//		std::cout << "RAW        : ("<< tmpPoint->getX() << "," << tmpPoint->getY() << "," << tmpPoint->getZ() << ")" << std::endl;

		Point3D resultPoint = (*cloudAggregated->getPointCloud())[count];
//		CPPUNIT_ASSERT_DOUBLES_EQUAL(resultPoint.getX(), it->getX(), maxTolerance);
//		CPPUNIT_ASSERT_DOUBLES_EQUAL(resultPoint.getY(), it->getY(), maxTolerance);
//		CPPUNIT_ASSERT_DOUBLES_EQUAL(resultPoint.getZ(), it->getZ(), maxTolerance);

		count++;
	}
	CPPUNIT_ASSERT_EQUAL(8, count);

	delete it;

	CPPUNIT_ASSERT_EQUAL(3u, cloud1->getSize());
	CPPUNIT_ASSERT_EQUAL(2u, cloud2->getSize());
	CPPUNIT_ASSERT_EQUAL(3u, cloud3->getSize());
}

void SceneGraphNodesTest::testScenePointIterator() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+----------
	 *        |                    |
	 *       pc1                  tf2
	 *                             |
	 *                            pc3
	 *
	 */
	Id rootId = 0;
	Id pc1Id = 1;
	Id tf2Id = 2;
	Id pc3Id = 3;

	Group::GroupPtr root(new Group());
	root->setId(rootId);

	brics_3d::PointCloud3D::PointCloud3DPtr pc1_data(new brics_3d::PointCloud3D());
	brics_3d::PointCloud3D::PointCloud3DPtr pc3_data(new brics_3d::PointCloud3D());
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pc1_container(new rsg::PointCloud<brics_3d::PointCloud3D>());
	PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pc3_container(new rsg::PointCloud<brics_3d::PointCloud3D>());
	pc1_container->data = pc1_data;
	pc3_container->data = pc3_data;

	pc1_container->data->addPoint(Point3D(1,2,3));
	pc1_container->data->addPoint(Point3D(4,5,6));
	pc1_container->data->addPoint(Point3D(7,8,9));

	pc3_container->data->addPoint(Point3D(10,11,12));
	pc3_container->data->addPoint(Point3D(13,14,15));

	GeometricNode::GeometricNodePtr pc1(new GeometricNode());
	pc1->setId(pc1Id);
	GeometricNode::GeometricNodePtr pc3(new GeometricNode());
	pc3->setId(pc3Id);

	pc1->setShape(pc1_container);
	pc3->setShape(pc3_container);

	rsg::Transform::TransformPtr tf2(new rsg::Transform());
	tf2->setId(tf2Id);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr someTransform(new HomogeneousMatrix44(1,0,0,  	//Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             100,100,100)); 						//Translation coefficients
	tf2->insertTransform(someTransform, TimeStamp(0.0));

	/*
	 * finally construct some scenegraph
	 */
	root->addChild(pc1);
	root->addChild(tf2);
	tf2->addChild(pc3);

	PointCloudAccumulator* pcAccumulator = new PointCloudAccumulator(root);
	root->accept(pcAccumulator);

	IPoint3DIterator* it = pcAccumulator->getAccumulatedPointClouds();
//	std::cout << "Scene graph iterator data:" << std::endl;
	int count = 0;
	for (it->begin(); !it->end(); it->next()){
		it->getX();
		it->getY();
		it->getZ();
//		Point3D* tmpPoint = it->getRawData();
//		std::cout << "transformed: ("<< it->getX() << "," << it->getY() << "," << it->getZ() << ")" << std::endl;
//		std::cout << "RAW        : ("<< tmpPoint->getX() << "," << tmpPoint->getY() << "," << tmpPoint->getZ() << ")" << std::endl;
		count++;
	}
	CPPUNIT_ASSERT_EQUAL(5, count);

	delete pcAccumulator;
}

void SceneGraphNodesTest::testSubGraphChecker() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *             root
	 *              |
	 *        ------+-------+--------------
	 *        |             |             |
	 *      group1        group2          tf7
	 *        |             |             |
	 *    ----+----  -------+-------      |
	 *    |       |  |      |      |      |
	 *   node3    node4  node5   group6   geode8
	 */

	const Id rootId = 0;
	const Id group1Id = 1;
	const Id group2Id = 2;
	const Id node3Id = 3;
	const Id node4Id = 4;
	const Id node5Id = 5;
	const Id group6Id = 6;
	const Id tf7Id = 7;
	const Id geode8Id = 8;

	Group::GroupPtr root(new Group());
	root->setId(rootId);
	Group::GroupPtr group1(new Group());
	group1->setId(group1Id);
	Group::GroupPtr group2(new Group());
	group2->setId(group2Id);

	Node::NodePtr node3(new Node());
	node3->setId(node3Id);
	Node::NodePtr node4(new Node());
	node4->setId(node4Id);
	Node::NodePtr node5(new Node());
	node5->setId(node5Id);

	Group::GroupPtr group6(new Group());
	group6->setId(group6Id);

	rsg::Transform::TransformPtr tf7(new rsg::Transform());
	tf7->setId(tf7Id);

	rsg::GeometricNode::GeometricNodePtr geode8(new rsg::GeometricNode());
	geode8->setId(geode8Id);

	CPPUNIT_ASSERT_EQUAL(rootId, root->getId()); // preconditions:
	CPPUNIT_ASSERT_EQUAL(group1Id, group1->getId());
	CPPUNIT_ASSERT_EQUAL(group2Id, group2->getId());
	CPPUNIT_ASSERT_EQUAL(node3Id, node3->getId());
	CPPUNIT_ASSERT_EQUAL(node4Id, node4->getId());
	CPPUNIT_ASSERT_EQUAL(node5Id, node5->getId());
	CPPUNIT_ASSERT_EQUAL(group6Id, group6->getId());
	CPPUNIT_ASSERT_EQUAL(tf7Id, tf7->getId());
	CPPUNIT_ASSERT_EQUAL(geode8Id, geode8->getId());

	/* set up graph */
	root->addChild(group1);
	root->addChild(group2);
	root->addChild(tf7);

	group1->addChild(node3);
	group1->addChild(node4);

	group2->addChild(node4);
	group2->addChild(node5);
	group2->addChild(group6);

	tf7->addChild(geode8);

	/* root chould be in all ...*/
	SubGraphChecker checker(rootId);
	CPPUNIT_ASSERT(!checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(0u, checker.getPathCount());
	root->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount()); //itself => 1

	checker.reset(rootId);
	group1->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	checker.reset(rootId);
	group2->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	checker.reset(rootId);
	node3->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	checker.reset(rootId);
	node4->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(2u, checker.getPathCount());

	checker.reset(rootId);
	node5->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	checker.reset(rootId);
	group6->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	checker.reset(rootId);
	tf7->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	checker.reset(rootId);
	geode8->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	/* who can see group 1...*/
	checker.reset(group1Id);
	root->accept(&checker);
	CPPUNIT_ASSERT(!checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(0u, checker.getPathCount());

	checker.reset(group1Id);
	group1->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	checker.reset(group1Id);
	group2->accept(&checker);
	CPPUNIT_ASSERT(!checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(0u, checker.getPathCount());

	checker.reset(group1Id);
	node3->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	checker.reset(group1Id);
	node4->accept(&checker);
	CPPUNIT_ASSERT(checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(1u, checker.getPathCount());

	checker.reset(group1Id);
	node5->accept(&checker);
	CPPUNIT_ASSERT(!checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(0u, checker.getPathCount());

	checker.reset(group1Id);
	group6->accept(&checker);
	CPPUNIT_ASSERT(!checker.nodeIsInSubGraph());
	CPPUNIT_ASSERT_EQUAL(0u, checker.getPathCount());
}

void SceneGraphNodesTest::testForcedIds() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *        -----------+------------------
	 *        |          |         |        |
	 *       tf1        group2    node3     geode4
	 *
	 */


	/* will be assigned later */
//	Id rootId = 0;
	Id tf1Id = 0;
	Id tf1IdForced = 0;
	Id group2Id = 0;
	Id group2IdForced = 0;
	Id node3Id = 0;
	Id node3IdForced = 0;
	Id geode4Id = 0;
	Id geode4IdForced = 0;
	const Id id2 = 2;
	const Id id3 = 3;
	const Id id4 = 4;
	const Id id6 = 6;
	const Id id7 = 7;
	const Id id8 = 8;
	const Id id10 = 10;
	const Id id11 = 11;
	const Id id12 = 12;
	const Id id14 = 14;
	const Id id15 = 15;
	const Id id16 = 16;
	const Id invalidId = 1000001;

	SimpleIdGenerator* idGenerator = new SimpleIdGenerator; // Will be deleted by scene automatically.
	SceneGraphFacade scene(idGenerator); //assumes SimpleIdGenerator
	vector<Attribute> attributes;
	TimeStamp dummyTime;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr dummyTransform(new HomogeneousMatrix44());
	Box::BoxPtr dummyBox(new Box());

	tf1IdForced = 0;
	CPPUNIT_ASSERT(!scene.addTransformNode(scene.getRootId(), tf1IdForced, attributes, dummyTransform, dummyTime, true));
	tf1IdForced = 1;
	CPPUNIT_ASSERT(!scene.addTransformNode(scene.getRootId(), tf1IdForced, attributes, dummyTransform, dummyTime, true)); //this would override root
	tf1IdForced = 2;
	CPPUNIT_ASSERT(scene.addTransformNode(scene.getRootId(), tf1IdForced, attributes, dummyTransform, dummyTime, true));
	CPPUNIT_ASSERT_EQUAL(id2, tf1IdForced);
	CPPUNIT_ASSERT(scene.addTransformNode(scene.getRootId(), tf1Id, attributes, dummyTransform, dummyTime, false));
	CPPUNIT_ASSERT_EQUAL(id3, tf1Id);
	tf1IdForced = 4; // ok, but invalid parent:
	CPPUNIT_ASSERT(!scene.addTransformNode(invalidId, tf1IdForced, attributes, dummyTransform, dummyTime, true)); //does not work but wastes an ID
	CPPUNIT_ASSERT_EQUAL(id4, tf1IdForced);
	CPPUNIT_ASSERT(!scene.addTransformNode(invalidId, tf1Id, attributes, dummyTransform, dummyTime, false)); //does not work but wastes an ID
	CPPUNIT_ASSERT_EQUAL(id3, tf1Id);

	group2IdForced = 0;
	CPPUNIT_ASSERT(!scene.addGroup(scene.getRootId(), group2IdForced, attributes, true)); //this would override root
	group2IdForced = 1;
	CPPUNIT_ASSERT(!scene.addGroup(scene.getRootId(), group2IdForced, attributes, true));
	group2IdForced = 2;
	CPPUNIT_ASSERT(!scene.addGroup(scene.getRootId(), group2IdForced, attributes, true));
	group2IdForced = 3;
	CPPUNIT_ASSERT(!scene.addGroup(scene.getRootId(), group2IdForced, attributes, true));
	group2IdForced = 4;
	CPPUNIT_ASSERT(!scene.addGroup(scene.getRootId(), group2IdForced, attributes, true));
	group2IdForced = 5;
	CPPUNIT_ASSERT(!scene.addGroup(scene.getRootId(), group2IdForced, attributes, true));
	group2IdForced = 6;
	CPPUNIT_ASSERT(scene.addGroup(scene.getRootId(), group2IdForced, attributes, true));
	CPPUNIT_ASSERT_EQUAL(id6, group2IdForced);
	CPPUNIT_ASSERT(scene.addGroup(scene.getRootId(), group2Id, attributes, false)); // automatic generated
	CPPUNIT_ASSERT_EQUAL(id7, group2Id);
	group2IdForced = 8; // ok, but invalid parent:
	CPPUNIT_ASSERT(!scene.addGroup(invalidId, group2IdForced, attributes, true)); //does not work but wastes an ID
	CPPUNIT_ASSERT_EQUAL(id8, group2IdForced);
	CPPUNIT_ASSERT(!scene.addGroup(invalidId, group2Id, attributes, false)); //does not work but wastes an ID
	CPPUNIT_ASSERT_EQUAL(id7, group2Id);

	node3IdForced = 1;
	CPPUNIT_ASSERT(!scene.addNode(scene.getRootId(), node3IdForced, attributes, true));
	node3IdForced = 10;
	CPPUNIT_ASSERT(scene.addNode(scene.getRootId(), node3IdForced, attributes, true));
	CPPUNIT_ASSERT_EQUAL(id10, node3IdForced);
	CPPUNIT_ASSERT(scene.addNode(scene.getRootId(), node3Id, attributes, false));
	CPPUNIT_ASSERT_EQUAL(id11, node3Id);
	node3IdForced = 12;  // ok, but invalid parent:
	CPPUNIT_ASSERT(!scene.addNode(invalidId, node3IdForced, attributes, true));
	CPPUNIT_ASSERT_EQUAL(id12, node3IdForced);
	CPPUNIT_ASSERT(!scene.addNode(invalidId, node3Id, attributes, false));
	CPPUNIT_ASSERT_EQUAL(id11, node3Id);

	geode4IdForced = 1;
	CPPUNIT_ASSERT(!scene.addGeometricNode(scene.getRootId(), geode4IdForced, attributes, dummyBox, dummyTime, true));
	geode4IdForced = 14;
	CPPUNIT_ASSERT(scene.addGeometricNode(scene.getRootId(), geode4IdForced, attributes, dummyBox, dummyTime, true));
	CPPUNIT_ASSERT_EQUAL(id14, geode4IdForced);
	CPPUNIT_ASSERT(scene.addGeometricNode(scene.getRootId(), geode4Id, attributes, dummyBox, dummyTime, false));
	CPPUNIT_ASSERT_EQUAL(id15, geode4Id);
	geode4IdForced = 16; // ok, but invalid parent:
	CPPUNIT_ASSERT(!scene.addGeometricNode(invalidId, geode4IdForced, attributes, dummyBox, dummyTime, true));
	CPPUNIT_ASSERT_EQUAL(id16, geode4IdForced);
	CPPUNIT_ASSERT(!scene.addGeometricNode(invalidId, geode4IdForced, attributes, dummyBox, dummyTime, false));
	CPPUNIT_ASSERT_EQUAL(id15, geode4Id);

}

void SceneGraphNodesTest::testSceneGraphToUpdates() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *             root
	 *              |
	 *        ------+-------
	 *        |             |
	 *      group1        group2
	 *        |             |
	 *    ----+----  -------+-------
	 *    |       |  |      |      |
	 *   node3    group4    tf5   geom6
	 *              |
	 *          ----+---
	 *          |      |
	 *       group7   group8
	 *          |      |
	 *          ----+---
	 *           group9
	 */

	Id group1Id = 0;
	Id group2Id = 0;
	Id node3Id = 0;
	Id group4Id = 0;
	Id tf5Id = 0;
	Id geom6Id = 0;
	Id group7Id = 0;
	Id group8Id = 0;
	Id group9Id = 0;

	brics_3d::rsg::SceneGraphFacade scene(new UuidGenerator(1u));			// The 3D world model handle
	vector<Attribute> attributes;	// with this one we can attach attibutes / tags to the nodes

	/* Some (dummy) data to be use within the scenegraph */
	brics_3d::rsg::TimeStamp dummyTime(0);
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						// Translation coefficients
	brics_3d::rsg::Box::BoxPtr dummyBox(new brics_3d::rsg::Box());

	/*
	 * Set up the scenegraph
	 */
	MyObserver elementCounterCretation;
	MyObserver elementCounterTraversal;

	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.deleteNodeCounter);

	scene.attachUpdateObserver(&elementCounterCretation);

	attributes.clear();
	attributes.push_back(Attribute("name","group1"));
	scene.addGroup(scene.getRootId(), group1Id, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","group2"));
	scene.addGroup(scene.getRootId(), group2Id, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","node3"));
	scene.addNode(group1Id, node3Id, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","group4"));
	scene.addGroup(group1Id, group4Id, attributes);

	scene.addParent(group4Id, group2Id);

	attributes.clear();
	attributes.push_back(Attribute("name","tf5"));
	scene.addTransformNode(group2Id, tf5Id, attributes, transform123, dummyTime);

	attributes.clear();
	attributes.push_back(Attribute("name","geom6"));
	scene.addGeometricNode(group2Id, geom6Id, attributes, dummyBox, dummyTime);

	attributes.clear();
	attributes.push_back(Attribute("name","group7"));
	scene.addGroup(group4Id, group7Id, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","group8"));
	scene.addGroup(group4Id, group8Id, attributes);

	attributes.clear();
	attributes.push_back(Attribute("name","group9"));
	scene.addGroup(group7Id, group9Id, attributes);

	scene.addParent(group9Id, group8Id);

	CPPUNIT_ASSERT_EQUAL(1, elementCounterCretation.addNodeCounter); //postcondition
	CPPUNIT_ASSERT_EQUAL(6, elementCounterCretation.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, elementCounterCretation.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, elementCounterCretation.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, elementCounterCretation.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.deleteNodeCounter);

	SceneGraphToUpdatesTraverser graphToUpdates(&elementCounterTraversal);
	scene.executeGraphTraverser(&graphToUpdates, scene.getRootId());

	CPPUNIT_ASSERT_EQUAL(1, elementCounterTraversal.addNodeCounter); //postcondition
	CPPUNIT_ASSERT_EQUAL(6, elementCounterTraversal.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, elementCounterTraversal.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, elementCounterTraversal.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterTraversal.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, elementCounterTraversal.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, elementCounterCretation.deleteNodeCounter);

//	cout << "Duplicating..." << endl;
	brics_3d::rsg::SceneGraphFacade sceneDuplication(new UuidGenerator(1u)); // we also dublicate the root Id (=1u), for better testability
	SceneGraphToUpdatesTraverser graphDuplicator(&sceneDuplication);
	scene.executeGraphTraverser(&graphDuplicator, scene.getRootId());

	DotGraphGenerator dotTraverser;
	dotTraverser.reset();
	scene.executeGraphTraverser(&dotTraverser, scene.getRootId());
//	cout << "Graph from scene 1:" << endl;
//	cout << dotTraverser.getDotGraph() << endl;
	string graph1 = dotTraverser.getDotGraph();

	dotTraverser.reset();
	sceneDuplication.executeGraphTraverser(&dotTraverser, sceneDuplication.getRootId());
//	cout << "Graph from duplicated scene 2:" << endl;
//	cout << dotTraverser.getDotGraph() << endl;
	string graph2 = dotTraverser.getDotGraph();

	CPPUNIT_ASSERT(graph1.compare(graph2) == 0);


}

void SceneGraphNodesTest::testRemoveParents() {
	/* Graph structure: (remember: nodes can only serve as are leaves)
	 *                 root
	 *                   |
	 *              +----------+
	 *              |          |
	 *            group1    group2
	 *              |          |
	 *              +----  ----+
	 *                  |  |
	 *                 node3
	 */

	Id group1Id = 0;
	Id group2Id = 0;
	Id node3Id = 0;

	brics_3d::rsg::SceneGraphFacade scene;
	std::vector<Attribute> attributes;
	std::vector<Id> resultIds;

	attributes.clear(); //empty dummy
	CPPUNIT_ASSERT(scene.addGroup(scene.getRootId(), group1Id, attributes));
	CPPUNIT_ASSERT(scene.addGroup(scene.getRootId(), group2Id, attributes));
	CPPUNIT_ASSERT(scene.addNode(group1Id, node3Id, attributes));
	CPPUNIT_ASSERT(scene.addParent(node3Id, group2Id));

	CPPUNIT_ASSERT(scene.getNodeAttributes(group1Id, attributes)); //result will be empty but the should be no error as the nodes exist
	CPPUNIT_ASSERT(scene.getNodeAttributes(group2Id, attributes));
	CPPUNIT_ASSERT(scene.getNodeAttributes(node3Id, attributes));

	/* check all relations */
	resultIds.clear(); //check parents
	CPPUNIT_ASSERT(scene.getNodeParents(scene.getRootId(), resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(group1Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(group2Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(node3Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultIds.size()));

	resultIds.clear();//check childs
	CPPUNIT_ASSERT(scene.getGroupChildren(scene.getRootId(), resultIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group1Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group2Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));

	/* remove one relation
	 *                 root
	 *                   |
	 *              +----------+
	 *              |          |
	 *            group1    group2
	 *                         |
	 *                 X   ----+
	 *                     |
	 *                 node3
	 */

	CPPUNIT_ASSERT(scene.removeParent(node3Id, group1Id));

	/* check again all relations */
	resultIds.clear(); //check parents
	CPPUNIT_ASSERT(scene.getNodeParents(scene.getRootId(), resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(group1Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(group2Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(node3Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));

	resultIds.clear();//check childs
	CPPUNIT_ASSERT(scene.getGroupChildren(scene.getRootId(), resultIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group1Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group2Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));

	/* those oprations should fail */
	CPPUNIT_ASSERT(!scene.removeParent(node3Id, group1Id));	 // again the same
	CPPUNIT_ASSERT(!scene.removeParent(group1Id, group1Id)); // self
	CPPUNIT_ASSERT(!scene.removeParent(node3Id, scene.getRootId())); // invalid relation
	CPPUNIT_ASSERT(!scene.removeParent(group2Id, node3Id)); // invalid ralation as it is inversed
	CPPUNIT_ASSERT(!scene.removeParent(scene.getRootId(), node3Id)); // invalid as root is first ID
	const unsigned int invalidNodeId = 10003522;
	CPPUNIT_ASSERT(!scene.removeParent(invalidNodeId, group1Id)); // invalid ID1
	CPPUNIT_ASSERT(!scene.removeParent(group1Id, invalidNodeId)); // invalid ID2


	/* check again all relations */
	resultIds.clear(); //check parents
	CPPUNIT_ASSERT(scene.getNodeParents(scene.getRootId(), resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(group1Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(group2Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(node3Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));

	resultIds.clear();//check childs
	CPPUNIT_ASSERT(scene.getGroupChildren(scene.getRootId(), resultIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group1Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group2Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));

	/* remove another relation
	 *                 root
	 *                   |
	 *              +-----    X
	 *              |
	 *            group1    group2
	 *                         |
	 *                     ----+
	 *                     |
	 *                   node3
	 */

	CPPUNIT_ASSERT(scene.removeParent(group2Id, scene.getRootId()));

	/* check again all relations */
	resultIds.clear(); //check parents
	CPPUNIT_ASSERT(scene.getNodeParents(scene.getRootId(), resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getNodeParents(group1Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(!scene.getNodeParents(group2Id, resultIds));
	resultIds.clear();
	CPPUNIT_ASSERT(!scene.getNodeParents(node3Id, resultIds));


	resultIds.clear();//check childs
	CPPUNIT_ASSERT(scene.getGroupChildren(scene.getRootId(), resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(scene.getGroupChildren(group1Id, resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size()));
	resultIds.clear();
	CPPUNIT_ASSERT(!scene.getGroupChildren(group2Id, resultIds));

}

void SceneGraphNodesTest::testNodeStorage() {
	brics_3d::rsg::SceneGraphFacade scene;

    std::vector<brics_3d::rsg::Attribute> attributes;
    attributes.clear();
	brics_3d::rsg::Id boxId = 234;

	{
	    brics_3d::rsg::Box::BoxPtr someBox(new brics_3d::rsg::Box(2, 3, 4));
	    attributes.clear();
	    attributes.push_back(brics_3d::rsg::Attribute("name","some_box"));
	    scene.addGeometricNode(scene.getRootId(), boxId, attributes, someBox, TimeStamp(0.0), true);
	} // let box ptr get out of scope....

	{
		brics_3d::rsg::Id resultId;
		vector<Id> results;
		scene.getNodes(attributes, results);
		CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(results.size()));
		resultId = results[0];
		CPPUNIT_ASSERT(resultId == boxId);
	}

	{
	    brics_3d::rsg::Shape::ShapePtr inputShape;
	    brics_3d::rsg::TimeStamp inputTime;
	    scene.getGeometry(boxId, inputShape, inputTime);

	    brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr inputPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	    inputPointCloudContainer = boost::dynamic_pointer_cast<brics_3d::rsg::PointCloud<brics_3d::PointCloud3D> >(inputShape);


	    brics_3d::rsg::Box::BoxPtr someOtherBox(new brics_3d::rsg::Box());
	    someOtherBox = boost::dynamic_pointer_cast<brics_3d::rsg::Box>(inputShape);

	    CPPUNIT_ASSERT(someOtherBox->getSizeX() == 2);

	}

}

}  // namespace unitTests

/* EOF */
