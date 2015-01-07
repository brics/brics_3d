/**
 * @file 
 * FunctionBlockTest.cpp
 *
 * @date: Dec 18, 2013
 * @author: sblume
 */

#include "FunctionBlockTest.h"
#include "brics_3d/core/Logger.h"
#include "brics_3d/worldModel/sceneGraph/DotGraphGenerator.h"

namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( FunctionBlockTest );

void FunctionBlockTest::setUp() {

	wm = new brics_3d::WorldModel();

//	functionBlockFile = "cppdemo";

	/*
	 * NOTE: This one is located in the brics_3d_function_blocks repository.
	 * If it is not installed the tests will fail.
	 */
	functionBlockFile = "testblock";

	if(getenv("FBX_MODULES") != 0) {
		string functionBlocksModulesPath(getenv("FBX_MODULES"));
		blockRepositoryPath = functionBlocksModulesPath + "/lib/";
	} else {
		blockRepositoryPath = "/opt/src/sandbox/brics_3d_function_blocks/lib/";
	}

	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
}

void FunctionBlockTest::tearDown() {
	delete wm;
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::WARNING);
}

void FunctionBlockTest::testFunctionBlockLoader() {
	CPPUNIT_ASSERT(!wm->loadFunctionBlock("wrongFileName"));
	CPPUNIT_ASSERT(wm->loadFunctionBlock(functionBlockFile, blockRepositoryPath));
}

void FunctionBlockTest::testFunctionBlockExecution() {
	CPPUNIT_ASSERT(wm->loadFunctionBlock(functionBlockFile, blockRepositoryPath));

	vector<brics_3d::rsg::Id> input;
	input.push_back(wm->getRootNodeId()); // input hook
	input.push_back(wm->getRootNodeId()); // output hook
	vector<brics_3d::rsg::Id> output;
	CPPUNIT_ASSERT(!wm->executeFunctionBlock("wrongFileName", input, output));
	CPPUNIT_ASSERT(wm->executeFunctionBlock(functionBlockFile, input, output));
	CPPUNIT_ASSERT(wm->executeFunctionBlock(functionBlockFile, input, output));
	CPPUNIT_ASSERT(wm->executeFunctionBlock(functionBlockFile, input, output));
}

void FunctionBlockTest::testExternalFunctionBlockExecution() {
	string blockName = "roifilter";//"testblock";
	string blockPath = blockRepositoryPath;

	CPPUNIT_ASSERT(wm->loadFunctionBlock(blockName, blockPath));


	brics_3d::rsg::Id pc1Id = 1025;
	brics_3d::PointCloud3D::PointCloud3DPtr pointCloud1(new brics_3d::PointCloud3D());
	pointCloud1->addPoint(brics_3d::Point3D(1,2,3));
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pc1Container(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	pc1Container->data = pointCloud1;
	std::vector<brics_3d::rsg::Attribute> tmpAttributes;
	tmpAttributes.clear();
	tmpAttributes.push_back(brics_3d::rsg::Attribute("name","point_cloud1"));
	wm->scene.addGeometricNode(wm->getRootNodeId(), pc1Id, tmpAttributes, pc1Container, brics_3d::rsg::TimeStamp(0.0)/*, true*/);

	/* Just print what the world model has to offer. */
	brics_3d::rsg::DotGraphGenerator* wmPrinter = new brics_3d::rsg::DotGraphGenerator();
	wmPrinter->reset();
	brics_3d::rsg::VisualizationConfiguration config;
	config.abbreviateIds = false;
	wmPrinter->setConfig(config);
	wm->scene.executeGraphTraverser(wmPrinter, wm->getRootNodeId());
	LOG(DEBUG) << "testExternalFunctionBlockExecution: Current state of the world model: " << std::endl << wmPrinter->getDotGraph();

	vector<brics_3d::rsg::Id> input;
	brics_3d::rsg::Id inputHook = pc1Id;//1042; //wm->getRootNodeId();
	brics_3d::rsg::Id outputHook = wm->getRootNodeId();
	input.push_back(outputHook); // output input hook
	input.push_back(inputHook); // input hook
	vector<brics_3d::rsg::Id> output;
	CPPUNIT_ASSERT(wm->executeFunctionBlock(blockName, input, output));

	wmPrinter->reset();
	wm->scene.executeGraphTraverser(wmPrinter, wm->getRootNodeId());
	LOG(DEBUG) << "testExternalFunctionBlockExecution: Current state of the world model: " << std::endl << wmPrinter->getDotGraph();


	/* modify mw */
	brics_3d::rsg::Id groupId = 0;
	std::vector<brics_3d::rsg::Attribute> attributes;
	attributes.push_back(brics_3d::rsg::Attribute("name","test_group1"));
	wm->scene.addGroup(wm->getRootNodeId(), groupId, attributes);

	wmPrinter->reset();
	wm->scene.executeGraphTraverser(wmPrinter, wm->getRootNodeId());
	LOG(DEBUG) << "testExternalFunctionBlockExecution: Current state of the world model: " << std::endl << wmPrinter->getDotGraph();


	output.clear();
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(output.size()));
	CPPUNIT_ASSERT(wm->executeFunctionBlock(blockName, input, output));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(output.size()));
	CPPUNIT_ASSERT(output[0] == outputHook);
	LOG(DEBUG) << "testExternalFunctionBlockExecution: result ID: " << output[1];

	wmPrinter->reset();
	wm->scene.executeGraphTraverser(wmPrinter, wm->getRootNodeId());
	LOG(DEBUG) << "testExternalFunctionBlockExecution: Current state of the world model: " << std::endl << wmPrinter->getDotGraph();


//	CPPUNIT_ASSERT(wm->loadFunctionBlock(blockName, blockPath));  //not yet working
	CPPUNIT_ASSERT(wm->executeFunctionBlock(blockName, input, output));

	wmPrinter->reset();
	wm->scene.executeGraphTraverser(wmPrinter, wm->getRootNodeId());
	LOG(DEBUG) << "testExternalFunctionBlockExecution: Current state of the world model: " << std::endl << wmPrinter->getDotGraph();

}

}  // namespace unitTests

/* EOF */
