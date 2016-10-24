/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2014, KU Leuven
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


#ifndef GraphConstraintTest_H_
#define GraphConstraintTest_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/worldModel/WorldModel.h"
#include "brics_3d/worldModel/sceneGraph/GraphConstraintUpdateFilter.h"

namespace unitTests {



class GraphConstraintTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( GraphConstraintTest );
	CPPUNIT_TEST( testParser );
	CPPUNIT_TEST( testNoConstraints );
	CPPUNIT_TEST( testSimpleConstraints );
	CPPUNIT_TEST( testInvalidConstraints );
	CPPUNIT_TEST( testSemanticContextConstraints );
	CPPUNIT_TEST( testDistanceConstraints );
	CPPUNIT_TEST( testContainmentConstraints );
	CPPUNIT_TEST( testLODConstraints );
	CPPUNIT_TEST( testFrequencyConstraints );
	CPPUNIT_TEST( testMultiConstraints );
	CPPUNIT_TEST( testConstraintsAsAttributes );
	CPPUNIT_TEST( testReceiverSimpleConstraints );
	CPPUNIT_TEST( testReceiverSemanticContextConstraints );
	CPPUNIT_TEST_SUITE_END();

public:

	void setUp();
	void tearDown();

	void testParser();
	void testNoConstraints();
	void testSimpleConstraints();
	void testInvalidConstraints();
	void testSemanticContextConstraints();
	void testDistanceConstraints();
	void testContainmentConstraints();
	void testLODConstraints();
	void testFrequencyConstraints();
	void testMultiConstraints();
	void testConstraintsAsAttributes();
	void testReceiverSimpleConstraints();
	void testReceiverSemanticContextConstraints();

private:
	/// Maximum deviation for equality check of double variables
	static const double maxTolerance = 0.00001;

	bool runAddAllSceneGraphPrimitives(brics_3d::WorldModel* wm);

	unsigned int i;

};

}  // namespace unitTests
#endif /* GraphConstraintTest_H_ */

/* EOF */
