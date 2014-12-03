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

#ifndef DistributedWorldModelTest_H_
#define DistributedWorldModelTest_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/util/Timer.h"
#include "brics_3d/worldModel/WorldModel.h"
#include "brics_3d/worldModel/sceneGraph/Attribute.h"
#include "brics_3d/worldModel/sceneGraph/Box.h"
#include "brics_3d/worldModel/sceneGraph/Cylinder.h"
#include "brics_3d/worldModel/sceneGraph/UuidGenerator.h"
#include "brics_3d/worldModel/sceneGraph/UpdatesToSceneGraphListener.h"

namespace unitTests {

using namespace brics_3d;
using namespace Eigen;
using namespace std;
using namespace brics_3d::rsg;
using std::cout;
using std::endl;



class DistributedWorldModelTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( DistributedWorldModelTest );
	CPPUNIT_TEST( testRootIds );
	CPPUNIT_TEST( testRemoteRootNodeCreationAndDeletion );
	CPPUNIT_TEST( testRemoteRootNodes );
	CPPUNIT_TEST( testRemoteRootNodeQueries );
	CPPUNIT_TEST( testDirectUpdateObserver );
	CPPUNIT_TEST( testSelfAnnouncemnt );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testRootIds();
	void testRemoteRootNodeCreationAndDeletion();
	void testRemoteRootNodes();
	void testRemoteRootNodeQueries();
	void testDirectUpdateObserver();
	void testSelfAnnouncemnt();


private:
	  /// Maximum deviation for equality check of double variables
	  static const double maxTolerance = 0.00001;
};

}
#endif /* DistributedWorldModelTest_H_ */

/* EOF */
