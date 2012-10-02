/**
 * @file 
 * WorldModelTest.h
 *
 * @date: Oct 19, 2011
 * @author: sblume
 */

#ifndef WORLDMODELTEST_H_
#define WORLDMODELTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "core/HomogeneousMatrix44.h"
#include "util/Timer.h"
#include "worldModel/WorldModel.h"
#include "worldModel/sceneGraph/Attribute.h"
#include "worldModel/sceneGraph/Box.h"
#include "worldModel/sceneGraph/Cylinder.h"


namespace unitTests {

using namespace brics_3d;
using namespace Eigen;
using namespace std;
using namespace brics_3d::rsg;
using std::cout;
using std::endl;



class WorldModelTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( WorldModelTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testSimpleHanoiUseCase );
	CPPUNIT_TEST( testTowerOfHanoi );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testConstructor();
	void testSimpleHanoiUseCase();
	void testTowerOfHanoi();

private:
	  /// Maximum deviation for equality check of double variables
	  static const double maxTolerance = 0.00001;
};

}
#endif /* WORLDMODELTEST_H_ */

/* EOF */
