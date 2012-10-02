/**
 * @file 
 * BoxROIExtractorTest.h
 *
 * @date: Apr 1, 2012
 * @author: sblume
 */

#ifndef BOXROIEXTRACTORTEST_H_
#define BOXROIEXTRACTORTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/algorithm/filtering/BoxROIExtractor.h"

using namespace std;
using namespace Eigen;
using namespace brics_3d;

namespace unitTests {
class BoxROIExtractorTest : public CPPUNIT_NS::TestFixture {
	CPPUNIT_TEST_SUITE( BoxROIExtractorTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testBoxExtraction );
	CPPUNIT_TEST_SUITE_END();

public:

	void setUp();
	void tearDown();

	void testConstructor();
	void testBoxExtraction();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2

private:
	IFiltering* boxFilter;

	/// Maximum deviation for equality check of double variables
	static const double maxTolerance = 0.00001;

	PointCloud3D* testCloud;

};
}

#endif /* BOXROIEXTRACTORTEST_H_ */

/* EOF */
