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
#ifndef DENSITYEXTRACTIONTEST_H_
#define DENSITYEXTRACTIONTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/algorithm/featureExtraction/DensityExtractor.h"

using namespace brics_3d;

namespace unitTests {

class DensityExtractionTest : public CPPUNIT_NS::TestFixture {


	CPPUNIT_TEST_SUITE( DensityExtractionTest );
	CPPUNIT_TEST( testSimpleDensityExtraction );
	CPPUNIT_TEST_SUITE_END();
public:

	void setUp();
	void tearDown();

	void testSimpleDensityExtraction();

	/// Maximum deviation for equality check of double variables
	static const double maxTolerance = 0.00001;

	PointCloud3D* testCloud;
	PointCloud3D* testCloudUnitCube;

};

} /* namespace unitTests */

#endif /* DENSITYEXTRACTIONTEST_H_ */

/* EOF */
