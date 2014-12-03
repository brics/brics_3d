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


#ifndef HDF5Test_H_
#define HDF5Test_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/worldModel/WorldModel.h"

namespace unitTests {



class HDF5Test : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( HDF5Test );
#ifdef BRICS_HDF5_ENABLE
	CPPUNIT_TEST( testLoopBack );
	CPPUNIT_TEST( testUpdateObersver );
#endif /* BRICS_HDF5_ENABLE */
	CPPUNIT_TEST_SUITE_END();

public:

	void setUp();
	void tearDown();

	void testLoopBack();
	void testUpdateObersver();
	void threadFunction(brics_3d::WorldModel* wm);

private:

//	brics_3d::WorldModel* wm;
	volatile bool doRun;

};

}  // namespace unitTests
#endif /* HDF5Test_H_ */

/* EOF */
