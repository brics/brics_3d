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
#ifndef IDTEST_H_
#define IDTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/Logger.h"
#include "brics_3d/worldModel/sceneGraph/Id.h"
#include "brics_3d/worldModel/sceneGraph/Uuid.h"
#include "brics_3d/worldModel/sceneGraph/UuidGenerator.h"

using namespace std;
using namespace brics_3d;
using namespace brics_3d::rsg;
using brics_3d::Logger;

namespace unitTests {

class IdTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( IdTest );
	CPPUNIT_TEST( testIds );
	CPPUNIT_TEST( testUuids );
	CPPUNIT_TEST( testIdGenerator );
	CPPUNIT_TEST( testUuidGenerator );
	CPPUNIT_TEST_SUITE_END();

public:

	void setUp();
	void tearDown();

	void testIds();
	void testUuids();
	void testIdGenerator();
	void testUuidGenerator();

};

} /* namespace unitTests */

#endif /* IDTEST_H_ */

/* EOF */
