/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#ifndef GENERICINTERFACESTEST_H_
#define GENERICINTERFACESTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/TriangleMeshExplicit.h"
#include "brics_3d/core/TriangleMeshImplicit.h"
#include "brics_3d/algorithm/filtering/Octree.h"
#include "brics_3d/algorithm/registration/IterativeClosestPoint.h"
#include "brics_3d/algorithm/registration/PointCorrespondenceKDTree.h"
#include "brics_3d/algorithm/registration/RigidTransformationEstimationSVD.h"
#include "brics_3d/algorithm/meshGeneration/DelaunayTriangulationOSG.h"

using namespace std;
using namespace Eigen;
using namespace brics_3d;

namespace unitTests {

class GenericInterfacesTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( GenericInterfacesTest );
	CPPUNIT_TEST( testFiltering );
	CPPUNIT_TEST( testRegistration );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testFiltering();
	void testRegistration();
	void testMeshGeneration();

private:

	/* input data */
	PointCloud3D* inputPointCloud1;
	PointCloud3D* inputPointCloud2;

	/* output data */
	PointCloud3D* outputPointCloud;
	HomogeneousMatrix44* resultTransform;
	ITriangleMesh* meshExplicit;
	ITriangleMesh* meshImplicit;

	static const double maxTolerance = 0.00001;
};

}

#endif /* GENERICINTERFACESTEST_H_ */

/* EOF */
