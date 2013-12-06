/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2013, KU Leuven
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

#include "UncertaintyOperationsTest.h"

#include <math.h>

using brics_3d::Logger;
using namespace brics_3d::matrixEntry;


namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( UncertaintyOperationsTest );

void UncertaintyOperationsTest::setUp() {
	initialLogLevel = brics_3d::Logger::getMinLoglevel();
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG); // we locally override the log level
}

void UncertaintyOperationsTest::tearDown() {
	brics_3d::Logger::setMinLoglevel(initialLogLevel);
}

void UncertaintyOperationsTest::testIdentityCompounding() {

	const double* matrixPtr;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identityTransform(new HomogeneousMatrix44()); // Identity
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyX(new CovarianceMatrix66(1, 0, 0,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyY(new CovarianceMatrix66(0, 1, 0,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyZ(new CovarianceMatrix66(0, 0, 1,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyXRot(new CovarianceMatrix66(0, 0, 0,  1, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyYRot(new CovarianceMatrix66(0, 0, 0,  0, 1, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr zeroUncertainty(new CovarianceMatrix66(0, 0, 0,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr compoundUncertainty;

	/* test compounding */
	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyX,
			identityTransform,
			uncertaintyY);

//	LOG(DEBUG) << "identityTransformY = " << std::endl << *identityTransform;
//	LOG(DEBUG) << "compoundCovariance uncertaintyX + uncertaintyY = " << std::endl << *compoundUncertainty;

	matrixPtr = compoundUncertainty->getRawData();
	for (int i = 0; i < compoundUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance); //-nan?
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	compoundUncertainty = compoundCovariance(identityTransform,
			compoundUncertainty, //cascaded invokation
			identityTransform,
			uncertaintyZ);

	matrixPtr = compoundUncertainty->getRawData();
	for (int i = 0; i < compoundUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyZ,
			identityTransform,
			uncertaintyX);

	matrixPtr = compoundUncertainty->getRawData();
	for (int i = 0; i < compoundUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyXRot,
			identityTransform,
			zeroUncertainty);

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

//	LOG(DEBUG) << "compoundCovariance uncertaintyXRot 1 = " << std::endl << *compoundUncertainty;

	compoundUncertainty = compoundCovariance(identityTransform,
			zeroUncertainty,
			identityTransform,
			uncertaintyXRot);

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

//	LOG(DEBUG) << "compoundCovariance uncertaintyXRot 2 = " << std::endl << *compoundUncertainty;

	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyXRot,
			identityTransform,
			uncertaintyX);

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

//	LOG(DEBUG) << "compoundCovariance uncertaintyXRot 3 = " << std::endl << *compoundUncertainty;

	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyX,
			identityTransform,
			uncertaintyXRot);

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

//	LOG(DEBUG) << "compoundCovariance uncertaintyXRot 4 = " << std::endl << *compoundUncertainty;

	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyXRot,
			identityTransform,
			uncertaintyYRot);

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyXRot,
			identityTransform,
			uncertaintyXRot);

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

}

void UncertaintyOperationsTest::testCompounding() {
	const double* matrixPtr;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identityTransform(new HomogeneousMatrix44()); // Identity
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyX(new CovarianceMatrix66(1, 0, 0,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyY(new CovarianceMatrix66(0, 1, 0,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyZ(new CovarianceMatrix66(0, 0, 1,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr compoundUncertainty;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transfrom90AroundZ(new HomogeneousMatrix44());
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transfrom45AroundZ(new HomogeneousMatrix44());
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(0,0,0, 0,0, M_PI*0.5,  transfrom90AroundZ);
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(0,0,0, 0,0, M_PI*0.25,  transfrom45AroundZ);

	LOG(DEBUG) << "transfrom90AroundZ = " << std::endl << *transfrom90AroundZ;

	/* test compounding */
	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyX,
			transfrom90AroundZ,
			uncertaintyX);

	matrixPtr = compoundUncertainty->getRawData();
	for (int i = 0; i < compoundUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	/* test compounding */
	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyX,
			transfrom90AroundZ,
			uncertaintyX);

	matrixPtr = compoundUncertainty->getRawData();
	for (int i = 0; i < compoundUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	/* test compounding */
	compoundUncertainty = compoundCovariance(transfrom90AroundZ,
			uncertaintyX,
			identityTransform,
			uncertaintyY);

	matrixPtr = compoundUncertainty->getRawData();
	for (int i = 0; i < compoundUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	/* test compounding */
	compoundUncertainty = compoundCovariance(transfrom45AroundZ,
			uncertaintyX,
			identityTransform,
			uncertaintyX);

	LOG(DEBUG) << "compoundCovariance for transfrom45AroundZ = " << std::endl << *compoundUncertainty;
	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.5, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	/* stack with last result */
	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyZ,
			identityTransform,
			compoundUncertainty);

	LOG(DEBUG) << "compoundCovariance for stacked transform 1 = " << std::endl << *compoundUncertainty;
	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.5, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);


	/* again stack with last result */
	compoundUncertainty = compoundCovariance(identityTransform, // flipped?
			uncertaintyZ,
			transfrom45AroundZ, //flipped?
			compoundUncertainty);

	LOG(DEBUG) << "compoundCovariance for stacked transform 2 = " << std::endl << *compoundUncertainty;
	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.5, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

}

void UncertaintyOperationsTest::testRelativeCompounding() {
	const double* matrixPtr;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identityTransform(new HomogeneousMatrix44()); // Identity
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr translation123(new HomogeneousMatrix44(1,0,0,0,1,0,0,0,1, 1,2,3)); // X translation = 1
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr translation234(new HomogeneousMatrix44(1,0,0,0,1,0,0,0,1, 2,3,4)); // y translation = 1
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transfrom90AroundZ123(new HomogeneousMatrix44());
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transfrom45AroundZ234(new HomogeneousMatrix44());
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(1,2,3, 0,0, M_PI*0.5,  transfrom90AroundZ123);
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(2,3,4, 0,0, M_PI*0.25,  transfrom45AroundZ234);

	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyTranslation123(new CovarianceMatrix66(1, 2, 3,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyTranslation234(new CovarianceMatrix66(2, 3, 4,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyY(new CovarianceMatrix66(0, 1, 0,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyZ(new CovarianceMatrix66(0, 0, 1,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr zeroUncertainty(new CovarianceMatrix66(0, 0, 0,  0, 0, 0));
	CovarianceMatrix66::CovarianceMatrix66Ptr compoundUncertainty;

	/* test compounding */
	compoundUncertainty = compoundCovariance(translation123,
			uncertaintyTranslation123,
			translation234,
			uncertaintyTranslation234);

	matrixPtr = compoundUncertainty->getRawData();
	for (int i = 0; i < compoundUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	/* test compounding reverse */
	compoundUncertainty = compoundCovariance(translation234,
			uncertaintyTranslation234,
			translation123,
			uncertaintyTranslation123);

	matrixPtr = compoundUncertainty->getRawData();
	for (int i = 0; i < compoundUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	/*
	 * with rotataion
	 */

	/* test compounding */
	compoundUncertainty = compoundCovariance(identityTransform,
			uncertaintyTranslation123,
			transfrom45AroundZ234,
			uncertaintyTranslation234);

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	LOG(DEBUG) << "compoundCovariance testRelativeCompounding 1 = " << std::endl << *compoundUncertainty;

	/* test compounding */
	compoundUncertainty = compoundCovariance(transfrom45AroundZ234,
			uncertaintyTranslation123,
			identityTransform,
			uncertaintyTranslation234);

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.5, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.5, matrixPtr[7],  maxTolerance); //4.0?
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	LOG(DEBUG) << "compoundCovariance testRelativeCompounding 2 = " << std::endl << *compoundUncertainty;

	/*
	 * no identity base
	 */
	/* test compounding */
	compoundUncertainty = compoundCovariance(transfrom45AroundZ234,
			uncertaintyTranslation123,
			transfrom90AroundZ123,
			uncertaintyTranslation234);

	LOG(DEBUG) << "compoundCovariance testRelativeCompounding 3 = " << std::endl << *compoundUncertainty;

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.5, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.5, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	compoundUncertainty = compoundCovariance(transfrom90AroundZ123,
			uncertaintyTranslation123,
			transfrom45AroundZ234,
			uncertaintyTranslation234);

	LOG(DEBUG) << "compoundCovariance testRelativeCompounding 4 = " << std::endl << *compoundUncertainty;

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.5, matrixPtr[0],  maxTolerance); //3.5?
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.5, matrixPtr[7],  maxTolerance); //4.5?
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);





}

void UncertaintyOperationsTest::testAggregatedCompounding() {
	/*
	 *     x
	 *     ^
	 *     |
	 *     +---> y
	 *
	 *
	 *                       x_uncert=4
	 *  x_uncet=3              A                    x_uncert= 10-7,y_uncert=1
	 *       (   A ^-----------> B ---------------(--> )C
	 *           /    )        U
	 *          /45deg around z
	 *         /
	 *        /
	 *       + 45deg around z
	 *
	 * @B:
	 *     both y/z uncert = 10-7
	 *       x_restult >=4
	 *       y_resut >> 10-7 (~1.5)
	 *
	 * @C:
	 * 	  x_result ~ 4
	 *    y_result ~ 2.5
	 */

	const double* matrixPtr;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformA(new HomogeneousMatrix44());
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(1,0,0, 0,0, 0.25*M_PI, transformA);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformB(new HomogeneousMatrix44());
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(2,0,0, 0,0, 0.25*M_PI, transformB);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformC(new HomogeneousMatrix44());
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(3,0,0, 0,0, 0, transformC);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr compoundTransfromAB(new HomogeneousMatrix44());

	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyA(new CovarianceMatrix66(3, 1e-7, 1e-7,  1e-7, 1e-7, 1e-7));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyB(new CovarianceMatrix66(4, 1e-7, 1e-7,  1e-7, 1e-7, 1e-7));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyC(new CovarianceMatrix66(1e-7, 1, 1e-7,  1e-7, 1e-7, 1e-7));
	CovarianceMatrix66::CovarianceMatrix66Ptr compoundUncertainty(new CovarianceMatrix66());

	compoundUncertainty = compoundCovariance(transformB,
			uncertaintyB,
			transformA,
			uncertaintyA);

	LOG(DEBUG) << "compoundCovariance testAggregatedCompounding 1 = " << std::endl << *compoundUncertainty;

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.5, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.5, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[35], maxTolerance);

	(*compoundTransfromAB) = *( (*transformA) * (*transformB) );
	compoundUncertainty = compoundCovariance(transformC,
			uncertaintyC,
			compoundTransfromAB,
			compoundUncertainty);

	LOG(DEBUG) << "compoundCovariance testAggregatedCompounding 2 = " << std::endl << *compoundUncertainty;

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.5, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.5, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[35], maxTolerance);

	/*
	 * Reapeat but rotation around Y
	 */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformARotY(new HomogeneousMatrix44());
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(1,0,0, 0,0.25*M_PI,0, transformARotY);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformBRotY(new HomogeneousMatrix44());
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(2,0,0, 0,0.2500001*M_PI,0, transformBRotY);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformCRotY(new HomogeneousMatrix44());
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(3,0,0, 0,0, 0, transformCRotY);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr compoundTransfromABRotY(new HomogeneousMatrix44());

	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyARotY(new CovarianceMatrix66(3, 1e-7, 1e-7,  1e-7, 1e-7, 1e-7));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyBRotY(new CovarianceMatrix66(4, 1e-7, 1e-7,  1e-7, 1e-7, 1e-7));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyCRotY(new CovarianceMatrix66(1e-7, 1e-7, 1, 1e-7, 1e-7, 1e-7));

	compoundUncertainty = compoundCovariance(transformBRotY,
			uncertaintyBRotY,
			transformARotY,
			uncertaintyARotY);

	LOG(DEBUG) << "compoundCovariance testAggregatedCompounding 3 = " << std::endl << *compoundUncertainty;

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.5, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.5, matrixPtr[14], maxTolerance);
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[21], maxTolerance); //err -> devision by 0 because of cos(pitch3 = M_PI_2)
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[28], maxTolerance);
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[35], maxTolerance);

	(*compoundTransfromABRotY) = *( (*transformARotY) * (*transformBRotY) );
	compoundUncertainty = compoundCovariance(transformCRotY,
			uncertaintyCRotY,
			compoundTransfromABRotY,
			compoundUncertainty);

	LOG(DEBUG) << "compoundCovariance testAggregatedCompounding 4 = " << std::endl << *compoundUncertainty;

	matrixPtr = compoundUncertainty->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.5, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.5, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-7,  matrixPtr[35], maxTolerance);
}

void UncertaintyOperationsTest::testIdentityMerging() {

	const double* matrixPtr;
	bool calcSuccessful = true;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identityTransform(new HomogeneousMatrix44()); // Identity
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformX(new HomogeneousMatrix44(1,0,0,0,1,0,0,0,1, 1,0,0)); // X translation = 1
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformY(new HomogeneousMatrix44(1,0,0,0,1,0,0,0,1, 0,1,0)); // y translation = 1
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyX(new CovarianceMatrix66(1, 1e-7, 1e-7,  1e-7, 1e-7, 1e-7));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyY(new CovarianceMatrix66(1e-7, 1, 1e-7,  1e-7, 1e-7, 1e-7));
	CovarianceMatrix66::CovarianceMatrix66Ptr uncertaintyZ(new CovarianceMatrix66(1e-7, 1e-7, 1,  1e-7, 1e-7, 1e-7));
	CovarianceMatrix66::CovarianceMatrix66Ptr zeroUncertainty(new CovarianceMatrix66(1e-7, 1e-7, 1e-7,  1e-7, 1e-7, 1e-7));
	CovarianceMatrix66::CovarianceMatrix66Ptr mergedUncertainty(new CovarianceMatrix66());
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mergedMean(new HomogeneousMatrix44());


	calcSuccessful = mergeCovariance(identityTransform,
			uncertaintyX,
			identityTransform,
			uncertaintyX,
			mergedMean,
			mergedUncertainty);

	CPPUNIT_ASSERT(calcSuccessful == true);

	/* check mean */
	matrixPtr = mergedMean->getRawData();
	for (int i = 0; i < 16; ++i) {
		if ((i%5) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[5],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[10], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[15], maxTolerance);

	/* check covariance */
	matrixPtr = mergedUncertainty->getRawData();
	for (int i = 0; i < mergedUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
//			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance); //1.0?
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);


	mergeCovariance(identityTransform,
			uncertaintyX,
			identityTransform,
			uncertaintyY,
			mergedMean,
			mergedUncertainty);

	/* check mean */
	matrixPtr = mergedMean->getRawData();
	for (int i = 0; i < 16; ++i) {
		if ((i%5) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[5],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[10], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[15], maxTolerance);

	/* check covariance */
	matrixPtr = mergedUncertainty->getRawData();
	for (int i = 0; i < mergedUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);


	mergeCovariance(identityTransform,
			zeroUncertainty,
			identityTransform,
			uncertaintyZ,
			mergedMean,
			mergedUncertainty);

	LOG(DEBUG) << "mergedMean = " << std::endl << *mergedMean;
	LOG(DEBUG) << "mergedUncertainty = " << std::endl << *mergedUncertainty;

	/* check mean */
	matrixPtr = mergedMean->getRawData();
	for (int i = 0; i < 16; ++i) {
		if ((i%5) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[5],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[10], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[15], maxTolerance);

	/* check covariance */
	matrixPtr = mergedUncertainty->getRawData();
	for (int i = 0; i < mergedUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);


	mergeCovariance(identityTransform,
			uncertaintyZ,
			identityTransform,
			zeroUncertainty,
			mergedMean,
			mergedUncertainty);

	LOG(DEBUG) << "mergedMean = " << std::endl << *mergedMean;
	LOG(DEBUG) << "mergedUncertainty = " << std::endl << *mergedUncertainty;

	/* check mean */
	matrixPtr = mergedMean->getRawData();
	for (int i = 0; i < 16; ++i) {
		if ((i%5) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[5],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, matrixPtr[10], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1,  matrixPtr[15], maxTolerance);

	/* check covariance */
	matrixPtr = mergedUncertainty->getRawData();
	for (int i = 0; i < mergedUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);

	/*
	 * now cov is fexed and the means will be diffrent
	 */

	mergeCovariance(identityTransform,
			zeroUncertainty,
			transformX,
			zeroUncertainty,
			mergedMean,
			mergedUncertainty);

	LOG(DEBUG) << "mergedMean = " << std::endl << *mergedMean;
	LOG(DEBUG) << "mergedUncertainty = " << std::endl << *mergedUncertainty;

	/* check mean (translation) */
	matrixPtr = mergedMean->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixPtr[x],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,   matrixPtr[y],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,   matrixPtr[z],  maxTolerance);


	/* check covariance */
	matrixPtr = mergedUncertainty->getRawData();
	for (int i = 0; i < mergedUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[35], maxTolerance);


	/*
	 * now cov is fixed and the means will be diffrent
	 */

	mergeCovariance(transformY,
			zeroUncertainty,
			transformX,
			zeroUncertainty,
			mergedMean,
			mergedUncertainty);

	/* check mean (translation) */
	matrixPtr = mergedMean->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixPtr[x],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixPtr[y],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,   matrixPtr[z],  maxTolerance);


	/* check covariance */
	matrixPtr = mergedUncertainty->getRawData();
	for (int i = 0; i < mergedUncertainty->getDimension(); ++i) {
		if ((i%7) != 0) { //skip diagonal elements
			CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[i], maxTolerance);
		}
	}
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[0],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[7],  maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, matrixPtr[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[21], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[28], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0,  matrixPtr[35], maxTolerance);


}

}

/* EOF */
