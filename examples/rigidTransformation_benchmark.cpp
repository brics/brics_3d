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

#ifdef WIN32
 #define _USE_MATH_DEFINES
#endif

#include <iostream>
#include <sstream>
#include "core/PointCloud3D.h"
#include "core/HomogeneousMatrix44.h"
#include "algorithm/nearestNeighbor/NearestNeighborANN.h"
#include "algorithm/nearestNeighbor/NearestNeighborFLANN.h"
#include "algorithm/nearestNeighbor/NearestNeighborSTANN.h"
#include "algorithm/registration/PointCorrespondenceKDTree.h"
#include "algorithm/registration/PointCorrespondenceGenericNN.h"
#include "algorithm/registration/RigidTransformationEstimationSVD.h"
#include "algorithm/registration/RigidTransformationEstimationHELIX.h"
#include "algorithm/registration/RigidTransformationEstimationAPX.h"
#include "algorithm/registration/RigidTransformationEstimationORTHO.h"
#include "algorithm/registration/RigidTransformationEstimationQUAT.h"
#include "algorithm/registration/IterativeClosestPoint.h"
#include "algorithm/registration/IIterativeClosestPointSetup.h"
#include "util/Timer.h"
#include "util/Benchmark.h"

#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;
using namespace BRICS_3D;

//#include "Image/ImageProcessor.h"
//#include "Image/ByteImage.h"
//#include <stdio.h>

int main(int argc, char **argv) {

	/* check arguments */
	string filename1;
	string filename2;
	if (argc == 1) {
		cout << "Usage: " << argv[0] << " <filename>" << " <filename>" << "[p2p-algo transEst-algo]" << endl;

		char defaultFilename1[255] = { BRICS_MODELS_DIR };
		strcat(defaultFilename1, "/scan1.txt\0");
		filename1 = defaultFilename1;

		char defaultFilename2[255] = { BRICS_MODELS_DIR };
		strcat(defaultFilename2, "/scan2.txt\0");
		filename2 = defaultFilename2;

		cout << "Trying to get default files: " << filename1 << ", " << filename2 << endl;
	} else if (argc == 3) {
		filename1 = argv[1];
		filename2 = argv[2];
		cout << filename1 << ", " << filename2 << endl;
	} else {
		cout << "Usage: " << argv[0] << " <filename>" << " <filename>" << endl;
		return -1;
	}

	Timer timer0;
	long double tmpTimeStamp = 0.0;
	Benchmark rigidTransBenchmark("rigidTransformation_benchmark");
	rigidTransBenchmark.output << "#timing , resulting RMS error, deviation from known vector" << endl;

	PointCloud3D* pointCloud1 = new PointCloud3D();
	PointCloud3D* pointCloud2 = new PointCloud3D();

	pointCloud1->readFromTxtFile(filename1);
//	cout << "Size of first point cloud: " << pointCloud1->getSize() << endl;
//	pointCloud2->readFromTxtFile(filename2);
	stringstream tmpSteam;
	tmpSteam << *pointCloud1;
	tmpSteam >> *pointCloud2;
	cout << "Size of first point cloud: " << pointCloud1->getSize() << endl;
	cout << "Size of second point cloud: " << pointCloud2->getSize() << endl;

	/* manipulate second point cloud */
	Translation<double,3> translation(1, 1, 1);
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
//	transformation = rotation;
	transformation = translation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloud2->homogeneousTransformation(homogeneousTrans);
	cout << *homogeneousTrans;

	/*
	 * rigidTransformationEstimation
	 * 0 SVD
	 * 1 QUAT
	 * 2 HELIX
	 * 3 APX
	 * (ORTHO does not work)
	 */
	int rigidTransformationEstimation = 0;
	IRigidTransformationEstimation* estimator = 0;

	for (int i = 0; i < 4; ++i) {

		rigidTransformationEstimation = i;

		/* setup estimator */
		switch (rigidTransformationEstimation) {
		case 0:
			estimator = new RigidTransformationEstimationSVD();
			cout << "INFO: Using RigidTransformationEstimationSVD." << endl;
			break;
		case 1:
			estimator = new RigidTransformationEstimationQUAT();
			cout << "INFO: Using RigidTransformationEstimationQUAT." << endl;
			break;
		case 2:
			estimator = new RigidTransformationEstimationHELIX();
			cout << "INFO: Using RigidTransformationEstimationHELIX." << endl;
			break;
		case 3:
			estimator = new RigidTransformationEstimationAPX();
			cout << "INFO: Using RigidTransformationEstimationAPX." << endl;
			break;

		default:
			cout << "ERROR: No rigidTransformationEstimation algorithm given." << endl;
			return -1;
			break;
		}

		/* we already know the correspondences...*/
		vector<CorrespondencePoint3DPair>* pointPairs = new vector<CorrespondencePoint3DPair>();
		for (unsigned int i = 0; i < pointCloud1->getSize(); ++i) {
			Point3D firstPoint = (*pointCloud1->getPointCloud())[i];
			Point3D secondPoint = (*pointCloud2->getPointCloud())[i];

			CorrespondencePoint3DPair tmpPair(firstPoint, secondPoint);
			pointPairs->push_back(tmpPair);
		}

		double resutError;
		IHomogeneousMatrix44* reusultTransformation = new HomogeneousMatrix44();
		timer0.reset();
		resutError = estimator->estimateTransformation(pointPairs, reusultTransformation);
		tmpTimeStamp = timer0.getElapsedTime();
		rigidTransBenchmark.output << tmpTimeStamp << " ";
		rigidTransBenchmark.output << resutError << " " ;

		/* test if initial and resulting homogeneous transformations are the same */
		const double* matrix1 = homogeneousTrans->getRawData();
		const double* matrix2 = reusultTransformation->getRawData();
		double MS = 0;
		for (int i = 0; i < 16; ++i) {
			MS += ((matrix1[i] - matrix2[i]) * (matrix1[i] - matrix2[i]));
		}
		cout << MS << endl;
		cout << *reusultTransformation;
		double RMS = sqrt(MS);
		rigidTransBenchmark.output << RMS << endl;
	}

	return 0;
}


/* EOF */
