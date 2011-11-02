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
#include "util/OSGPointCloudVisualizer.h"
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

	/*
	 * set up icp
	 *
	 * pointCorrespondence
	 * 0 k-d tree
	 * 1 ANN
	 * 2 FLANN
	 * 3 STANN
	 *
	 * rigidTransformationEstimation
	 * 0 SVD
	 * 1 QUAT
	 * 2 HELIX
	 * 3 APX
	 * (ORTHO does not work)
	 *
	 */
	 int pointCorrespondence = 0;
	 int rigidTransformationEstimation = 0;

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
//	} else if (argc == 5) {
//		filename1 = argv[1];
//		filename2 = argv[2];
//		pointCorrespondence = atoi[3];
//		rigidTransformationEstimation = atoi[4];
//		cout << filename1 << ", " << filename2 << endl;
	} else {
		cout << "Usage: " << argv[0] << " <filename>" << " <filename>" << endl;
		return -1;
	}

	Timer timer0;
	long double tmpTimeStamp = 0.0;
	Benchmark icpBenchmark("icp_blackbox");
	icpBenchmark.output << "#Algorithm index combination, time for matching [ms], number iterations, resulting error" << endl;

	PointCloud3D* pointCloud1 = new PointCloud3D();
	PointCloud3D* pointCloud2 = new PointCloud3D();

	pointCloud1->readFromTxtFile(filename1);
	cout << "Size of first point cloud: " << pointCloud1->getSize() << endl;
	pointCloud2->readFromTxtFile(filename2);
	cout << "Size of second point cloud: " << pointCloud2->getSize() << endl;

//	PointCloud3D* pointCloud3 = new PointCloud3D();
//	stringstream tmpSteam;
//	tmpSteam << *pointCloud1;
//	tmpSteam << *pointCloud2;
//	tmpSteam >> *pointCloud3;
//	cout << "Size of third point cloud: " << pointCloud3->getSize() << endl;

	/* manipulate second point cloud */
//	AngleAxis<double> rotation(M_PI_2l/4.0, Vector3d(1,0,0));
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;


//	OSGPointCloudVisualizer* viewer = new OSGPointCloudVisualizer();
	//viewer->addPointCloud(pointCloud3, 1.0f, 0.0f, 0.0f, 1.0f); //combined version
	//viewer->addPointCloud(pointCloud2, 1.0f, 0.0f, 0.0f, 1.0f);
//	viewer->addPointCloud(pointCloud1, 0.0f, 1.0f, 0.0f, 1.0f); //initial


	IIterativeClosestPoint* icp = 0; //abstract interface to ICP
	IIterativeClosestPointSetup* icpConfigurator = 0; //abstract interface to ICP
	IPointCorrespondence* assigner = 0; //only needed for generic icp
	INearestPoint3DNeighbor* nearestNeigbourFinder = 0;
	IRigidTransformationEstimation* estimator = 0;

	IterativeClosestPoint* concreteIcp = 0;
//	IterativeClosestPoint6DSLAM* concreteIcp6DSLAM;

//	/* set up assigner */
//	INearestNeighbor* nearestNeigbourFinder = new NearestNeighborFLANN();
////	nearestNeigbourFinder->setMaxDistance(1);
//	NearestNeighborFLANN* nearestNeigbourFinderFLANN = dynamic_cast<NearestNeighborFLANN*>(nearestNeigbourFinder); //polymorph down cast
//	FLANNParameters parameters;
//	parameters = nearestNeigbourFinderFLANN->getParameters();
////	parameters.algorithm = KMEANS;
//	parameters.target_precision = -1;
//	parameters.trees = 16;
//	nearestNeigbourFinderFLANN->setParameters(parameters);


	for (int i = 0; i <= 3; ++i) { // loop over all combinations
		for (int j = 0; j <= 3; ++j) {
			pointCorrespondence = i;
			rigidTransformationEstimation = j;

			/* alwas read _fresh_ data */
			pointCloud1->getPointCloud()->clear();
			pointCloud1->readFromTxtFile(filename1);
			cout << "Size of first point cloud: " << pointCloud1->getSize() << endl;
			pointCloud2->getPointCloud()->clear();
			pointCloud2->readFromTxtFile(filename2);
			cout << "Size of second point cloud: " << pointCloud2->getSize() << endl;

			/* set up point-to-point correspondence */
			switch (pointCorrespondence) {
			case 0:
				assigner = new PointCorrespondenceKDTree();
				cout << "INFO: Using PointCorrespondenceKDTree." << endl;
				break;
			case 1:
				nearestNeigbourFinder = new NearestNeighborANN();
				assigner = new PointCorrespondenceGenericNN(nearestNeigbourFinder);
				cout << "INFO: Using PointCorrespondenceGenericNN with NearestNeighborANN." << endl;
				break;
			case 2:
				nearestNeigbourFinder = new NearestNeighborFLANN();
				assigner = new PointCorrespondenceGenericNN(nearestNeigbourFinder);
				cout << "INFO: Using PointCorrespondenceGenericNN with NearestNeighborFLANN." << endl;
				break;
			case 3:
				nearestNeigbourFinder = new NearestNeighborSTANN();
				assigner = new PointCorrespondenceGenericNN(nearestNeigbourFinder);
				cout << "INFO: Using PointCorrespondenceGenericNN with NearestNeighborSTANN." << endl;
				break;
			default:
				cout << "ERROR: No pointCorrespondence algorithm given." << endl;
				return -1;
				break;
			}

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
			icpBenchmark.output << i << j << " ";


			//	icp = new IterativeClosestPoint(assigner, estimator);
			concreteIcp = new IterativeClosestPoint(assigner, estimator);
			icp = dynamic_cast<IIterativeClosestPoint*>(concreteIcp);
			icpConfigurator = dynamic_cast<IIterativeClosestPointSetup*>(concreteIcp);


			/* invoke ICP */
			IHomogeneousMatrix44* resultTransformation = new HomogeneousMatrix44();
			icpConfigurator->setMaxIterations(100);
			timer0.reset();
			icp->match(pointCloud1, pointCloud2, resultTransformation);
			tmpTimeStamp = timer0.getElapsedTime();
			icpBenchmark.output << tmpTimeStamp << " ";
			icpBenchmark.output << concreteIcp->icpresultIterations << " ";
			icpBenchmark.output << concreteIcp->icpResultError << " ";
			icpBenchmark.output << endl;
			cout << concreteIcp->icpresultIterations << "ITS, " << concreteIcp->icpResultError << "RMS" << endl;
			cout <<  "Time for matching [ms]: " << tmpTimeStamp << endl;

		}
	}

//	viewer->visualizePointCloud(pointCloud2, 1.0f, 1.0f, 1.0f, 1.0f);
	//viewer->visualizePointCloud(pointCloud3, 1.0f,0.0f,1.0f,1.0f);


	/* clean up */
	delete icp;
	//delete estimator; //TODO: crashes (+following lines)
	//delete assigner;
//	delete viewer;
	delete pointCloud2;
	delete pointCloud1;

	return 0;
}

/* EOF */
