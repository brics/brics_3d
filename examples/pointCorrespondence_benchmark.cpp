/**
 * @file 
 * pointCorrespondence_benchmark.cpp
 *
 * @date: Mar 29, 2010
 * @author: sblume
 */


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
	Benchmark ptCorresBenchmark("pointCorrespondence_benchmark");
	ptCorresBenchmark.output << "#timing , %correct values" << endl;
	static const double maxTolerance = 0.00001;
	int correctCounter = 0;
	int incorrectCounter = 0;
	double correctness = 0; // in %

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
	Translation<double,3> translation(0.1, 0.1, 0.1);
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
//	transformation = rotation;
	transformation = translation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloud2->homogeneousTransformation(homogeneousTrans);
	cout << *homogeneousTrans;

	/*
	 * set up pointCorrespondence
	 * 0 k-d tree
	 * 1 ANN
	 * 2 FLANN
	 * 3 STANN
	 */
	int pointCorrespondence = 0;
	INearestPoint3DNeighbor* nearestNeigbourFinder = 0;
	IPointCorrespondence* assigner = 0;


	for (int i = 0; i <= 3; ++i) {

		pointCorrespondence = i;

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

		/* we already know the correspondences...*/
		vector<CorrespondencePoint3DPair>* pointPairs = new vector<CorrespondencePoint3DPair>();



		timer0.reset();
		assigner->createNearestNeighborCorrespondence(pointCloud1, pointCloud2, pointPairs);
		tmpTimeStamp = timer0.getElapsedTime();
		ptCorresBenchmark.output << tmpTimeStamp << " ";

		correctCounter = 0;
		incorrectCounter = 0;

	    cout << (int)(*pointCloud1->getPointCloud())[i].getX() - (int)(*pointPairs)[i].firstPoint.getX() << endl;

		for (unsigned int i = 0;  i < pointPairs->size(); ++i) {
			/*
			 * expectation as the same point cloud is used:
			 * direct correspondence from one cloud the other
			 */
			//CPPUNIT_ASSERT_EQUAL(0, (int)(*pointPairs)[i].firstIndex);
			if (
					(abs(((int)(*pointCloud1->getPointCloud())[i].getX() - (int)(*pointPairs)[i].firstPoint.getX())) < maxTolerance) &&
					(abs(((int)(*pointCloud2->getPointCloud())[i].getX() - (int)(*pointPairs)[i].secondPoint.getX())) < maxTolerance) &&

					(abs(((int)(*pointCloud1->getPointCloud())[i].getY() - (int)(*pointPairs)[i].firstPoint.getY()))  < maxTolerance) &&
					(abs(((int)(*pointCloud2->getPointCloud())[i].getY() - (int)(*pointPairs)[i].secondPoint.getY())) < maxTolerance) &&

					(abs(((int)(*pointCloud1->getPointCloud())[i].getZ() - (int)(*pointPairs)[i].firstPoint.getZ())) < maxTolerance) &&
				    (abs(((int)(*pointCloud2->getPointCloud())[i].getZ() - (int)(*pointPairs)[i].secondPoint.getZ())) < maxTolerance)
				) {
				correctCounter++;
			} else {
				incorrectCounter++;
			}

		} //end for

		correctness = ((double)correctCounter / (double)(correctCounter + incorrectCounter));
		cout << correctCounter << "/" << correctCounter + incorrectCounter << " => "<< correctness << endl;
		ptCorresBenchmark.output << correctness;
		ptCorresBenchmark.output << endl;
	}

	return 0;
}


/* EOF */

