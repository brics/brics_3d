/**
 * @file
 * icp_test.cpp
 * 
 * @brief Simple test file to experiment with ICP (from IVT library)
 *
 * @author: Sebastian Blumenthal
 * @date: Aug 31, 2009
 * @version: 0.1
 */

#ifdef WIN32
 #define _USE_MATH_DEFINES
#endif 

#include <iostream>
#include <sstream>
#include "util/OSGPointCloudVisualizer.h"
#include "core/PointCloud3D.h"
#include "core/HomogeneousMatrix44.h"
#include "algorithm/nearestNeighbor/NearestNeighborFLANN.h"
#include "algorithm/registration/IterativeClosestPoint6DSLAM.h"
#include "algorithm/registration/PointCorrespondenceKDTree.h"
#include "algorithm/registration/PointCorrespondenceGenericNN.h"
#include "algorithm/registration/RigidTransformationEstimationSVD.h"
#include "algorithm/registration/RigidTransformationEstimationHELIX.h"
#include "algorithm/registration/RigidTransformationEstimationAPX.h"
#include "algorithm/registration/RigidTransformationEstimationORTHO.h"
#include "algorithm/registration/IterativeClosestPoint.h"
#include "algorithm/registration/IIterativeClosestPointSetup.h"

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
		cout << "Usage: " << argv[0] << " <filename>" << " <filename>" << endl;

		char defaultFilename1[255] = { BRICS_MODELS_DIR };
		strcat(defaultFilename1, "/scan1.txt\0");
		filename1 = defaultFilename1;

		char defaultFilename2[255] = { BRICS_MODELS_DIR };
		strcat(defaultFilename2, "/scan2.txt\0");
		filename2 = defaultFilename2;

		cout << "Trying to get default files: " << filename1 << ", " << filename2 << endl;
	} else if (argc == 2) {
		string filename1 = argv[1];
		string filename2 = argv[2];
		cout << filename1 << ", " << filename2 << endl;
	} else {
		cout << "Usage: " << argv[0] << " <filename>" << " <filename>" << endl;
		return -1;
	}

	PointCloud3D* pointCloud1 = new PointCloud3D();
	pointCloud1->readFromTxtFile(filename1);
	cout << "Size of first point cloud: " << pointCloud1->getSize() << endl;

	PointCloud3D* pointCloud2 = new PointCloud3D();
	pointCloud2->readFromTxtFile(filename2);
	cout << "Size of second point cloud: " << pointCloud2->getSize() << endl;

	PointCloud3D* pointCloud3 = new PointCloud3D();
	stringstream tmpSteam;
	tmpSteam << *pointCloud1;
	tmpSteam << *pointCloud2;
	tmpSteam >> *pointCloud3;
	cout << "Size of third point cloud: " << pointCloud3->getSize() << endl;

	/* manipulate second point cloud */
//	AngleAxis<double> rotation(M_PI_2l/4.0, Vector3d(1,0,0));
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;


	OSGPointCloudVisualizer* viewer = new OSGPointCloudVisualizer();
	//viewer->addPointCloud(pointCloud3, 1.0f, 0.0f, 0.0f, 1.0f); //combined version
	//viewer->addPointCloud(pointCloud2, 1.0f, 0.0f, 0.0f, 1.0f);
	viewer->addPointCloud(pointCloud1, 0.0f, 1.0f, 0.0f, 1.0f); //initial

	/*
	 * set up icp
	 */
	int algorithm = 0; //TODO icp switch e.g. input parameter,...
	int pointCorrespondence = 1; //TODO pointCorrespondence switch e.g. input parameter,...

	IIterativeClosestPoint* icp; //abstract interface to ICP
	IIterativeClosestPointSetup* icpConfigurator; //abstract interface to ICP
	IPointCorrespondence* assigner; //only needed for generic icp

	IterativeClosestPoint* concreteIcp;
	IterativeClosestPoint6DSLAM* concreteIcp6DSLAM;

	/* set up assigner */
	INearestNeighbor* nearestNeigbourFinder = new NearestNeighborFLANN();
//	nearestNeigbourFinder->setMaxDistance(1);
	NearestNeighborFLANN* nearestNeigbourFinderFLANN = dynamic_cast<NearestNeighborFLANN*>(nearestNeigbourFinder); //polymorph down cast
	FLANNParameters parameters;
	parameters = nearestNeigbourFinderFLANN->getParameters();
//	parameters.algorithm = KMEANS;
	parameters.target_precision = -1;
	parameters.trees = 16;
	nearestNeigbourFinderFLANN->setParameters(parameters);

	switch (pointCorrespondence) {
		case 0:
			assigner = new PointCorrespondenceGenericNN(nearestNeigbourFinder);
			cout << "INFO: Using PointCorrespondenceGenericNN icp." << endl;
			break;
		case 1:
			assigner = new PointCorrespondenceKDTree(); //only needed for generic icp
			cout << "INFO: Using PointCorrespondenceKDTree icp." << endl;
			break;
		default:
			cout << "ERROR: No pointCorrespondence algorithm given." << endl;
			return -1;
			break;
	}


	/* setup estimator */
	IRigidTransformationEstimation* estimator = new RigidTransformationEstimationAPX(); //only needed for generic icp

	switch (algorithm) {
		case 0:
//			icp = new IterativeClosestPoint(assigner, estimator);
			concreteIcp = new IterativeClosestPoint(assigner, estimator);
			icp = dynamic_cast<IIterativeClosestPoint*>(concreteIcp);
			icpConfigurator = dynamic_cast<IIterativeClosestPointSetup*>(concreteIcp);

			cout << "INFO: Using generic icp." << endl;
			break;
		case 1:
//			icp = new IterativeClosestPoint6DSLAM();
			concreteIcp6DSLAM = new IterativeClosestPoint6DSLAM();
			icp = dynamic_cast<IIterativeClosestPoint*>(concreteIcp6DSLAM);
			icpConfigurator = dynamic_cast<IIterativeClosestPointSetup*>(concreteIcp6DSLAM);
			cout << "INFO: Using 6Dslam icp." << endl;
			break;
		default:
			cout << "ERROR: No algorithm given." << endl;
			return -1;
			break;
	}

	/* invoke ICP */
	IHomogeneousMatrix44* resultTransformation = new HomogeneousMatrix44();
	icpConfigurator->setMaxIterations(100);
	icp->match(pointCloud1, pointCloud2, resultTransformation);


	viewer->visualizePointCloud(pointCloud2, 1.0f, 1.0f, 1.0f, 1.0f);
	//viewer->visualizePointCloud(pointCloud3, 1.0f,0.0f,1.0f,1.0f);


	/* clean up */
	delete icp;
//	delete estimator; //TODO: crashes (+following lines)
//	delete assigner;
	delete viewer;
	delete pointCloud2;
	delete pointCloud1;

	return 0;
}

/* EOF */
