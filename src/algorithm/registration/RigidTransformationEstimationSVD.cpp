/**
 * @file 
 * RigidTransformationEstimationSVD.cpp
 *
 * @date: Dec 7, 2009
 * @author: sblume
 */

#include "RigidTransformationEstimationSVD.h"

//#define MAX_OPENMP_NUM_THREADS 4
#define OPENMP_NUM_THREADS 4
//
//#define WANT_STREAM ///< define the WANT stream :)
//#include <string>
//using std::string;
#include <iostream>
//using std::cout;
//using std::cerr;
//using std::endl;
//#include <fstream>
//using std::ifstream;
//
//#include "6dslam/src/scan.h"
//
//#include "6dslam/src/icp6Dapx.h"
#include "6dslam/src/icp6Dsvd.h"
//#include "6dslam/src/icp6Dquat.h"
//#include "6dslam/src/icp6Dortho.h"
//#include "6dslam/src/icp6Dhelix.h"
//#include "6dslam/src/icp6D.h"
//#include "6dslam/src/lum6Deuler.h"
//#include "6dslam/src/lum6Dquat.h"
//#include "6dslam/src/ghelix6DQ2.h"
//#include "6dslam/src/elch6Deuler.h"
//#include "6dslam/src/elch6Dquat.h"
//#include "6dslam/src/elch6DunitQuat.h"
//#include "6dslam/src/elch6Dslerp.h"
//#include "6dslam/src/graphSlam6D.h"
//#include "6dslam/src/gapx6D.h"
//#include "6dslam/src/graph.h"
//#include "6dslam/src/globals.icc"


namespace BRICS_3D {

RigidTransformationEstimationSVD::RigidTransformationEstimationSVD() {
	// TODO Auto-generated constructor stub

}

RigidTransformationEstimationSVD::~RigidTransformationEstimationSVD() {
	// TODO Auto-generated destructor stub
}

double RigidTransformationEstimationSVD::estimateTransformation(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2, std::vector<CorrespondencePoint3DPair>* pointPairs, IHomogeneousMatrix44* resultTransformation){
	bool quiet = true;
	icp6Dminimizer *errorMinimizer = new icp6D_SVD(quiet);

	vector<PtPair> minimizerPointPairs;
	double* alignxf = new double [16];
	double centroid_m[3] = {0.0, 0.0, 0.0}; // TODO
	double centroid_d[3] = {0.0, 0.0, 0.0}; // TODO

	/* convert data structures */
	for (int i = 0; i < pointPairs->size(); ++i) {
		double firstPoint[3];
		firstPoint[0] = (*pointPairs)[i].firstPoint.x;
		firstPoint[1] = (*pointPairs)[i].firstPoint.y;
		firstPoint[2] = (*pointPairs)[i].firstPoint.z;

		double secondPoint[3];
		secondPoint[0] = (*pointPairs)[i].secondPoint.x;
		secondPoint[1] = (*pointPairs)[i].secondPoint.y;
		secondPoint[2] = (*pointPairs)[i].secondPoint.z;

		PtPair pair;
		pair.p1 = firstPoint;
		pair.p2 = secondPoint;
		minimizerPointPairs.push_back(pair);

	}

	errorMinimizer->Point_Point_Align(minimizerPointPairs, alignxf, centroid_m, centroid_d);
	for (int i = 0; i < 16; ++i) {
		std::cout << alignxf[i];
		if (((i+1)%4) == 0) {
			std::cout << std::endl;
		}
	}

	double* resultRawData;
	resultRawData = resultTransformation->setRawData();
	for (int i = 0; i < 16; ++i) {
		resultRawData[i] = alignxf[i];
	}

//	  double Point_Point_Align(const vector<PtPair>& Pairs, double *alignxf,
//						  const double centroid_m[3], const double centroid_d[3]);
}

}

/* EOF */
