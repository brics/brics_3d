/**
 * @file 
 * PointCorrespondenceKDTree.cpp
 *
 * @date: Dec 3, 2009
 * @author: sblume
 */

#include "algorithm/registration/PointCorrespondenceKDTree.h"

#define MAX_OPENMP_NUM_THREADS 4
#include "6dslam/src/d2tree.h"
#include "6dslam/src/kd.h"
#include "6dslam/src/kdc.h"

#include <iostream>
#include <assert.h>

using std::cout;
using std::endl;
namespace BRICS_3D {

PointCorrespondenceKDTree::PointCorrespondenceKDTree() {

}

PointCorrespondenceKDTree::~PointCorrespondenceKDTree() {

}

void PointCorrespondenceKDTree::createNearestNeighborCorrespondence(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2, std::vector<CorrespondencePoint3DPair>* resultPointPairs) {
	assert(pointCloud1 != 0);
	assert(pointCloud2 != 0);
	assert(resultPointPairs != 0);

	double *centroid_m = new double[3];
	double *centroid_d = new double[3];
	double maxMatchingDistance = 50;

	resultPointPairs->clear();

	/* prepare data */
	double** pointCloud1Points = new double*[pointCloud1->getSize()];
	for (int i = 0; i < pointCloud1->getSize(); i++) {
		pointCloud1Points[i] = new double[3];
		pointCloud1Points[i][0] = (*pointCloud1->getPointCloud())[i].x;
		pointCloud1Points[i][1] = (*pointCloud1->getPointCloud())[i].y;
		pointCloud1Points[i][2] = (*pointCloud1->getPointCloud())[i].z;
	}

	KDtree* kDTree = new KDtree(pointCloud1Points, pointCloud1->getSize());
//	cout << "k-d tree created" << endl;

//	unsigned int numpts_target;
//	KDtree *kd = 0;
//	kd = new KDtree(Source->points_red, Source->points_red_size);
//	numpts_target = Target->points_red_size;

	for (unsigned int i = 0; i < pointCloud2->getSize(); i++) {
		//if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only

		double queryPoint[3];
		queryPoint[0] = (*pointCloud2->getPointCloud())[i].x;
		queryPoint[1] = (*pointCloud2->getPointCloud())[i].y;
		queryPoint[2] = (*pointCloud2->getPointCloud())[i].z;

		double *closest = kDTree->FindClosest(queryPoint, maxMatchingDistance, 0);
		if (closest) {
			centroid_m[0] += queryPoint[0];
			centroid_m[1] += queryPoint[1];
			centroid_m[2] += queryPoint[2];
			centroid_d[0] += closest[0];
			centroid_d[1] += closest[1];
			centroid_d[2] += closest[2];

			Point3D firstPoint = Point3D (closest[0], closest[1], closest[2]);
			Point3D secondPoint = Point3D (queryPoint[0], queryPoint[1], queryPoint[2]);

			CorrespondencePoint3DPair foundPair(firstPoint, secondPoint);
			resultPointPairs->push_back(foundPair);
		}
	}
//	centroid_m[0] /= pairs[thread_num].size();
//	centroid_m[1] /= pairs[thread_num].size();
//	centroid_m[2] /= pairs[thread_num].size();
//	centroid_d[0] /= pairs[thread_num].size();
//	centroid_d[1] /= pairs[thread_num].size();
//	centroid_d[2] /= pairs[thread_num].size();

	delete kDTree;

	return;
}

}

/* EOF */
