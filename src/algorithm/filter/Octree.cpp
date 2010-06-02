/**
 * @file 
 * Octree.cpp
 *
 * @date: Feb 18, 2010
 * @author: sblume
 */

#include "Octree.h"
#include "6dslam/src/octtree.h"
#include "assert.h"
#include <stdexcept>

using std::runtime_error;

namespace BRICS_3D {

Octree::Octree() {
	this->voxelSize = 0;
}

Octree::~Octree() {

}

void Octree::createOctree(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud) {
	assert(originalPointCloud != 0);
	assert(resultPointCloud != 0);

	resultPointCloud->getPointCloud()->clear();

	if (voxelSize <=0) {
		for (int i = 0; i < static_cast<int>(originalPointCloud->getSize()); ++i) { //just copy data
			resultPointCloud->addPoint((*originalPointCloud->getPointCloud())[i]);
		}
		return;
	}

	/* prepare data */
	double** tmpPointCloudPoints = new double*[originalPointCloud->getSize()];
	for (unsigned int i = 0; i < originalPointCloud->getSize(); i++) {
		tmpPointCloudPoints[i] = new double[3];
		tmpPointCloudPoints[i][0] = (*originalPointCloud->getPointCloud())[i].getX();
		tmpPointCloudPoints[i][1] = (*originalPointCloud->getPointCloud())[i].getY();
		tmpPointCloudPoints[i][2] = (*originalPointCloud->getPointCloud())[i].getZ();
	}

	/* create octree */
	OctTree *octree = new OctTree(tmpPointCloudPoints, originalPointCloud->getSize(), this->voxelSize);

	/* process results */
	resultPointCloud->getPointCloud()->clear();
	vector<double*> center;
	center.clear();
	octree->GetOctTreeCenter(center);

	for (int i = 0; i < static_cast<int>(center.size()); ++i) {
		Point3D tmpPoint = Point3D();
		tmpPoint.setX(center[i][0]);
		tmpPoint.setY(center[i][1]);
		tmpPoint.setZ(center[i][2]);
		resultPointCloud->addPoint(tmpPoint);
	}

	/* some plausibility checks */
	assert (resultPointCloud->getSize() == center.size());
	assert (resultPointCloud->getSize() <= originalPointCloud->getSize());

	/* clean up */
	delete octree;
	for (int i = 0; i < static_cast<int>(originalPointCloud->getSize()); ++i) {
		delete [] tmpPointCloudPoints[i];
	}
	delete [] tmpPointCloudPoints;
}

void Octree::partitionPointCloud(PointCloud3D* pointCloud, std::vector<PointCloud3D*>* pointCloudCells) {
	assert(pointCloud != 0);
	assert(pointCloudCells != 0);

	unsigned int totalPointsCount = 0; //for plausibility check

	pointCloudCells->clear();
	if (voxelSize <=0) {
		PointCloud3D* tmpPointCloud = new PointCloud3D();
		for (int i = 0; i < static_cast<int>(pointCloud->getSize()); ++i) { //just copy data
			tmpPointCloud->addPoint((*pointCloud->getPointCloud())[i]);
			totalPointsCount++;
		}
		pointCloudCells->push_back(tmpPointCloud);
		assert (pointCloud->getSize() == totalPointsCount); //plausibility check
		return;
	}

	/* prepare data */
	double** tmpPointCloudPoints = new double*[pointCloud->getSize()];
	for (unsigned int i = 0; i < pointCloud->getSize(); i++) {
		tmpPointCloudPoints[i] = new double[3];
		tmpPointCloudPoints[i][0] = (*pointCloud->getPointCloud())[i].getX();
		tmpPointCloudPoints[i][1] = (*pointCloud->getPointCloud())[i].getY();
		tmpPointCloudPoints[i][2] = (*pointCloud->getPointCloud())[i].getZ();
	}

	/* create octree */
	OctTree *octree = new OctTree(tmpPointCloudPoints, pointCloud->getSize(), this->voxelSize);

	/* process results */
	vector<vector<double*> > partition;
	partition.clear();
	octree->GetOctTreePartition(partition);

	for (unsigned int i = 0; i < partition.size(); ++i) { // each partition/cell
		PointCloud3D* tmpPointCloud = new PointCloud3D();
		for (unsigned int j = 0; j < partition[i].size(); ++j) { // each point in a partition/cell
			Point3D tmpPoint = Point3D();
			vector<double*> tmpResultPoint;
			tmpResultPoint = partition[i];
			tmpPoint.setX(tmpResultPoint[j][0]);
			tmpPoint.setY(tmpResultPoint[j][1]);
			tmpPoint.setZ(tmpResultPoint[j][2]);
			tmpPointCloud->addPoint(tmpPoint);
			totalPointsCount++;
		}
		pointCloudCells->push_back(tmpPointCloud);
	}

	/* plausibility check */
	assert (pointCloud->getSize() == totalPointsCount);

	/* clean up */
	delete octree;
}

void Octree::setVoxelSize(double voxelSize) {
	if (voxelSize < 0.0) {
		throw runtime_error("ERROR: voxelSize for Octree cannot be less than 0.");
	}
	this->voxelSize = voxelSize;
}

double Octree::getVoxelSize() {
	return this->voxelSize;
}


}

/* EOF */
