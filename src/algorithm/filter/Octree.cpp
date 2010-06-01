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
