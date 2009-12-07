/**
 * @file 
 * PointCorrespondenceKDTree.h
 *
 * @date: Dec 3, 2009
 * @author: sblume
 */

#ifndef POINTCORRESPONDANCEKDTREE_H_
#define POINTCORRESPONDANCEKDTREE_H_


#include "IPointCorrespondence.h"

namespace BRICS_3D {

class PointCorrespondenceKDTree: public BRICS_3D::IPointCorrespondence {
public:
	PointCorrespondenceKDTree();
	virtual ~PointCorrespondenceKDTree();

	void createNearestNeighborCorrespondence(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2, std::vector<CorrespondencePoint3DPair>* resultPointPairs);
};

}

#endif /* POINTCORRESPONDANCEKDTREE_H_ */

/* EOF */
