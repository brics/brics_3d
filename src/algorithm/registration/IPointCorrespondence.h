/**
 * @file 
 * IPointCorrespondence.h
 *
 * @date: Dec 3, 2009
 * @author: sblume
 */

#ifndef IPOINTCORRESPONDENCE_H_
#define IPOINTCORRESPONDENCE_H_

#include "core/PointCloud3D.h"
//#include "core/CorrespondenceIndexPair.h"
#include "core/CorrespondencePoint3DPair.h"

#include <vector>

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Abstract interface for algorithms that solve the correspondence problem for points
 */
class IPointCorrespondence {
public:

	/**
	 * @brief Standard constructor
	 */
	IPointCorrespondence(){};

	/**
	 * @brief Standard destructor
	 */
	virtual ~IPointCorrespondence(){};

//	/**
//	 * @brief Establishes a point to point correspondence with nearest neighborhood search
//	 * @param[in] pointCloud1 Pointer to first point cloud
//	 * @param[in] pointCloud2 Pointer to second point cloud
//	 * @param[out] resultPointPairs Pointer where to store the resulting point correspondences. If no correspondences are found, this vector will be empty
//	 */
//	virtual void createNearestNeighborCorrespondence(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2, std::vector<CorrespondenceIndexPair>* resultPointPairs) = 0;

	/**
	 * @brief Establishes a point to point correspondence with nearest neighborhood search
	 * @param[in] pointCloud1 Pointer to first point cloud
	 * @param[in] pointCloud2 Pointer to second point cloud
	 * @param[out] resultPointPairs Pointer where to store the resulting point correspondences. If no correspondences are found, this vector will be empty
	 */
	virtual void createNearestNeighborCorrespondence(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2, std::vector<CorrespondencePoint3DPair>* resultPointPairs) = 0;

};

}

#endif /* IPOINTCORRESPONDENCE_H_ */

/* EOF */
