/**
 * @file 
 * PointCorrespondenceGenericNN.h
 *
 * @date: Jan 6, 2010
 * @author: sblume
 */

#ifndef POINTCORRESPONDENCEGENERICNN_H_
#define POINTCORRESPONDENCEGENERICNN_H_

#include "IPointCorrespondence.h"
#include "algorithm/nearestNeighbor/INearestNeighbor.h"

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Implementation of correspondence problem for points using generic nearest neighbor search
 */
class PointCorrespondenceGenericNN: public BRICS_3D::IPointCorrespondence {
public:

	/**
	 * @brief Standard constructor
	 */
	PointCorrespondenceGenericNN();

	/**
	 * @brief Constructor that defines the nearest neighbor search strategy
	 * @param[in] nearestNeighborAlgorithm The nearest neighbor search strategy that will be used
	 */
	PointCorrespondenceGenericNN(INearestNeighbor* nearestNeighborAlgorithm);

	/**
	 * @brief Standard destructor
	 */
	virtual ~PointCorrespondenceGenericNN();

	void createNearestNeighborCorrespondence(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2,
			std::vector<CorrespondencePoint3DPair>* resultPointPairs);
	/**
	 * @brief Get the nearest neighbor search strategy
	 * @return Returns the nearest neighbor search strategy
	 */
	INearestNeighbor* getNearestNeighborAlgorithm() const;

	/**
	 * @brief Set the nearest neighbor search strategy
	 * @param[in] nearestNeighborAlgorithm The nearest neighbor search strategy that will be used
	 */
	void setNearestNeighborAlgorithm(INearestNeighbor* nearestNeighborAlgorithm);

private:

	/// Internal handle to the nearest neighbor search strategy
	INearestNeighbor* nearestNeighborAlgorithm;
};

}

#endif /* POINTCORRESPONDENCEGENERICNN_H_ */

/* EOF */
