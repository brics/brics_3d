/*
 * @file: SACMethodMLESAC.h
 *
 * @date: Apr 21, 2011
 * @author: reon
 */

#ifndef SACMETHODMLESAC_H_
#define SACMETHODMLESAC_H_

#include "ISACMethods.h"


namespace BRICS_3D {

/**
 * @brief class implementing the MLESAC method. The code is a re-factored version
 *  of the algorithm implementation in ROS:PCL
 */
//ToDo add reference to the method
class SACMethodMLESAC : public ISACMethods {
private:
	/** @brief Maximum number of Expectation Maximization (EM) iterations. */
	int iterationsEM;

	/** @brief The MLESAC sigma parameter. */
	double sigma_;

protected:

	/** @brief Compute the median value of a 3D point cloud
	 *  @param cloud the point cloud data message
	 *  @param median the resultant median value
	 */
	void
	computeMedian (PointCloud3D *cloud, Eigen::Vector4f &median);


	/** @brief Determine the minimum and maximum 3D bounding box coordinates
	 *  @param cloud the point cloud
	 *  @param minP the resultant minimum bounding box coordinates
	 *  @param maxP the resultant maximum bounding box coordinates
	 */
	void
	getMinMax (PointCloud3D *cloud, Eigen::Vector4d &minP, Eigen::Vector4d &maxP);


	/** @brief Compute the median absolute deviation:
	 * @f[
	 * MAD = \sigma * median_i (| Xi - median_j(Xj) |)
	 * @f]
	 * @note Sigma needs to be chosen carefully (a good starting sigma value is 1.4826)
	 * @param cloud the point cloud data message
	 * @param sigma the sigma value
	 */
	double
	computeMedianAbsoluteDeviation (PointCloud3D *cloud, double sigma);


public:

	SACMethodMLESAC();

	bool computeModel();

	virtual ~SACMethodMLESAC();


	/** @brief Set the number of EM iterations.
	 *  @param iterations the number of EM iterations
	 */
	inline void setEMIterations (int iterations) { this->iterationsEM = iterations; }


	/** @brief Get the number of EM iterations. */
	inline int getEMIterations () { return (iterationsEM); }

};

}

#endif /* SACMETHODMLESAC_H_ */
