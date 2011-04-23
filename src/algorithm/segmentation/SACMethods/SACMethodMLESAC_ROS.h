/*
 * SACMethodMLESAC_ROS.h
 *
 *  Created on: Apr 21, 2011
 *      Author: reon
 */

#ifndef SACMETHODMLESAC_ROS_H_
#define SACMETHODMLESAC_ROS_H_

#include "ISACMethods.h"


namespace BRICS_3D {

class SACMethodMLESAC_ROS : public ISACMethods {
private:
	/** \brief Maximum number of EM (Expectation Maximization) iterations. */
	int iterationsEM;

	/** \brief The MLESAC sigma parameter. */
	double sigma_;

protected:

	/** \brief Compute the median value of a 3D point cloud using a given set point indices and return it as a Point32.
	 * \param cloud the point cloud data message
	 * \param median the resultant median value
	 */
	void
	computeMedian (PointCloud3D *cloud, Eigen::Vector4f &median);


	/** \brief Determine the minimum and maximum 3D bounding box coordinates for a given set of points
	 * \param cloud the point cloud message
	 * \param min_p the resultant minimum bounding box coordinates
	 * \param max_p the resultant maximum bounding box coordinates
	 */
	void
	getMinMax (PointCloud3D *cloud, Eigen::Vector4f &min_p, Eigen::Vector4f &max_p);


	/** \brief Compute the median absolute deviation:
	 * \f[
	 * MAD = \sigma * median_i (| Xi - median_j(Xj) |)
	 * \f]
	 * \note Sigma needs to be chosen carefully (a good starting sigma value is 1.4826)
	 * \param cloud the point cloud data message
	 * \param indices the set of point indices to use
	 * \param sigma the sigma value
	 */
	double
	computeMedianAbsoluteDeviation (PointCloud3D *cloud, double sigma);


public:

	SACMethodMLESAC_ROS();

	bool computeModel();

	virtual ~SACMethodMLESAC_ROS();


	/** \brief Set the number of EM iterations.
	 * \param iterations the number of EM iterations
	 */
	inline void setEMIterations (int iterations) { this->iterationsEM = iterations; }


	/** \brief Get the number of EM iterations. */
	inline int getEMIterations () { return (iterationsEM); }

};

}

#endif /* SACMETHODMLESAC_ROS_H_ */
