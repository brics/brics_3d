/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#ifndef BRICS_3D_OBJECTMODELCYLINDER_H_
#define BRICS_3D_OBJECTMODELCYLINDER_H_

#include "brics_3d/algorithm/segmentation/objectModels/IObjectModelUsingNormals.h"

namespace brics_3d {

/**
 * @note The implementation is reusing the object model implementation in ROS:PCl
 * @ingroup segmentation
 */
class ObjectModelCylinder : public IObjectModelUsingNormals {

public:
	ObjectModelCylinder(){};
	virtual ~ObjectModelCylinder(){};

	void getSamples (int &iterations, std::vector<int> &samples);
	bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients);
	void optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
			Eigen::VectorXd &optimized_coefficients);
	void getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances);
	void selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
			std::vector<int> &inliers);
	void getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
			std::vector<double> &distances);
	void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
			PointCloud3D* projectedPointCloud);
	bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients,
			double threshold);
	void computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate, bool &modelFound);

	inline int getNumberOfSamplesRequired(){return 3;};

protected:
	/** @brief Get the distance from a point to a line (represented by a point and a direction)
	 *  @param pt a point
	 *  @param model_coefficients the line coefficients (a point on the line, line direction)
	 */
	double
	pointToLineDistance (const Eigen::Vector4d &pt, const Eigen::VectorXd &model_coefficients);

	/** @brief Get the distance from a point to a line (represented by a point and a direction)
	 * @param pt a point
	 * @param line_pt a point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
	 * @param line_dir the line direction
	 */
	double
	pointToLineDistance (const Eigen::Vector4d &pt, const Eigen::Vector4d &line_pt,
			const Eigen::Vector4d &line_dir);

	/** @brief Project a point onto a line given by a point and a direction vector
	 * @param pt the input point to project
	 * @param line_pt the point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
	 * @param line_dir the direction of the line (make sure that line_dir[3] = 0 as there are no internal checks!)
	 * @param pt_proj the resultant projected point
	 */
	inline void
	projectPointToLine (const Eigen::Vector4d &pt, const Eigen::Vector4d &line_pt, const Eigen::Vector4d &line_dir,
			Eigen::Vector4d &pt_proj)
	{
		double k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);
		// Calculate the projection of the point on the line
		pt_proj = line_pt + k * line_dir;
	}

	/** @brief Project a point onto a cylinder given by its model coefficients (point_on_axis, axis_direction,
	 * cylinder_radius_R)
	 * @param pt the input point to project
	 * @param model_coefficients the coefficients of the cylinder (point_on_axis, axis_direction, cylinder_radius_R)
	 * @param pt_proj the resultant projected point
	 */
	inline void
	projectPointToCylinder(const Eigen::Vector4d &pt, const Eigen::VectorXd &model_coefficients,
			Eigen::Vector4d &pt_proj)
	{
		Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
		Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

		double k = (pt.dot (line_dir) - line_pt.dot (line_dir)) * line_dir.dot (line_dir);
		pt_proj = line_pt + k * line_dir;

		Eigen::Vector4d dir = pt - pt_proj;
		dir.normalize ();

		// Calculate the projection of the point onto the cylinder
		pt_proj += dir * model_coefficients[6];
	}


};

}

#endif /* BRICS_3D_OBJECTMODELCYLINDER_H_ */
