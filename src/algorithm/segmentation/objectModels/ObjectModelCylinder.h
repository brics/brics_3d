/*
 * ObjectModelCylinder.h
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#ifndef OBJECTMODELCYLINDER_H_
#define OBJECTMODELCYLINDER_H_

#include "algorithm/segmentation/objectModels/IObjectModelUsingNormals.h"

namespace BRICS_3D {

class ObjectModelCylinder : public IObjectModelUsingNormals {

public:
	ObjectModelCylinder(){};
	virtual ~ObjectModelCylinder(){};

	void getSamples (int &iterations, std::vector<int> &samples);
	bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients);
	void optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
			Eigen::VectorXf &optimized_coefficients);
	void getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances);
	void selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold,
			std::vector<int> &inliers);
	void getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
			std::vector<double> &distances);
	void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
			PointCloud3D* projectedPointCloud);
	bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients,
			double threshold);
	void computeRandomModel (int &iterations, Eigen::VectorXf &model_coefficients, bool &isDegenerate, bool &modelFound);

	inline int getNumberOfSamplesRequired(){return 3;};

protected:
	/** \brief Get the distance from a point to a line (represented by a point and a direction)
	 * \param pt a point
	 * \param model_coefficients the line coefficients (a point on the line, line direction)
	 */
	double
	pointToLineDistance (const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients);

	/** \brief Get the distance from a point to a line (represented by a point and a direction)
	 * \param pt a point
	 * \param line_pt a point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
	 * \param line_dir the line direction
	 */
	double
	pointToLineDistance (const Eigen::Vector4f &pt, const Eigen::Vector4f &line_pt,
			const Eigen::Vector4f &line_dir);

	/** \brief Project a point onto a line given by a point and a direction vector
	 * \param pt the input point to project
	 * \param line_pt the point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
	 * \param line_dir the direction of the line (make sure that line_dir[3] = 0 as there are no internal checks!)
	 * \param pt_proj the resultant projected point
	 */
	inline void
	projectPointToLine (const Eigen::Vector4f &pt, const Eigen::Vector4f &line_pt, const Eigen::Vector4f &line_dir,
			Eigen::Vector4f &pt_proj)
	{
		double k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);
		// Calculate the projection of the point on the line
		pt_proj = line_pt + k * line_dir;
	}

	/** \brief Project a point onto a cylinder given by its model coefficients (point_on_axis, axis_direction,
	 * cylinder_radius_R)
	 * \param pt the input point to project
	 * \param model_coefficients the coefficients of the cylinder (point_on_axis, axis_direction, cylinder_radius_R)
	 * \param pt_proj the resultant projected point
	 */
	inline void
	projectPointToCylinder(const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients,
			Eigen::Vector4f &pt_proj)
	{
		Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
		Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

		double k = (pt.dot (line_dir) - line_pt.dot (line_dir)) * line_dir.dot (line_dir);
		pt_proj = line_pt + k * line_dir;

		Eigen::Vector4f dir = pt - pt_proj;
		dir.normalize ();

		// Calculate the projection of the point onto the cylinder
		pt_proj += dir * model_coefficients[6];
	}


};

}

#endif /* OBJECTMODELCYLINDER_H_ */
