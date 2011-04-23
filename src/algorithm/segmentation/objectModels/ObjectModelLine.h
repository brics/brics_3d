/*
 * ObjectModelLine.h
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#ifndef OBJECTMODELLINE_H_
#define OBJECTMODELLINE_H_

#include "algorithm/segmentation/objectModels/IObjectModel.h"

namespace BRICS_3D {

class ObjectModelLine : public IObjectModel {
public:
	ObjectModelLine():IObjectModel(){};
	virtual ~ObjectModelLine(){};

	void getSamples (int &iterations, std::vector<int> &samples);
	bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients);
	void optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
			Eigen::VectorXf &optimized_coefficients);
	void getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances);
	void selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold,
			std::vector<int> &inliers);
	void getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,  std::vector<double> &distances);
	void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
			PointCloud3D* projectedPointCloud);
	bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold);

};

}

#endif /* OBJECTMODELLINE_H_ */
