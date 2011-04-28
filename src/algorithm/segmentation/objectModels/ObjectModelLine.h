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

	void computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate,
			bool &modelFound);

		inline int getNumberOfSamplesRequired(){return 2;};
};

}

#endif /* OBJECTMODELLINE_H_ */
