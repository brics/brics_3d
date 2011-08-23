/*
 * @file: ObjectModelCircle.h
 *
 * @date:Created on: Apr 22, 2011
 * @author:Author: reon
 * @note The implementation is reusing the object model implementation in ROS:PCl
 */
#ifndef OBJECTMODELCIRCLE_H_
#define OBJECTMODELCIRCLE_H_

#include "algorithm/segmentation/objectModels/IObjectModel.h"

namespace BRICS_3D {

class ObjectModelCircle : public IObjectModel {

private:


public:
	ObjectModelCircle():IObjectModel(){};
	virtual ~ObjectModelCircle(){};

	void getSamples (int &iterations, std::vector<int> &samples);
	bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients);
	void getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances);
	void selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
			std::vector<int> &inliers);
	void getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,  std::vector<double> &distances);
	bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients, double threshold);
	void computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate, bool &modelFound);

	inline int getNumberOfSamplesRequired(){return 2;};
};

}

#endif /* OBJECTMODELCIRCLE_H_ */
