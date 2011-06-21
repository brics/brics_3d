/*
 * ObjectModelCircle.h
 *
 *  Created on: Apr 22, 2011
 *      Author: reon
 */

#ifndef OBJECTMODELCIRCLE_H_
#define OBJECTMODELCIRCLE_H_

#include "algorithm/segmentation/objectModels/IObjectModel.h"

namespace BRICS_3D {

class ObjectModelCircle : public IObjectModel {

private:
	/** \brief Temporary boost mutex for \a tmp_inliers_ */
//	boost::mutex tmp_mutex_;

	/** \brief Temporary pointer to a list of given indices for optimizeModelCoefficients () */
	const std::vector<int> *tmp_inliers_;

public:
	ObjectModelCircle():IObjectModel(){};
	virtual ~ObjectModelCircle(){};

	void getSamples (int &iterations, std::vector<int> &samples);
	bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients);
	void optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
			Eigen::VectorXd &optimized_coefficients);
	void getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances);
	void selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
			std::vector<int> &inliers);
	void getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,  std::vector<double> &distances);
	void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
			PointCloud3D* projectedPointCloud);
	bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients, double threshold);
	void computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate, bool &modelFound);

	inline int getNumberOfSamplesRequired(){return 2;};
};

}

#endif /* OBJECTMODELCIRCLE_H_ */
