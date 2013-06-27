/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2013, KU Leuven
*
* Author: Sebastian Blumenthal
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

#ifndef COVARIANCEMATRIX66_H_
#define COVARIANCEMATRIX66_H_

#include "ITransformUncertainty.h"
#include "IHomogeneousMatrix44.h" //for compound

#include <Eigen/Geometry>

#ifdef EIGEN3
	#define EIGEN_ALIGN_MEMORY EIGEN_ALIGN16
#else
	#define EIGEN_ALIGN_MEMORY EIGEN_ALIGN_128
	#include <Eigen/LU>
#endif

namespace brics_3d {




class CovarianceMatrix66 : public ITransformUncertainty {
public:

	typedef boost::shared_ptr<CovarianceMatrix66> CovarianceMatrix66Ptr;
	typedef boost::shared_ptr<CovarianceMatrix66 const> CovarianceMatrix66ConstPtr;

	CovarianceMatrix66();
	CovarianceMatrix66(double x,double y, double z, double roll, double pitch, double yaw);
	virtual ~CovarianceMatrix66();


	int getDimension() const;

	int getRowDimension() const;

	int getColumnDimension() const;

	const double* getRawData() const;

	double* setRawData();

	void getVisualizationDimensions(double& x, double& y, double& z);

	friend ostream& operator<<(ostream &outStream, const ITransformUncertainty &uncertainty);

private:
	/// Amount of elements in 6x6 matrix
	const static int matrixElements = 36; // 6x6

	/// Row dimansion
	const static int rowDimension = 6;

	/// Column dimension
	const static int columnDimension = 6;

	/// Array that holds data in column-row (column-major) order
	double matrixData[matrixElements];

	/// Set all values to zero.
	void zeros();
};

extern CovarianceMatrix66::CovarianceMatrix66Ptr compoundCovariance(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean1,
		                                                            CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty1,
		                                                            IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean2,
		                                                            CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty2);

extern CovarianceMatrix66::CovarianceMatrix66Ptr invertCovariance(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean,
		                                                            CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty);

extern bool mergeCovariance(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean1,
		                    CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty1,
		                    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mean2,
		                    CovarianceMatrix66::CovarianceMatrix66Ptr uncertainty2,
		                    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mergedMean,
		                    CovarianceMatrix66::CovarianceMatrix66Ptr mergedUncertainty);

}

#endif /* COVARIANCEMATRIX66_H_ */

/* EOF */
