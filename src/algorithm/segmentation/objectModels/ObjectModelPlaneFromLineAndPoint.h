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

#ifndef OBJECTMODELPLANEFROMLINEANDPOINT_H_
#define OBJECTMODELPLANEFROMLINEANDPOINT_H_

#include "algorithm/segmentation/objectModels/ObjectModelPlane.h"

namespace BRICS_3D {

/**
 * @note The implementation is reusing the object model implementation in MRPT
 * @ingroup segmentation
 */
class ObjectModelPlaneFromLineAndPoint : public ObjectModelPlane{
protected:
	/** Used to check if a number is small enough to be considered to be 0 */
	static const double geometryEpsilon=1e-3;


public:

	ObjectModelPlaneFromLineAndPoint(){};
	virtual ~ObjectModelPlaneFromLineAndPoint(){};

	struct line{
		Point3D pbase;
		std::vector<double> director;
	};

	/** @brief Checks if a line contains a point or not*/

	inline bool contains(const line &line,Point3D &point){
		double dx=point.getX()-line.pbase.getX();
		double dy=point.getY()-line.pbase.getY();
		double dz=point.getZ()-line.pbase.getZ();
		if (abs(dx)<geometryEpsilon&&abs(dy)<geometryEpsilon&&abs(dz)<geometryEpsilon) return true;

		//       dx          dy          dz
		//if -----------=-----------=-----------, point is inside the line.
		//   director[0] director[1] director[2]

		return (abs(dx*line.director[1]-dy*line.director[0])<geometryEpsilon)&&
				(abs(dx*line.director[2]-dz*line.director[0])<geometryEpsilon)&&
				(abs(dy*line.director[2]-dz*line.director[1])<geometryEpsilon);
	}

	/** @brief Computes the cross product of two 3D vectors, returning a vector normal to both.
	 *  It uses the simple implementation:

	    @f[  v_out = \left(
				\begin{array}{c c c}
				\hat{i} ~ \hat{j} ~ \hat{k} \\
				x0 ~ y0 ~ z0 \\
				x1 ~ y1 ~ z1 \\
				\end{array} \right)
		@f]
	 */

	inline void crossProduct3D(
			const std::vector<double> &v0,
			const std::vector<double> &v1,
			std::vector<double> &v_out){
				assert(v0.size()==3);
				assert(v1.size()==3);
		v_out[0] =  v0[1]*v1[2] - v0[2]*v1[1];
		v_out[1] = -v0[0]*v1[2] + v0[2]*v1[0];
		v_out[2] =  v0[0]*v1[1] - v0[1]*v1[0];
	}

	void getSamples (int &iterations, std::vector<int> &samples);
	bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients);

	void computeRandomModel (int &iterations, Eigen::VectorXf &model_coefficients, bool &isDegenerate, bool &modelFound);
	inline int getNumberOfSamplesRequired(){return 3;};


};
}
#endif /* OBJECTMODELPLANEFROMLINEANDPOINT_H_ */
