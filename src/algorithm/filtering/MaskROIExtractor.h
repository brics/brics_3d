/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#ifndef MASKROIEXTRACTOR_H_
#define MASKROIEXTRACTOR_H_

#include "IFiltering.h"

namespace BRICS_3D {

class MaskROIExtractor : public IFiltering {
public:
	MaskROIExtractor();
	virtual ~MaskROIExtractor();

	void filter(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud);

	void extractIndexedPointCloud(BRICS_3D::PointCloud3D* inputPoinCloud, std::vector<int> inliers, BRICS_3D::PointCloud3D* outputPointCloud);

	void extractNonIndexedPointCloud(BRICS_3D::PointCloud3D* inputPoinCloud, std::vector<int> inliers, BRICS_3D::PointCloud3D* outputPointCloud);

	void setMask(std::vector<int>* mask);

	void setUseInvertedMask(bool useInvertedMask);

	bool getUseInvertedMask();

private:

	std::vector<int>* mask;

	bool useInvertedMask;

};

}

#endif /* MASKROIEXTRACTOR_H_ */

/* EOF */
