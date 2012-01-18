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

#ifndef RGBCOLORBASEDEUCLIDEANCLUSTERING_H_
#define RGBCOLORBASEDEUCLIDEANCLUSTERING_H_

#include "algorithm/segmentation/ISegmentation.h"
#include <vector>


#include <iostream>

namespace BRICS_3D {

/**
 * @brief Segmentation based on Eucledian distance and distance in RGB color-space.
 * @ingroup segmentation
 */
class RGBColorBasedEuclideanClustering : public ISegmentation{

private:

	bool isSimilar(uint32_t rgb_24bit_first, uint32_t rgb_24bit_second){

		int rgb_first[3], rgb_second[3];

		rgb_first[0] = ((rgb_24bit_first >> 16) & 0xff);
		rgb_first[1] = ((rgb_24bit_first >> 8) & 0xff);
		rgb_first[2] = (rgb_24bit_first & 0xff);

		rgb_second[0] = ((rgb_24bit_second >> 16) & 0xff);
		rgb_second[1] = ((rgb_24bit_second >> 8) & 0xff);
		rgb_second[2] = (rgb_24bit_second & 0xff);

		double currentDistanceRGBSpace = 	std::sqrt((rgb_second[0]-rgb_first[0])*(rgb_second[0]-rgb_first[0]) +
				(rgb_second[1]-rgb_first[1])*(rgb_second[1]-rgb_first[1]) +
				(rgb_second[2]-rgb_first[2])*(rgb_second[2]-rgb_first[2]));

		if(currentDistanceRGBSpace <= toleranceRGBSpace){
			return true;
		} else {
			return false;
		}

	}


	double toleranceRGBSpace;
	double toleranceEuclideanDistance;
	unsigned int minClusterSize;
	unsigned int maxClusterSize;

	std::vector<BRICS_3D::ColoredPointCloud3D*> extractedClusters;

public:
	RGBColorBasedEuclideanClustering();
	virtual ~RGBColorBasedEuclideanClustering();
	void extractClusters(BRICS_3D::ColoredPointCloud3D *inCloud);
	int segment();

    int getMaxClusterSize() const;
    int getMinClusterSize() const;
    double getToleranceEuclideanDistance() const;
    double getToleranceRgbSpace() const;
    void setMaxClusterSize(int maxClusterSize);
    void setMinClusterSize(int minClusterSize);
    void setToleranceEuclideanDistance(double toleranceEuclideanDistance);
    void setToleranceRgbSpace(double toleranceRgbSpace);

	void getExtractedClusters(std::vector<BRICS_3D::ColoredPointCloud3D*> &extractedClusters){
		extractedClusters = this->extractedClusters;
	}

};

} /* namespace BRICS_3D */
#endif /* RGBCOLORBASEDEUCLIDEANCLUSTERING_H_ */
