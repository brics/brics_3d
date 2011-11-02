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

#ifndef COLORBASEDROIEXTRACTORHSV_H_
#define COLORBASEDROIEXTRACTORHSV_H_

#include "IColorBasedROIExtractor.h"

namespace BRICS_3D {

class ColorBasedROIExtractorHSV : public IColorBasedROIExtractor {

private:
	/**
	 * Maximum and minimum values for Hue
	 */
	double maxH;
	double minH;

	/**
	 * Maximum and minimum values for Saturation
	 */
	double maxS;
	double minS;

	/**
	 * Maximum and minimum values for Lightness(V)
	 */
	double maxV;
	double minV;



public:


	ColorBasedROIExtractorHSV();


	virtual ~ColorBasedROIExtractorHSV();


	/**
	 * Extracts subset of input point cloud based on color-properties
	 * @param in_cloud Input pointcloud (Colored)
	 * @param out_cloud Extracted Subset (Color Information Discarded)
	 */
	void extractColorBasedROI(BRICS_3D::ColoredPointCloud3D *in_cloud, BRICS_3D::ColoredPointCloud3D *out_cloud);


	/**
	 * Extracts subset of input point cloud based on color-properties
	 * @param in_cloud Input pointcloud (Colored)
	 * @param out_cloud Extracted Subset (Color Information Discarded)
	 */
	void extractColorBasedROI(BRICS_3D::ColoredPointCloud3D *in_cloud,
			BRICS_3D::PointCloud3D *out_cloud);

	/**
	 *
	 * @return maximum Hue allowed
	 */
    double getMaxH() const;


    /**
     *
     * @return maximum Saturation allowed
     */
    double getMaxS() const;


    /**
     *
     * @return maximum Lightness allowed
     */
    double getMaxV() const;


    /**
     *
     * @return minimum Hue allowed
     */
    double getMinH() const;


    /**
     *
     * @return minimum Saturation allowed
     */
    double getMinS() const;


    /**
     *
     * @return minimum Lightness allowed
     */
    double getMinV() const;


    /**
     *
     * @param maxH maximum Hue allowed
     */
    void setMaxH(double maxH);


    /**
     *
     * @param maxS maximum Saturation allowed
     */
    void setMaxS(double maxS);


    /**
     *
     * @param maxV maximum Lightness allowed
     */
    void setMaxV(double maxV);


    /**
     *
     * @param minH minimum Hue allowed
     */
    void setMinH(double minH);


    /**
     *
     * @param minS minimum Saturation allowed
     */
    void setMinS(double minS);


    /**
     *
     * @param minV minimum Lightness allowed
     */
    void setMinV(double minV);



};

}

#endif /* COLORBASEDROIEXTRACTORHSV_H_ */
