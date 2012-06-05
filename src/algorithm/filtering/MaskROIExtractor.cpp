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

#include "MaskROIExtractor.h"
#include "core/Logger.h"

namespace BRICS_3D {

MaskROIExtractor::MaskROIExtractor() {
	mask = 0;
	useInvertedMask = false;
}

MaskROIExtractor::~MaskROIExtractor() {
	if (mask) {
		//delete mask;
		mask = 0;
	}
}

void MaskROIExtractor::filter(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud) {
	assert (mask != 0);
	if(useInvertedMask) {
		extractNonIndexedPointCloud(originalPointCloud, *mask, resultPointCloud);
	} else {
		extractIndexedPointCloud(originalPointCloud, *mask, resultPointCloud);
	}
}

void MaskROIExtractor::extractIndexedPointCloud(BRICS_3D::PointCloud3D* inputPoinCloud, std::vector<int> inliers, BRICS_3D::PointCloud3D* outputPointCloud) {
	assert(inputPoinCloud !=0);
	assert(outputPointCloud !=0);

	(*outputPointCloud->getPointCloud()).resize(inliers.size());
	//copy over inliers
	for (int i = 0; i < static_cast<int>(inliers.size()); ++i) {
		(*outputPointCloud->getPointCloud())[i] = (*inputPoinCloud->getPointCloud())[inliers[i]];
	}
}

void MaskROIExtractor::extractNonIndexedPointCloud(BRICS_3D::PointCloud3D* inputPoinCloud, std::vector<int> inliers, BRICS_3D::PointCloud3D* outputPointCloud) {
	assert(inputPoinCloud !=0);
	assert(outputPointCloud !=0);
	std::vector<int> invertedInliers;

	int indliersIndex = 0;
	int invertedIndliersIndex = 0;
	std::sort(inliers.begin(), inliers.end());
	invertedInliers.resize(inputPoinCloud->getSize()-inliers.size());
	invertedInliers[invertedInliers.size()-1] = -1;
	for (int i = 0; i < static_cast<int>(inputPoinCloud->getSize()); ++i) { // run ofer all input data and skip those that belong to inliers
		if (i == inliers[indliersIndex]) {
			indliersIndex++;
			continue; // skip insertion
		}
		invertedInliers[invertedIndliersIndex] = i;
		invertedIndliersIndex++;
	}
	assert( invertedInliers[invertedInliers.size()-1] > 0);
//		assert(invertedInliers.size() == inputPoinCloud->getSize());


//		for (int i = 0; i < static_cast<int>(inliers.size()); ++i) {
//			invertedInliers.erase(invertedInliers.begin() + inliers[i]);
//		}
	LOG(DEBUG) << "inliers.size(), invertedInliers.size(), inputPoinCloud->getSize()" << inliers.size() << ", " << invertedInliers.size() << ", " << inputPoinCloud->getSize();
	assert(inliers.size() + invertedInliers.size() == inputPoinCloud->getSize());
	extractIndexedPointCloud(inputPoinCloud, invertedInliers, outputPointCloud);
}

void MaskROIExtractor::setMask(std::vector<int>* mask) {
	assert(mask != 0);
	this->mask = mask;
}

void MaskROIExtractor::setUseInvertedMask(bool useInvertedMask) {
	this->useInvertedMask = useInvertedMask;
}

bool MaskROIExtractor::getUseInvertedMask() {
	return this->useInvertedMask;
}

}

/* EOF */
