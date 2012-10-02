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

#ifndef BRICS_3D_EVALUATOR_H_
#define BRICS_3D_EVALUATOR_H_
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
using namespace std;
namespace brics_3d{

class Evaluator {
private:
	std::string gtBaseName;
	std::string msBaseName;
	std::string fileExt;				//store filename details
	int countGt;						//store no of regions in Ground Truth Image
	int countMs;						//stores no of regions in Machine Segmented Image
	std::string regionCorrespondence; 	//store information of GT-MS region correspondence
	int imageSize;
	bool initializationStatus;
public:
	vector<std::string> *vectorGt;	//store GT region points
	vector<std::string> *vectorMs;	//store MS region points
	Evaluator();
	virtual ~Evaluator();

	void setGtBaseName(std::string baseName);
	std::string getGtBaseName();

	void setMsBaseName(std::string baseName);
	std::string getMsBaseName();

	void setFileExt(std::string ext);
	std::string getFileExt();

	void setCountGt(int count);
	int getCountGt();

	void setCountMs(int count);
	int getCountMs();

	void setRegionCorrespondence(std::string regCorres);
	std::string getRegionCorrespondence();

	void setImageSize(int size);
	int getImageSize();

	/**
	 * Initialize the vectors by reading the required GT and MS files
	 */
	void Initialize();
	bool IsInitialized();

};

}

#endif /* BRICS_3D_EVALUATOR_H_ */
