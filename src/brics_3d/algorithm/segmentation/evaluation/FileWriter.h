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

#ifndef BRICS_3D_FILEWRITER_H_
#define BRICS_3D_FILEWRITER_H_
#include "MetricCalculator.h"

namespace brics_3d{
class FileWriter {
public:
	FileWriter();
	virtual ~FileWriter();
	/*
	 * Write the results in Metric Calculator object into a predefined
	 * HTML format.
	 *@param  "prefix" : the prefix to be used for the filename
	 */
	void WriteToHTML(MetricCalculator m,string prefix);

	/**
	 * Write the results in Metric Calculator object into a predefined
	 * CSV formats. These files will be used at the time of comparing the algorithms
	 *@param  "prefix" : the prefix to be used for the filename
	 */
	void WriteToCsv(MetricCalculator m,string prefix);
};
}
#endif /* BRICS_3D_FILEWRITER_H_ */
