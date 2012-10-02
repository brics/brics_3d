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

#ifndef FILEREADER_H_
#define FILEREADER_H_
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;
namespace brics_3d{
class FileReader {
public:
	FileReader();
	virtual ~FileReader();

	/**
	 * Reads a file and stores each line to the vector pointed by the param "vector"
	 *
	 */
	int toStringVector(string fileName, vector<string> &vector);
};
}
#endif /* FILEREADER_H_ */
