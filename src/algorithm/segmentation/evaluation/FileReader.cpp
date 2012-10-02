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

#include "FileReader.h"
#include <sstream>

namespace brics_3d{
FileReader::FileReader() {}

FileReader::~FileReader() {}

int FileReader::toStringVector(string fileName, vector<string> &vector){
	ifstream file;
	string s;
	file.open(fileName.c_str(),ifstream::in);
			if(file.is_open()){
				while(getline(file, s)){
					vector.push_back(s);
				}
				file.close();
				return 1;
			} else {
				cout<< fileName <<" --- > File Not found" <<endl;
				return 0;
			}
}

}
