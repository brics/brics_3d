/*
 * FileReader.cpp
 *
 * @author: Pinaki Banerjee
 * @date: Apr 10, 2011
 * @version: 0.1
 *
 */

#include "FileReader.h"
#include <sstream>

namespace BRICS_3D{
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
