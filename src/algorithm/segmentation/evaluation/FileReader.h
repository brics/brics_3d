/*
 * FileReader.h
 *
 * @author: Pinaki Banerjee
 * @date: Apr 10, 2011
 * @version: 0.1
 * @description
 *	Intended to be used for reading files into vector formats
 */

#ifndef FILEREADER_H_
#define FILEREADER_H_
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;
namespace BRICS_3D{
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
