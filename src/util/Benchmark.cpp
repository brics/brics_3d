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

#include "Benchmark.h"

using namespace std;

#include <sstream>
#include <iomanip> 	//for setw and setfill
#include <ctime>
#include <stdexcept>

#ifdef WIN32
	#include <direct.h>
#else
	#include <sys/stat.h> //for mkdir
#endif

namespace BRICS_3D {

std::string Benchmark::timeStamp = ""; // deduced from time-stamp
std::string Benchmark::directoryName = ""; // deduced from time-stamp
#ifdef WIN32
	std::string Benchmark::seperator = "\\"; // one \ is escape character...
#else
	std::string Benchmark::seperator = "/";
#endif

Benchmark::Benchmark() {
	Benchmark("unnamedBenchmark");
}

Benchmark::Benchmark(std::string benchmarkName) {
	this->benchmarkName = benchmarkName;
	setupTargetFile();
}

Benchmark::~Benchmark() {
	output.flush();
	output.close();
}

void Benchmark::setupTargetFile() {
	string pathName = std::string(BRICS_LOGFILES_DIR);

	if (Benchmark::timeStamp.compare("") == 0) { // set only once
		stringstream tmpFileName;

		time_t rawtime;
		struct tm* timeinfo;

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		tmpFileName << "20" // This will have to be adjusted in year 2100 ;-)
				<< (timeinfo->tm_year)-100 << "-"
				<< setw(2) << setfill('0')
				<<	(timeinfo->tm_mon)+1 << "-"
				<< setw(2) << setfill('0')
				<<	timeinfo->tm_mday << "_"
				<< setw(2) << setfill('0')
				<<	timeinfo->tm_hour << "-"
				<< setw(2) << setfill('0')
				<<	timeinfo->tm_min << "-"
				<< setw(2) << setfill('0')
				<<	timeinfo->tm_sec;

//		cout << "tmpFileName " << tmpFileName.str() << endl;
		Benchmark::timeStamp = tmpFileName.str();

		Benchmark::directoryName = pathName +
				Benchmark::seperator +
				Benchmark::timeStamp;

		int err;
#ifdef WIN32 //__MSDOS__
		err = _mkdir (directoryName.c_str());
#else  /*  Unix */
		err = mkdir (Benchmark::directoryName.c_str(), ACCESSPERMS);
#endif
		if (err < 0) {
			string error = "ERROR: cannot create folder" + Benchmark::directoryName;
			throw runtime_error(error.c_str());
		}
	}

	/* concatenate resulting filename string */
	fileName = Benchmark::directoryName + Benchmark::seperator + benchmarkName + ".txt";
	cout << "INFO: Logging benchmark results to: " << fileName << endl;

	output.open(fileName.c_str(), ios::trunc);

}

}

/* EOF */
