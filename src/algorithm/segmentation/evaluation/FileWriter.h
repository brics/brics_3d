/*
 * FileWriter.h
  *
 * @author: Pinaki Banerjee
 * @date: Apr 10, 2011
 * @version: 0.1
 */

#ifndef FILEWRITER_H_
#define FILEWRITER_H_
#include "MetricCalculator.h"

namespace BRICS_3D{
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
#endif /* FILEWRITER_H_ */
