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

#include "FileWriter.h"
#include "MetricCalculator.h"
namespace BRICS_3D{

FileWriter::FileWriter() {}

FileWriter::~FileWriter() {}

void FileWriter::WriteToHTML(MetricCalculator metricCalculatorObject,string prefix){
	Evaluator evaluatorObject = metricCalculatorObject.getEvaluatorObject();
	int countMs = evaluatorObject.getCountMs();
	int countGt = evaluatorObject.getCountGt();
	double** resultOverlap = metricCalculatorObject.getResultOverlap();
	double** result_gt = metricCalculatorObject.getResultGtMetrics();
	double** result_ms = metricCalculatorObject.getResultMsMetrics();
	stringstream filename;

	filename.str("");
	filename.clear();
	filename << "../src/algorithm/segmentation/evaluation/data/" << prefix <<".html";
	ofstream result_stream;
	result_stream.precision(5);
	result_stream.open(filename.str().c_str(), ios::out);
	cout<< filename.str() <<"\n Wrting Success(0/1): "<<result_stream.is_open()<<endl;
	result_stream.clear();
	result_stream << "<b>Overlap Confusion Matrix:</b><p style=\"clear:both\"> <i>Every cell gives the percentage of overlap between the intersecting Ground Truth and Machine Segmented Regions</i></p>" <<endl;
	//Overlap Confusion Matrix
	result_stream << "<table border=\"1\" align = center>" <<endl;
	result_stream << "<tr>"<<endl;
	result_stream << "<td>" << " "  << "</td>" <<endl;
	for(int j =0; j<(countMs); j++) {
		result_stream << "<td><b>" << "MS_Region" << j+1 << "</b></td>" <<endl;
	}
	result_stream << "<td><b>" << "Unlabelled" << "</b></td>" <<endl;
	result_stream << "</tr>"<<endl;

	for (int i =0; i<(countGt+1); i++) {
		result_stream << "<tr>"<<endl;
		if ( i < countGt){
			result_stream << "<td><b>" << "GT_Region" << i+1 << "</b></td>" <<endl;
		} else {
			result_stream << "<td><b>" << "Unlabelled" << "</b></td>" <<endl;
		}

		int total_points=0;

		for (int x=0; x<countMs+1; x++){
			total_points += resultOverlap[i][x];
		}

		double percent=0.0,temp_a;


		for(int j =0; j<(countMs+1); j++) {
			temp_a = resultOverlap[i][j];
			percent = temp_a/total_points;
			result_stream << "<td>" << fixed << percent <<"</td>" <<endl;
		}

		result_stream << "</tr>"<<endl;
	}

	result_stream << "</table>" <<endl;


	//Writing GT-region based evaluation metrics
	result_stream << "<BR>&nbsp;<BR><BR>&nbsp;<BR><BR>&nbsp;";
	result_stream << "<b>GT Region based metrics:</b>";
	result_stream <<"<p style=\"clear:both\"> <i>";
	result_stream <<"Every cell gives the value of the evaluation metric for the corresponding GT Region. </i></p>" <<endl;
	result_stream << "<table border=\"1\"  align = center>" <<endl;

	result_stream << "<tr>"<<endl;
	result_stream << "<td>" << " "  << "</td>" <<endl;
	result_stream << "<td><b>" << "Occupancy of the region (%)" << "</b></td>" <<endl;
	result_stream << "<td><b>" << "Correct detection (%)" << "</b></td>" <<endl;
	result_stream << "<td><b>" << "No of Oversegmented Regions" << "</b></td>" <<endl;
	result_stream << "<td><b>" << "Oversegmentation (%)" << "</b></td>" <<endl;
	result_stream << "<td><b>" << "Non-Classification (%)" << "</b></td>" <<endl;
	result_stream << "</tr>"<<endl;

	for (int i =0; i<(countGt); i++) {

		result_stream << "<tr>"<<endl;
		result_stream << "<td><b>" << "GT_Region" << i+1 << "</b></td>" <<endl;

		for(int j =0; j<metricCalculatorObject.getNoOfMetricsGt(); j++) {
			result_stream << "<td>" <<fixed<<result_gt[i][j]<<"</td>" <<endl;
		}

		result_stream << "</tr>"<<endl;
	}


	result_stream << "</table>"<<endl;
	//Writing MS region based metrics
	result_stream << "<BR>&nbsp;<BR><BR>&nbsp;<BR><BR>&nbsp;";
	result_stream <<"<b>MS Region based metrics:</b>";
	result_stream <<"<p style=\"clear:both\"> <i>";
	result_stream <<"Every cell gives the value of the evaluation metric for the corresponding MS Region. </i></p>" <<endl;

	result_stream << "<table border=\"1\" align = center >" <<endl;

	result_stream << "<tr>"<<endl;
	result_stream << "<td>" << " "  << "</td>" <<endl;
	result_stream << "<td><b>" << "Occupancy of the region (%)" << "</b></td>" <<endl;
	result_stream << "<td><b>" << "Wrong Classification (%)" << "</b></td>" <<endl;
	result_stream << "<td><b>" << "No of Undersegmented Regions" << "</b></td>" <<endl;
	result_stream << "<td><b>" << "Undersegmentation (%)" << "</b></td>" <<endl;
	result_stream << "<td><b>" << "Noise-Classification (%)" << "</b></td>" <<endl;
	result_stream << "</tr>"<<endl;

	for (int i =0; i<(countMs); i++) {

		result_stream << "<tr>"<<endl;
		result_stream << "<td><b>" << "MS_Region" << i+1 << "</b></td>" <<endl;

		for(int j =0; j<metricCalculatorObject.getNoOfMetricsMs(); j++) {
			result_stream << "<td>" <<fixed<<result_ms[i][j]<<"</td>" <<endl;
		}

		result_stream << "</tr>"<<endl;
	}

	result_stream << " </table> <BR>&nbsp;<BR><BR>&nbsp;<BR><BR>&nbsp;"<<
			"For metric definitions see <b>\"EvaluationMetrics.pdf\"</b> in\""<<
			"\"trunk/src/evaluation\"";
	result_stream.close();
}

void FileWriter::WriteToCsv(MetricCalculator metricCalculatorObject,string prefix){
	stringstream filename;
	Evaluator evaluatorObject = metricCalculatorObject.getEvaluatorObject();
	int countMs = evaluatorObject.getCountMs();
	int countGt = evaluatorObject.getCountGt();
	double** resultOverlap = metricCalculatorObject.getResultOverlap();
	double** resultGt = metricCalculatorObject.getResultGtMetrics();
	double** resultMs = metricCalculatorObject.getResultMsMetrics();
	ofstream result_stream;

	//writing overlap matrix
	filename.str("");
	filename.clear();
	filename<< "../src/algorithm/segmentation/evaluation/data/"<<prefix<<"_overlapMatrix.csv";
	result_stream.precision(5);
	result_stream.open(filename.str().c_str(), ios::out);
	cout<< filename.str() <<"\n Wrting Success(0/1): "<<result_stream.is_open()<<endl;
	result_stream.clear();

	for(int i=0; i<countGt; i++){
		result_stream << endl;
		for(int j =0; j<countMs; j++){
			if(j!=countMs-1) {
				result_stream<<fixed<<resultOverlap[i][j]<<",";
			}else{
				result_stream<<fixed<<resultOverlap[i][j];
			}
		}
	}

	result_stream.close();

	//Writing GT based metrics
	filename.str("");
	filename.clear();
	filename<< "../src/algorithm/segmentation/evaluation/data/"<<prefix<<"_GtMetrics.csv";
	result_stream.precision(5);
	result_stream.open(filename.str().c_str(), ios::out);
	cout<< filename.str() <<"\n Wrting Success(0/1): "<<result_stream.is_open()<<endl;
	result_stream.clear();

	for(int i=0; i<countGt; i++){

		for(int j =0; j<metricCalculatorObject.getNoOfMetricsGt(); j++){
			if(j!=metricCalculatorObject.getNoOfMetricsGt()-1) {
				result_stream<<fixed<<resultGt[i][j]<<",";
			}else{
				result_stream<<fixed<<resultGt[i][j];
			}
		}
		result_stream << endl;
	}

	result_stream.close();

	//Writing GT based metrics
	filename.str("");
	filename.clear();
	filename<< "../src/algorithm/segmentation/evaluation/data/"<<prefix<<"_MsMetrics.csv";
	result_stream.precision(5);
	result_stream.open(filename.str().c_str(), ios::out);
	cout<< filename.str() <<"\n Wrting Success(0/1): "<<result_stream.is_open()<<endl;
	result_stream.clear();

	for(int i=0; i<countMs; i++){

		for(int j =0; j<metricCalculatorObject.getNoOfMetricsMs(); j++){
			if(j!=metricCalculatorObject.getNoOfMetricsMs()-1) {
				result_stream<<fixed<<resultMs[i][j]<<",";
			}else{
				result_stream<<fixed<<resultMs[i][j];
			}
		}
		result_stream << endl;
	}

	result_stream.close();


}

}
