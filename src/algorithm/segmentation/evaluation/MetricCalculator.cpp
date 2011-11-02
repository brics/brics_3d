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

#include "MetricCalculator.h"
#include <boost/algorithm/string.hpp>
namespace BRICS_3D{
MetricCalculator::MetricCalculator() {}

double **MetricCalculator::getResultGtMetrics() const
{
	return resultGtMetrics;
}

double **MetricCalculator::getResultMsMetrics() const
{
	return resultMsMetrics;
}

double **MetricCalculator::getResultOverlap() const
{
	return resultOverlap;
}

void MetricCalculator::setEvaluatorObject(Evaluator evalObject){
	evaluatorObject=evalObject;
}

int MetricCalculator::calculateMetrics(){

	if(evaluatorObject.IsInitialized() && evaluatorObject.getImageSize()>0){
		//setting array size of overlap result
		resultOverlap = new double*[(evaluatorObject.getCountGt()+2)];
		for(int i = 0; i < evaluatorObject.getCountGt()+2; i++)
			resultOverlap[i] = new double[(evaluatorObject.getCountMs()+2)];

		//setting array size for GT metrics result
		resultGtMetrics = new double*[evaluatorObject.getCountGt()];
		for(int i = 0; i < evaluatorObject.getCountGt(); i++)
			resultGtMetrics[i] = new double[noOfMetricsGt];
		//setting array size for MS metrics result
		resultMsMetrics = new double*[evaluatorObject.getCountMs()];
		for(int i = 0; i < evaluatorObject.getCountMs(); i++)
			resultMsMetrics[i] = new double[noOfMetricsMs];

		for( int i =0; i< evaluatorObject.getCountGt()+2; i++){
			for(int j =0; j<evaluatorObject.getCountMs()+2; j++){
				resultOverlap[i][j]=0;
			}
		}

		for (int i =0; i<evaluatorObject.getCountGt(); i++){
			for (int j =0; j<noOfMetricsGt; j++){
				resultGtMetrics[i][j]=0;
			}
		}


		for (int i =0; i<evaluatorObject.getCountMs(); i++){
			for (int j =0; j<noOfMetricsMs; j++){
				resultMsMetrics[i][j]=0;
			}
		}

		//Calculating the results
		cout << "[Check Point]: " << "Calculating Overlap Matrix" << endl;
		calculateOverlap();
		cout << "[Check Point]: " << "Calculating Ground Truth Region Based Metrics" << endl;
		calculateGtMetrics();
		cout << "[Check Point]: " << "Calculating Machine Segmented Region Based Metrics" << endl;
		calculateMsMetrics();
		cout << "[Check Point]: " << "Successfully Calculated the Metrics :)" << endl;
		return 1;
	} else {
		cout << "[ERROR]" << "Evaluator Not Initialized Properly :(" << endl;
		return 0;
	}

}


void MetricCalculator::calculateOverlap(){

	vector<string> vec_comparisons;		//stores the performed comparisons

	if(evaluatorObject.getRegionCorrespondence()!="") {
		vector<string> vecCorrespondence; 	//set of correspondences like GT1:MS2,MS3,MS4
		vector<string> vecRegionCorrespondence; // Each GT and MS regions seperated
		vector<string> vec_ms_regions;		//set of MS regions for each GT region

		vector<string>::iterator it_correspondence;
		vector<string>::iterator it_region_correspondence;
		vector<string>::iterator it_ms_regions;
		int gt_index;
		int ms_index;
		string regCorres = evaluatorObject.getRegionCorrespondence();

		boost::split(vecCorrespondence, regCorres, boost::is_any_of(";"));

		//Evaluation for each correspondence

		it_correspondence=vecCorrespondence.begin();
		while(it_correspondence != vecCorrespondence.end()){

			//getting each gt-ms region correspondence
			boost::split(vecRegionCorrespondence, *it_correspondence, boost::is_any_of(":"));
			it_region_correspondence = vecRegionCorrespondence.begin();

			//getting the ms regions for each gt-region
			boost::split(vec_ms_regions, *(it_region_correspondence+1), boost::is_any_of(","));
			it_ms_regions = vec_ms_regions.begin();

			//Coparing each GT-MS region pair
			while (it_ms_regions != vec_ms_regions.end()) {
				string temp;
				temp = *it_region_correspondence;
				gt_index = atoi(temp.c_str());
				temp = *it_ms_regions;
				ms_index = atoi(temp.c_str());

				//Actual Region Comparison
				int count=0;
		/**/				count = compareVectors(evaluatorObject.vectorGt[gt_index-1],evaluatorObject.vectorMs[ms_index-1]);

				//storing info for comparisons done to not repeat again
				stringstream stream_temp;
				stream_temp << gt_index << ":" << ms_index;
				vec_comparisons.push_back(stream_temp.str().c_str());

				//Storing Result
				resultOverlap[gt_index-1][ms_index-1] = count;		//storing the overlap result
				it_ms_regions++;
			}

			it_correspondence++;
		}
	}
	//Brute Force comparison for the rest of the points
	for(int i = 0; i < evaluatorObject.getCountGt(); i++){

		for(int j = 0; j < evaluatorObject.getCountMs(); j++){
			//checking if the comparison is already done
			stringstream stream_temp;
			stream_temp<<i+1<<":"<<j+1;

			if( find(vec_comparisons.begin(),vec_comparisons.end(),stream_temp.str().c_str()) == vec_comparisons.end()){
				int count=0;
		/**/				count = compareVectors(evaluatorObject.vectorGt[i],evaluatorObject.vectorMs[j]);

				resultOverlap[i][j] = count;		//storing the overlap result
			}
		}
	}

	//Calculating the unlabelled pixels from each region
	for ( int i =0; i<evaluatorObject.getCountGt(); i++){
		if(!evaluatorObject.vectorGt[i].empty())
			resultOverlap[i][evaluatorObject.getCountMs()] = evaluatorObject.vectorGt[i].size();

	}

	//Calculating labelled noise
	for ( int i =0; i<evaluatorObject.getCountMs(); i++){
		if(!evaluatorObject.vectorMs[i].empty())
			resultOverlap[evaluatorObject.getCountGt()][i] = evaluatorObject.vectorMs[i].size();
	}


}

void MetricCalculator::calculateGtMetrics(){

	for (int i=0; i < evaluatorObject.getCountGt(); i++) {
		int maxIndex=0;
		int max = resultOverlap[i][0];
		for(int j = 0; j<evaluatorObject.getCountMs(); j++)
		{
			if( resultOverlap[i][j] > max ){
				max = resultOverlap[i][j];
				maxIndex=j;
			}
		}


		//size of region
		resultOverlap[i][evaluatorObject.getCountMs()+1]=0;
		for(int j = 0; j< evaluatorObject.getCountMs();j++){
			resultOverlap[i][evaluatorObject.getCountMs()+1] +=resultOverlap[i][j];
		}
		resultGtMetrics[i][size_of_region_gt]=(resultOverlap[i][evaluatorObject.getCountMs()+1]/evaluatorObject.getImageSize()) * 100;
		//correct_detection
		resultGtMetrics[i][correct_detection] = (resultOverlap[i][maxIndex] /resultOverlap[i][evaluatorObject.getCountMs()+1]) * 100;
		//no_of_overseg_regions
		int temp=0;
		for(int j = 0; j< evaluatorObject.getCountMs();j++){
			if(resultOverlap[i][j]>0) temp++;
		}
		if(temp>0)resultGtMetrics[i][no_overseg_region]=temp-1;
		else resultGtMetrics[i][no_overseg_region]=0;
		//amount_of_overseg
		resultGtMetrics[i][percent_overseg_region] =0;
		for(int j = 0; j< evaluatorObject.getCountMs();j++){
			if(j != maxIndex) resultGtMetrics[i][percent_overseg_region] += resultOverlap[i][j];
		}
		resultGtMetrics[i][percent_overseg_region] = (resultGtMetrics[i][percent_overseg_region]/resultOverlap[i][evaluatorObject.getCountMs()+1])*100;
		//amount of non-classification
		resultGtMetrics[i][percent_non_classification] = (resultOverlap[i][evaluatorObject.getCountMs()]/resultOverlap[i][evaluatorObject.getCountMs()+1])*100;

	}

}
void MetricCalculator::calculateMsMetrics(){
	for (int i = 0; i<evaluatorObject.getCountMs(); i++){

		//counting best fit
		double max = resultOverlap[0][i];
		int maxIndex = 0;
		for(int j = 0; j<evaluatorObject.getCountGt(); j++)
		{
			if( resultOverlap[j][i] > max ){
				max = resultOverlap[j][i];
				maxIndex=j;
			}
		}

		//total pixels in the ith ms region
		double total_points_ms = 0;

		for (int j=0; j<evaluatorObject.getCountGt(); j++){
			total_points_ms += resultOverlap[j][i];
		}



		//MS region size
		resultMsMetrics[i][size_of_region_ms] = ( total_points_ms/evaluatorObject.getImageSize() )*100;

		//percentage of wrong classification
		resultMsMetrics[i][percent_wrong_classification] =0;
		for(int j = 0; j< evaluatorObject.getCountGt()+1;j++){
			if(j != maxIndex) {
				resultMsMetrics[i][percent_wrong_classification] += resultOverlap[j][i];
			}
		}
		resultMsMetrics[i][percent_wrong_classification] = (resultMsMetrics[i][percent_wrong_classification]/total_points_ms)*100;

		//no of under segmented regions
		int temp=0;
		for(int j = 0; j< evaluatorObject.getCountGt();j++){
			if(resultOverlap[j][i]>0) temp++;
		}
		if(temp>0)resultMsMetrics[i][no_underseg_region]=temp-1;
		else resultMsMetrics[i][no_underseg_region]=0;


		//amount_of_underseg
		resultMsMetrics[i][percent_underseg] =0;
		for(int j = 0; j< evaluatorObject.getCountGt();j++){
			if(j != maxIndex) resultMsMetrics[i][percent_underseg] += resultOverlap[j][i];
		}
		resultMsMetrics[i][percent_underseg] = (resultMsMetrics[i][percent_underseg]/total_points_ms)*100;


		//amount of noise-classification
		resultMsMetrics[i][percent_noise_classification] = (resultOverlap[evaluatorObject.getCountGt()][i]/total_points_ms)*100;

	}

}


int MetricCalculator::compareVectors (vector<string>& v1, vector<string>& v2){
	int count=0;
	vector<string>::iterator it1,it2;
	string s;

	it1 = v1.begin();
	while(it1!=v1.end()){
		s = *it1;

		it2 = find(v2.begin(), v2.end(),s);

		if (  it2 != v2.end() ) {
			count++;
			v2.erase(it2);
			v1.erase(it1);
		}
		else{
			it1++;
		}
	}
	return count;
}

int MetricCalculator::maxElementIndex(double array[],int start,int end)
{
	int index=start;
	if(start<=end){
		double max = array[start];       // start with max = first element

		for(int i = start; i<=end; i++)
		{
			if(array[i] > max){
				max = array[i];
				index=i;
			}
		}

	}
	return index;
}

Evaluator MetricCalculator::getEvaluatorObject(){
	return evaluatorObject;
}

int MetricCalculator::getNoOfMetricsGt(){
	return noOfMetricsGt;
}

int MetricCalculator::getNoOfMetricsMs(){
	return noOfMetricsMs;
}

MetricCalculator::~MetricCalculator() {}
}
