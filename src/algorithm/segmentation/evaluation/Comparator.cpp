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

#include <vector>
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <boost/algorithm/string.hpp>

#include "Comparator.h"
#include "FileReader.h"
#include "gnuplot_i.hpp" //Gnuplot class handles POSIX-Pipe-communikation with Gnuplot

using namespace std;
namespace brics_3d{
Comparator::Comparator() {
	initialiseStatus=false;
	overlapBaseName="_result_overlapMatrix.csv";
	GtMetricsBaseName="_reslut_GtMetrics.csv";
	MsMetricsBaseName="_result_MsMetrics.csv";
	path="";
}

void Comparator::setPath(string path){
	this->path=path;
}

string Comparator::getGtMetricsBaseName() const
{
	return GtMetricsBaseName;
}

bool Comparator::getInitialiseStatus() const
{
	return initialiseStatus;
}

vector<string> Comparator::getMethods() const
{
	return vecMethodNames;
}

string Comparator::getMsMetricsBaseName() const
{
	return MsMetricsBaseName;
}

string Comparator::getOverlapBaseName() const
{
	return overlapBaseName;
}

void Comparator::setGtMetricsBaseName(string GtMetricsBaseName)
{
	this->GtMetricsBaseName = GtMetricsBaseName;
}

void Comparator::setMethods(vector<string> methods)
{
	this->vecMethodNames = methods;
	initialiseStatus=true;
}

void Comparator::setMsMetricsBaseName(string MsMetricsBaseName)
{
	this->MsMetricsBaseName = MsMetricsBaseName;
}

void Comparator::setOverlapBaseName(string overlapBaseName)
{
	this->overlapBaseName = overlapBaseName;
}


bool Comparator::isInitialised(){
	return initialiseStatus;
}

void Comparator::compare(){

	FileReader fileReader;
	ifstream file;
	stringstream filename;
	vector<string> gtMetricNames;
	vector<string> *vecGtMetrics = new vector<string>[vecMethodNames.size()];
	vector<string> *vecMsMetrics= new vector<string>[vecMethodNames.size()];
	vector<string> vecSeperatedMsMetrics;
	vector<string> msMetricNames;
	vector<string> vecSeperatedGtMetrics;
	int noOfGtMetrics =0;
	int noOfMsMetrics =0;


	//reading files into the corresponding vectors

	for(unsigned int i=0; i<vecMethodNames.size();i++){

		//Getting GtMetrics
		filename.str("");
		filename.clear();
		filename << path << vecMethodNames[i] <<GtMetricsBaseName;
		fileReader.toStringVector(filename.str(),vecGtMetrics[i]);

		//Getting MsMetrics
		filename.str("");
		filename.clear();
		filename << path<<vecMethodNames[i] <<MsMetricsBaseName;
		fileReader.toStringVector(filename.str(),vecMsMetrics[i]);
	}


	if (vecGtMetrics[0].size() > 0 ) {

		//Plotting Gt based metric comparison
		boost::split(vecSeperatedGtMetrics, vecGtMetrics[0][0], boost::is_any_of(","));
		noOfGtMetrics = vecSeperatedGtMetrics.size();
		//setting metric names
		gtMetricNames.push_back("Occupacy of the region");
		gtMetricNames.push_back("Correct Detection (%)");
		gtMetricNames.push_back("No of oversegmented regions");
		gtMetricNames.push_back("Amount of oversegmentation (%)");
		gtMetricNames.push_back("Amount of non-classification (%)");

		generateGTPlots(vecGtMetrics,noOfGtMetrics,gtMetricNames,"GT Region","GT-metrics based comparison");

		//Plotting GT based parametric comparison

		generateParametericPlots(vecGtMetrics,noOfGtMetrics,gtMetricNames,"GT-overall comparison");

		//Plotting MS based metric comparison

		boost::split(vecSeperatedMsMetrics, vecMsMetrics[0][0], boost::is_any_of(","));
		noOfMsMetrics = vecSeperatedMsMetrics.size();
		msMetricNames.push_back("Occupacy_of_the_region");
		msMetricNames.push_back("Wrong_Classification(%)");
		msMetricNames.push_back("No_of_undersegmented_regions");
		msMetricNames.push_back("Amount_of_undersegmentation(%)");
		msMetricNames.push_back("Amount_of_noise-classification(%)");
		generateParametericPlots(vecMsMetrics,noOfMsMetrics,msMetricNames,"MS-metrics based comparison");
		//remove temp files

		for(int i=1; i<noOfGtMetrics; i++){
			stringstream temp;
			temp<<"histData"<<i;
			remove(temp.str().c_str());
		}

	}
	else {
		cout << "File Read Error" << endl;
	}
}

void Comparator::generateParametericPlots(vector<string> *vecMetrics, int noOfMetrics, vector<string> vecmetricNames,string nameOfFile){

	int noOfRegions;
	ofstream tempFile;
	vector<string> vecSeperatedMetrics;
	stringstream queryString;
	Gnuplot g1;
	g1.reset_all();
	g1.savetops(nameOfFile);

	vector<string> vecParametersOfComparison;
	vecParametersOfComparison.push_back("Mean");
	vecParametersOfComparison.push_back("Median");
	//vecParametersOfComparison.push_back("Variance");
	vecParametersOfComparison.push_back("Standard deviation");

	for(int i=1; i<noOfMetrics; i++){
		double mean,median,variance,stdDeviation;
		double *values;
		int yrange=1;
		//create temporary file to create the histogram data
		queryString.str("");
		queryString.clear();
		queryString<<"histData"<<i;
		tempFile.open(queryString.str().c_str());
		tempFile.precision(5);

		//for every method to be compared calculate the values
		for(unsigned int methodCount=0; methodCount<vecMethodNames.size(); methodCount++){
			noOfRegions = vecMetrics[methodCount].size();
			values = new double[noOfRegions];

			for(int j=0; j<noOfRegions;j++){
				boost::split(vecSeperatedMetrics, vecMetrics[methodCount][j], boost::is_any_of(","));
				values[j]= atof(vecSeperatedMetrics[i].c_str());
			}


			//Calculate Mean
			mean = calculateMean(values,noOfRegions);
			//Calculate Median
			median = calculateMedian(values,noOfRegions);
			//Calculate Variance
			variance = calculateVariance(values,noOfRegions);
			//calculate Standard Deviation
			stdDeviation = calculateStdDeviation(values,noOfRegions);
			//find the yRange of the plot
			int tempRange = (int)(mean+ median+ stdDeviation);
			if(yrange < tempRange ) yrange = tempRange;
			//create histogram file
			tempFile<< vecMethodNames[methodCount] << " "<< fixed <<mean<< " " << fixed <<median<< " " << fixed << stdDeviation << endl;
		}

		tempFile.close();

		//initialise the gnuplot object


		g1.reset_plot();
		g1.set_grid();

		queryString.str("");
		queryString.clear();
		queryString << "[" << vecmetricNames[i]<<"] over all Regions";
		g1.set_title(queryString.str());

		g1.set_xlabel("Method Names");

		queryString.str("");
		queryString.clear();
		queryString << "set yrange [0:" << yrange+1 << "]";
		g1.cmd(queryString.str());

		g1.cmd("set boxwidth 0.30 absolute");
		g1.cmd("set style fill solid 1.00 border -1").cmd("set style histogram rowstacked gap 4");
		g1.cmd("set style data histograms");
		g1.set_legend("outside right top");
		g1.cmd("set xtic rotate by -45 scale 0");

	queryString.str("");
	queryString.clear();
	queryString<<"plot 'histData"<<i << "' using ";

	unsigned int parameterCount=0;
	queryString <<2 <<":xtic(1)t \"" << vecParametersOfComparison[parameterCount] <<"\"";
	for(parameterCount =1; parameterCount<= vecParametersOfComparison.size()-1; parameterCount++){
		queryString<<", '' using "<<(parameterCount+2)<< "t \"" << vecParametersOfComparison[parameterCount]<<"\"";
	}
	queryString<<endl;
	//cout<< "\n\n" <<queryString.str()<<endl;
	g1.cmd(queryString.str());
	g1.reset_plot();
	}

}


double Comparator::calculateMean(double *values, int sizeOfArray){

	double mean = 0.0;

	for(int i =0; i < sizeOfArray; i++){
		mean += values[i];
	}

	mean = mean/sizeOfArray;

	return mean;
}

double Comparator::calculateMedian(double *values, int sizeOfArray){

	double median=0.0,temp;

	for(int i=0; i<sizeOfArray-1; i++){
		for(int j=i+1; j<sizeOfArray; j++){
			if(values[j] < values[i])
			{
				temp=values[i]	;
				values[i]=values[j];
				values[j]=temp;
			}
		}
	}

	if(sizeOfArray%2==0)
		return((values[sizeOfArray/2]+values[sizeOfArray/2-1])/2.0);
	else
		return values[sizeOfArray/2];

	return median;
}

double Comparator::calculateVariance(double *values, int sizeOfArray){

	double variance=0.0,mean;

	mean = calculateMean(values,sizeOfArray);

	for (int i =0; i<sizeOfArray; i++){
//		cout<< i<< " : "<<mean << " : "<<values[i]-mean << " : " << (values[i]-mean)*(values[i]-mean)<< "\n";
		variance += (values[i]-mean)*(values[i]-mean);
	}

	variance = variance/sizeOfArray;

	return variance;
}

double Comparator::calculateStdDeviation(double *values, int sizeOfArray){
	double variance = calculateVariance(values,sizeOfArray);
	return sqrt(variance);
}

void Comparator::generateGTPlots(vector<string> *vecMetrics, int noOfMetrics, vector<string> metricNames,string regionType,string nameOfFile){

	int noOfRegions=vecMetrics[0].size();
	stringstream queryString;
	Gnuplot g1;

	g1.reset_all();
	g1.savetops(nameOfFile);

	for(int i=1; i<noOfMetrics;i++){

		ofstream tempFile;
		queryString.str("");
		queryString.clear();
		queryString<<"histData"<<i;
		tempFile.open(queryString.str().c_str());
		vector<string> vecSeperatedMetrics;
		for (int j =0; j< noOfRegions; j++){
			boost::split(vecSeperatedMetrics, vecMetrics[0][j], boost::is_any_of(","));
			tempFile.precision(2);
			tempFile<< "Reg." <<j+1<<"("<<fixed<<atof(vecSeperatedMetrics[0].c_str()) << "%) ";
			for(unsigned int methodCount=0; methodCount<vecMethodNames.size();methodCount++){
				boost::split(vecSeperatedMetrics, vecMetrics[methodCount][j], boost::is_any_of(","));
				tempFile<< vecSeperatedMetrics[i].c_str() << " ";
			}
			tempFile<<endl;
		}
		tempFile.close();
		g1.reset_plot();
		g1.set_grid();
		queryString.str("");
		queryString.clear();
		queryString<<"Comparing Algorithms in terms of: " << metricNames[i];
		g1.set_title(queryString.str());
		g1.set_ylabel(metricNames[i]);
		g1.set_xlabel("GT Regions");
		g1.cmd("set style fill solid 1.00 border -1").cmd("set style histogram clustered gap 4");
		g1.cmd("set style data histograms");
		g1.set_legend("outside right top");
		g1.cmd("set xtic rotate by -45 scale 0");
		queryString.str("");
		queryString.clear();
		queryString<<"plot 'histData"<<i << "' using ";
		unsigned int methodCount=0;
		queryString <<2 <<":xtic(1)t \"" << vecMethodNames[methodCount] <<"\"";
		for(methodCount =1; methodCount<= vecMethodNames.size()-1; methodCount++){
			queryString<<", '' using "<<(methodCount+2)<< " t \"" << vecMethodNames[methodCount]<<"\"";
		}
		queryString<<endl;
		g1.cmd(queryString.str());
		g1.reset_plot();
	}

}

Comparator::~Comparator() {}

}
