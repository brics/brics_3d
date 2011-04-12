/*
 * Comparator.h
 *
 * @author: Pinaki Banerjee
 * @date: Apr 10, 2011
 * @version: 0.1
 * @description
 * The class handles all the calculations and plotting related to comparing the algorithms
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_
#include<vector>
#include<string>
using namespace std;
namespace BRICS_3D{
class Comparator {
private:
	bool initialiseStatus;
	vector<string> vecMethodNames;	//contains the names of the methoods to be compared
	string overlapBaseName;
	string GtMetricsBaseName;
	string MsMetricsBaseName;
	string path;	//path to csv evaluation metric files

	/**
	 * private method for generating the plots for GT metric and MS metric based comparison
	 */
	void generateGTPlots(vector<string> *vecMetrics, int noOfMetrics, vector<string> metricNames,string regionType, string nameOfFile);
	void generateParametericPlots(vector<string> *vecMetrics, int noOfMetrics, vector<string> metricNames,string nameOfFile);

	/**
	 * Calculate and return the mean of an array of double values
	 */
	double calculateMean(double *values, int sizeOfArray);

	/**
	 * Calculate and return the median of an array of double values
	 */
	double calculateMedian(double *values, int sizeOfArray);

	/**
	 * Calculate and return the variance of an array of double values
	 */
	double calculateVariance(double *values, int sizeOfArray);

	/**
	 * Calculate and return the standard deviation of an array of double values
	 */
	double calculateStdDeviation(double *values, int sizeOfArray);


public:
	Comparator();
	virtual ~Comparator();

	/**
	 * Check if the comparator object is properly initialized or not
	 */
	bool isInitialised();

	/**
	 * Performs the comparison of set methods
	 */
	void compare();

	string getGtMetricsBaseName() const;

	/**
	 * Setting folder path to the metric files to be used for comparison
	 */
	void setPath(string path);

	/**
	 * Get the initialization status of the comparator
	 */
	bool getInitialiseStatus() const;
	vector<string> getMethods() const;
	string getMsMetricsBaseName() const;
	string getOverlapBaseName() const;
	void setGtMetricsBaseName(string GtMetricsBaseName);
	void setMethods(vector<string> methods);
	void setMsMetricsBaseName(string MsMetricsBaseName);
	void setOverlapBaseName(string overlapBaseName);

};
}
#endif /* COMPARATOR_H_ */
