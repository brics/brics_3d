/*
 * MetricCalculator.h
 *
 * @author: Pinaki Banerjee
 * @date: Apr 10, 2011
 * @version: 0.1
 * @description
 * Performs all the calculation for generating the results of evaluation metrics.
 */

#ifndef METRICCALCULATOR_H_
#define METRICCALCULATOR_H_

#include "Evaluator.h"



namespace BRICS_3D{
class MetricCalculator {
private:
	double **resultOverlap;
	double **resultGtMetrics;
	double **resultMsMetrics;
	Evaluator evaluatorObject;
	static const int noOfMetricsGt =5;
	static const int noOfMetricsMs =5;
	static const int size_of_region_gt = 0;
	static const int correct_detection = 1;
	static const int no_overseg_region = 2;
	static const int percent_overseg_region = 3;
	static const int percent_non_classification=4;
	static const int size_of_region_ms =0;
	static const int percent_wrong_classification = 1;
	static const int no_underseg_region = 2;
	static const int percent_underseg = 3;
	static const int percent_noise_classification = 4;
	void calculateOverlap();
	void calculateGtMetrics();
	void calculateMsMetrics();

public:
	MetricCalculator();
	virtual ~MetricCalculator();
	int calculateMetrics();
    double **getResultGtMetrics() const;
    double **getResultMsMetrics() const;
    double **getResultOverlap() const;
    int getNoOfMetricsGt();
    int getNoOfMetricsMs();
    void setEvaluatorObject(Evaluator evaluatorObj);
    Evaluator getEvaluatorObject();

    /**
     * Method to copare two vectors and return number of identical elements. It also
     * deletes the elements from the vectors that have matched with an element from the
     * other vector
     */
    int compareVectors (vector<string>& v1, vector<string>& v2);

    /**
     * Returns the index of the largest element in the array. start and end defines the
     * span of indexes to be checked for
     */
    int maxElementIndex(double array[],int start,int end);
};
}
#endif /* METRICCALCULATOR_H_ */
