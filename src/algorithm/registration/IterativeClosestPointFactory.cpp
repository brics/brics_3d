/**
 * @file 
 * IterativeClosestPointFactory.cpp
 *
 * @date: Dec 15, 2009
 * @author: sblume
 */

#include "IterativeClosestPointFactory.h"
#include "util/ConfigurationFileHandler.h"
#include "algorithm/registration/IterativeClosestPoint6DSLAM.h"
#include "algorithm/registration/IterativeClosestPoint.h"
#include "algorithm/registration/PointCorrespondenceKDTree.h"
#include "algorithm/registration/RigidTransformationEstimationSVD.h"
#include "algorithm/registration/RigidTransformationEstimationHELIX.h"
#include "algorithm/registration/RigidTransformationEstimationAPX.h"
#include "algorithm/registration/RigidTransformationEstimationQUAT.h"
#include "algorithm/registration/RigidTransformationEstimationORTHO.h"


#include <iostream>
#include <sstream>

using std::cout;
using std::cerr;
using std::endl;
using std::string;

namespace BRICS_3D {



IterativeClosestPointFactory::IterativeClosestPointFactory() {

}

IterativeClosestPointFactory::~IterativeClosestPointFactory() {

}

IIterativeClosestPoint* IterativeClosestPointFactory::createIterativeClosestPoint() {
	return new IterativeClosestPoint(); //TODO return smart pointer
}

IIterativeClosestPoint* IterativeClosestPointFactory::createIterativeClosestPoint(std::string configurationFile) {
	if (((configurationFile.compare("")) == 0) || (configurationFile.compare("default") == 0)) {
		return createIterativeClosestPoint();
	}

	ConfigurationFileHandler configReader(configurationFile);
	if (configReader.getErrorsOccured()) {
		cout << "WARNING: Errors during parsing configuration file. Factory fill provide default configuration." << endl;
		return createIterativeClosestPoint();
	}

	std::stringstream summary;
	summary << "#################### ICP Configuration ####################" << endl << "#" <<endl;

	/* set up icp */
	IIterativeClosestPoint* icp; //abstract interface to ICP

	string icpImplementation;
	configReader.getAttribute("IterativeClosestPoint", "implementation", &icpImplementation);
	if (icpImplementation.compare("IterativeClosestPoint6DSLAM") == 0) {
		icp = new IterativeClosestPoint6DSLAM();
		summary << "# Implementation: IterativeClosestPoint6DSLAM" << endl;

	} else if (icpImplementation.compare("IterativeClosestPoint") == 0) {
		IPointCorrespondence* assigner;
		IRigidTransformationEstimation* estimator;
		string subalgorithm;
		summary << "# Implementation: IterativeClosestPoint" << endl;

		/* process assigner */
		if(configReader.getSubAlgorithm("IterativeClosestPoint", "PointCorrespondence", &subalgorithm)) {
			if (subalgorithm.compare("PointCorrespondenceKDTree") == 0) {
				assigner = new PointCorrespondenceKDTree();
				summary << "#  Subalgorithm: " << subalgorithm << endl;
//			} else if (...) {	//add more implemetation here

			} else {
				cout << "WARNING: PointCorrespondence implementation not found. Factory fill provide default configuration: PointCorrespondenceKDTree" << endl;
					assigner = new PointCorrespondenceKDTree();
					summary << "#  Subalgorithm: PointCorrespondenceKDTree (DEFAULT)" << endl;
			}
		} else {
			cout << "WARNING: PointCorrespondence implementation not found. Factory fill provide default configuration: PointCorrespondenceKDTree" << endl;
			assigner = new PointCorrespondenceKDTree();
			summary << "#  Subalgorithm: PointCorrespondenceKDTree (DEFAULT)" << endl;
		}

		/* process estimator */
		if(configReader.getSubAlgorithm("IterativeClosestPoint", "RigidTransformationEstimation", &subalgorithm)) {
			if (subalgorithm.compare("RigidTransformationEstimationSVD") == 0) {
				estimator = new RigidTransformationEstimationSVD();
				summary << "#  Subalgorithm: " << subalgorithm << endl;

			} else if (subalgorithm.compare("RigidTransformationEstimationAPX") == 0) {
				estimator = new RigidTransformationEstimationAPX();
				summary << "#  Subalgorithm: " << subalgorithm << endl;

			} else if (subalgorithm.compare("RigidTransformationEstimationHELIX") == 0) {
				estimator = new RigidTransformationEstimationHELIX();
				summary << "#  Subalgorithm: " << subalgorithm << endl;

			} else if (subalgorithm.compare("RigidTransformationEstimationQUAT") == 0) {
				estimator = new RigidTransformationEstimationQUAT();
				summary << "#  Subalgorithm: " << subalgorithm << endl;

			} else if (subalgorithm.compare("RigidTransformationEstimationORTHO") == 0) {
				estimator = new RigidTransformationEstimationORTHO();
				summary << "#  Subalgorithm: " << subalgorithm << endl;

//			} else if (...) {	//add more implementation here

			} else {
				cout << "WARNING: RigidTransformationEstimation implementation not found. Factory fill provide default configuration: RigidTransformationEstimationSVD" << endl;
				estimator = new RigidTransformationEstimationSVD();
				summary << "#  Subalgorithm: RigidTransformationEstimationSVD (DEFAULT)" << endl;
			}

		} else {
			cout << "WARNING: RigidTransformationEstimation implementation not found. Factory fill provide default configuration: RigidTransformationEstimationSVD" << endl;
			estimator = new RigidTransformationEstimationSVD();
			summary << "#  Subalgorithm: RigidTransformationEstimationSVD (DEFAULT)" << endl;
		}


		icp = new IterativeClosestPoint(assigner, estimator);

	} else { //Neither IterativeClosestPoint6DSLAM nor IterativeClosestPoint
		cout << "WARNING: Errors during parsing configuration file. Factory fill provide default configuration." << endl;
		return createIterativeClosestPoint();
	}

	/*
	 * process parameters of icp
	 */
	int maxIterations;
	if (configReader.getAttribute("IterativeClosestPoint", "maxIterations", &maxIterations)) {
		icp->setMaxIterations(maxIterations);
	}

	double convergenceThreshold;
	if (configReader.getAttribute("IterativeClosestPoint", "convergenceThreshold", &convergenceThreshold)) {
		icp->setConvergenceThreshold(convergenceThreshold);
	}

	summary << "#" << endl << "# Parameters:" << endl;
	summary << "#  maxIterations = " << icp->getMaxIterations() << endl;
	summary << "#  convergenceThreshold = " << icp->getConvergenceThreshold() << endl;
	summary << "#" << endl << "###########################################################" << endl;

	cout << summary.str();

	return icp; //TODO

}

}

/* EOF */
