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

#include "IterativeClosestPointFactory.h"
#include "util/ConfigurationFileHandler.h"
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
//	this->icpConfigurator = 0; //boost does not support this?
}

IterativeClosestPointFactory::~IterativeClosestPointFactory() {

}

IIterativeClosestPointPtr IterativeClosestPointFactory::createIterativeClosestPoint() {
	IPointCorrespondence* assigner = new PointCorrespondenceKDTree();;
	IRigidTransformationEstimation* estimator = new RigidTransformationEstimationSVD();
	return IIterativeClosestPointPtr(new IterativeClosestPoint(assigner, estimator));
}

IIterativeClosestPointPtr IterativeClosestPointFactory::createIterativeClosestPoint(std::string configurationFile) {
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
	IIterativeClosestPointPtr icp; //abstract interface to ICP
//	IIterativeClosestPointSetupPtr icpConfigurator; //setup interface

	string icpImplementation;
	configReader.getAttribute("IterativeClosestPoint", "implementation", &icpImplementation);

	if (icpImplementation.compare("IterativeClosestPoint") == 0) {
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


		icp = IIterativeClosestPointPtr(new IterativeClosestPoint(assigner, estimator));
		boost::shared_ptr<IterativeClosestPoint> icpTmpHandle = boost::dynamic_pointer_cast<IterativeClosestPoint>(icp); //downcast
		icpConfigurator = boost::dynamic_pointer_cast<IIterativeClosestPointSetup>(icp); //upcast to setup interface

	} else { //Neither IterativeClosestPoint6DSLAM nor IterativeClosestPoint
		cout << "WARNING: Errors during parsing configuration file. Factory fill provide default configuration." << endl;
		return createIterativeClosestPoint();
	}

	/*
	 * process parameters of icp
	 */
//	IIterativeClosestPointSetup* icpConfigurator;

//	std::tr1::shared_ptr<Basesp0(new Derived); // http://bytes.com/topic/c/answers/542252-boost-shared_ptr-polymorphism
//	std::tr1::shared_ptr<Derivedsp1 = dynamic_pointer_cast<Derived>(sp0);
//	boost::shared_ptr<Derived> dp=boost::dynamic_pointer_cast<Derived>(bp);


	assert(icpConfigurator != 0); //cast worked => icp implementation implements IIterativeClosestPointSetup interface

	int maxIterations;
	if (configReader.getAttribute("IterativeClosestPoint", "maxIterations", &maxIterations)) {
		icpConfigurator->setMaxIterations(maxIterations);
	}

	double convergenceThreshold;
	if (configReader.getAttribute("IterativeClosestPoint", "convergenceThreshold", &convergenceThreshold)) {
		icpConfigurator->setConvergenceThreshold(convergenceThreshold);
	}

	summary << "#" << endl << "# Parameters:" << endl;
	summary << "#  maxIterations = " << icpConfigurator->getMaxIterations() << endl;
	summary << "#  convergenceThreshold = " << icpConfigurator->getConvergenceThreshold() << endl;
	summary << "#" << endl << "###########################################################" << endl;

	LOG(INFO) << "Summary: " << std::endl << summary.str();

	return icp;

}

IIterativeClosestPointSetupPtr IterativeClosestPointFactory::getIcpSetupHandle() {
	return this->icpConfigurator;
}


}

/* EOF */
