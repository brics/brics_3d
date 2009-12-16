/**
 * @file 
 * IterativeClosestPointFactory.h
 *
 * @date: Dec 15, 2009
 * @author: sblume
 */

#ifndef ITERATIVECLOSESTPOINTFACTORY_H_
#define ITERATIVECLOSESTPOINTFACTORY_H_

#include "algorithm/registration/IIterativeClosestPoint.h"
#include <string>

#include <boost/shared_ptr.hpp>


namespace BRICS_3D {

typedef boost::shared_ptr<IIterativeClosestPoint> IIterativeClosestPointPtr;

/**
 * @ingroup registration
 * @brief IterativeClosestPoint factory that can be configured with a XML file
 *
 */
class IterativeClosestPointFactory {
public:

	/**
	 * @brief Standard constructor
	 */
	IterativeClosestPointFactory();

	/**
	 * @brief Standard destructor
	 */
	virtual ~IterativeClosestPointFactory();

	/**
	 * @brief Factory method the returns an IterativeClosestPoint in a standard configuration.
	 */
	IIterativeClosestPointPtr createIterativeClosestPoint();

	/**
	 * @brief Factory method the returns an IterativeClosestPoint configured by an XML file
	 *
	 * Wrong parameters descriptions in the XML file will be handled as warnings and the default parameter is chosen.
	 *
	 * @param configurationFile Path to the XML configuration file
	 * @return Returns an instance of the abstract Iterative Closest Point interface
	 *
	 * <br><b>Example XML file:</b><br>
	 * <code>
	 * <!DOCTYPE dummy> <br>
	 * <BRICS_3D-Configuration> <br> <br>
	 *
	 * &nbsp;&nbsp;<IterativeClosestPoint implementation="IterativeClosestPoint" maxIterations="25" convergenceThreshold="0.001"> <br>
     * &nbsp;&nbsp;&nbsp;&nbsp;<PointCorrespondence implementation="PointCorrespondenceKDTree"></PointCorrespondence> <br>
	 * &nbsp;&nbsp;&nbsp;&nbsp;<RigidTransformationEstimation implementation="RigidTransformationEstimationSVD"></RigidTransformationEstimation> <br>
	 * &nbsp;&nbsp;</IterativeClosestPoint> <br> <br>
     *
     * </BRICS_3D-Configuration>
	 * </code>
	 * <br><br>
	 * <b>NOTE1:</b> The current implementation of the <code>ConfigurationFileHandlerTest</code> configuration file parser bases on the Xerces library.
	 * If it is not installed the default configuration is choosen.<br>
	 */
	IIterativeClosestPointPtr createIterativeClosestPoint(std::string configurationFile);

};

}

#endif /* ITERATIVECLOSESTPOINTFACTORY_H_ */

/* EOF */
