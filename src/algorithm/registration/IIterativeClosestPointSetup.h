/**
 * @file 
 * IIterativeClosestPointSetup.h
 *
 * @date: Feb 12, 2010
 * @author: sblume
 */

#ifndef IITERATIVECLOSESTPOINTSETUP_H_
#define IITERATIVECLOSESTPOINTSETUP_H_


namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Abstract interface to setup and configure the ICP
 */
class IIterativeClosestPointSetup {
public:

	/**
	 * @brief Standard constructor
	 */
	IIterativeClosestPointSetup(){};

	/**
	 * @brief Standard destructor
	 */
	virtual ~IIterativeClosestPointSetup(){};

	/**
	 * @brief Get the threshold to define convergence
	 * @return The convergence threshold
	 */
	virtual double getConvergenceThreshold() const = 0;

	/**
	 * @brief Get the maximum amount of iterations for matching process
	 * @return The maximum amount of iterations
	 */
	virtual int getMaxIterations() const = 0;

	/**
	 * @brief Set the threshold to define convergence
	 * @param convergenceThreshold The convergence threshold
	 */
	virtual void setConvergenceThreshold(double convergenceThreshold) = 0;

	/**
	 * @brief Set the maximum amount of iterations for matching process
	 * @param maxIterations The maximum amount of iterations
	 */
	virtual void setMaxIterations(int maxIterations) = 0;
};

}

#endif /* IITERATIVECLOSESTPOINTSETUP_H_ */

/* EOF */
