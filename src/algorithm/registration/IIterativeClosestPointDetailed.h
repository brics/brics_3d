/**
 * @file 
 * IIterativeClosestPointDetailed.h
 *
 * @date: Feb 12, 2010
 * @author: sblume
 */

#ifndef IITERATIVECLOSESTPOINTDETAILED_H_
#define IITERATIVECLOSESTPOINTDETAILED_H_

#include "core/IHomogeneousMatrix44.h"

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Stateful abstract interface for ICP
 *
 * This is a stateful interface, that relies on previous states e.g. invocations of setData(), setModel() or
 * performNextIteration().
 * To correctly use this interface first set data and model, then invoke performNextIteration
 * as long as you want (probably until the error is smaller than a certain value).
 */
class IIterativeClosestPointDetailed {
public:

	/**
	 * Standard constructor
	 */
	IIterativeClosestPointDetailed(){};

	/**
	 * Standard destructor
	 */
	virtual ~IIterativeClosestPointDetailed(){};

	/**
	 * @brief Set the "data"
	 * @param data Pointer to data point cloud
	 */
	virtual void setData(PointCloud3D* data) = 0;

	/**
	 * @brief Set the "model"
	 * @param model Pointer to data point cloud
	 */
	virtual void setModel(PointCloud3D* model) = 0;

	/**
	 * @brief Get data
	 * @return Pointer to data point cloud, 0 if data was not set before
	 */
	virtual PointCloud3D* getData() = 0;

	/**
	 * @brief Get model
	 * @return Pointer to data point cloud, 0 if data was not set before
	 */
	virtual PointCloud3D* getModel() = 0;

	/**
	 * @brief Perform one step/iteration of the ICP algoritham
	 *
	 * This is a stateful function, that relies on previous states. Thus the output is dependent on
	 * what kind of data was set in setData() and setModel() and how many time performNextIteration()
	 * has been invoked before. <br>
	 * <br>Assumption:<br> model and data are set before! <br>
	 * Note that the data will me updated during this operation.
	 */
	virtual double performNextIteration() = 0;

	/**
	 * @brief Get the estimated rigid transformation in the last ICP iteration.
	 * @return Pointer to transformation, 0 if no ICP iteration was done before
	 */
	virtual IHomogeneousMatrix44* getLastEstimatedTransformation() = 0;

	/**
	 * @brief Get the accumulated rigid transformation in the last ICP iteration.
	 * If no more iterations are performed this will be the final result.
	 *
	 * @return Pointer to transformation, 0 if no ICP iteration was done before
	 */
	virtual IHomogeneousMatrix44*  getAccumulatedTransfomation() = 0;



};

}

#endif /* IITERATIVECLOSESTPOINTDETAILED_H_ */

/* EOF */
