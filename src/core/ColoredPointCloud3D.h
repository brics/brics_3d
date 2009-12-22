/**
 * @file 
 * ColoredPointCloud3D.h
 *
 * @date: Dec 21, 2009
 * @author: sblume
 */

#ifndef COLOREDPOINTCLOUD_H_
#define COLOREDPOINTCLOUD_H_

#include "core/ColoredPoint3D.h"
#include "core/PointCloud3D.h"

namespace BRICS_3D {

/**
 * @brief Class to represent a Cartesian 3D point cloud with color information
 *
 */
class ColoredPointCloud3D : public PointCloud3D {
public:

	/**
	 * @brief: Standard constructor
	 */
	ColoredPointCloud3D();

	/**
	 * @brief Standard destructor
	 */
	virtual ~ColoredPointCloud3D();

    /**
     * @brief Get the number of points in the colored point cloud
     * @return Size of colored point cloud (= number of stored points)
     */
	unsigned int getSize();

	/**
	 * @brief Add a point to the colored point cloud
	 * @param point Colored point that will be added
	 */
	void addPoint(ColoredPoint3D point);

	/**
	 * @brief Get the reference to the colored point cloud
	 * @return Pointer to point cloud
	 */
	std::vector<ColoredPoint3D>* getPointCloud();

	/**
	 * @brief Set the reference to the colored point cloud
	 * @param pointCloud Pointer to new colored point cloud
	 */
	void setPointCloud(std::vector<ColoredPoint3D>* pointCloud);

protected:

	///Overridden pointer to vector which represents a Cartesian point cloud with color information
	std::vector<ColoredPoint3D>* pointCloud;

};

}

#endif /* COLOREDPOINTCLOUD_H_ */

/* EOF */
