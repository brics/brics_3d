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
/* Currently (nearly) all the functionality of PointCloud3D is reimlemented, although ColoredPointCloud3D is a derivative of PointCloud3D!
 * This is necessary, as the internal data structure is changed from std::vector<Point3D>* pointCloud  to std::vector<ColoredPoint3D>* pointCloud.
 * The vector itself is not polymorph as it is a template. Therefore the std::vector<ColoredPoint3D>* pointCloud cannot be up-casted to std::vector<Point3D>* pointCloud.
 * Whenever a non-overridden, inherited function in ColoredPointCloud is invoked the method works with an empty vector from the base class...
 *
 * To prevent too much code duplication the refactorization to a template might be a better solution - but when the point cloud is a template then dynamic polymorphism can not be applied anymore...
 * forcing algorithms to have an overloaded function with all possible template variants - certainly something we do not want...
 *
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

//	std::vector<Point3D>* getPointCloud();

	/**
	 * @brief Get the reference to the colored point cloud
	 * @return Pointer to point cloud
	 */
	std::vector<ColoredPoint3D>* getPointCloud();
//	std::vector<ColoredPoint3D>* getColoredPointCloud();

	/**
	 * @brief Set the reference to the colored point cloud
	 * @param pointCloud Pointer to new colored point cloud
	 */
	void setPointCloud(std::vector<ColoredPoint3D>* pointCloud);

	/**
	 * @brief Overridden >> operator.
	 *
	 * Reads a point cloud from a stream an stores it. E.g. e.g. std::cin >> coloredPointCloudObj;
	 *
	 * @param inStream The input stream
	 * @param pointCloud Pointer to point where input is stored
	 * @return Input stream
	 */
	friend istream& operator>>(istream &inStream, ColoredPointCloud3D &pointCloud);

	/**
	 * @brief Overridden << operator.
	 *
	 * Writes a point cloud to a stream e.g. std::cout << coloredPointCloudObj;
	 *
	 * @param outStream The output stream
	 * @param pointCloud Pointer to point which data is forwarded to output stream
	 * @return Output stream
	 */
	friend ostream& operator<<(ostream &outStream, ColoredPointCloud3D &pointCloud);

	/**
	 * @brief Applies a homogeneous transformation to the point cloud
	 *
	 * @param[in] transformation The homogeneous transformation matrix that will be applied
	 */
	void homogeneousTransformation(IHomogeneousMatrix44 *transformation);

protected:

	///Overridden pointer to vector which represents a Cartesian point cloud with color information
	std::vector<ColoredPoint3D>* pointCloud;

};

}

#endif /* COLOREDPOINTCLOUD_H_ */

/* EOF */
