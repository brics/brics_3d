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

#ifndef BRICS_3D_CARTESIANPOINTCLOUD_H_
#define BRICS_3D_CARTESIANPOINTCLOUD_H_

#include <vector>
#include <string>
#include <boost/ptr_container/ptr_vector.hpp>

#include "core/Point3D.h"

#define USE_POINTER_VECTOR

namespace brics_3d {

/**
 * @brief Class to represent a Cartesian 3D point cloud
 */
class PointCloud3D {
public:

	typedef boost::shared_ptr<PointCloud3D> PointCloud3DPtr;
	typedef boost::shared_ptr<PointCloud3D const> PointCloud3DConstPtr;

	/**
	 * @brief Standard constuctor
	 */
	PointCloud3D();

	/**
	 * @brief Standard destructor
	 */
	virtual ~PointCloud3D();

	/**
	 * @brief Add a point to the point cloud
	 * @param point Point that will be added
	 */
	void addPoint(Point3D point);

	/**
	 * @brief Add a point to the point cloud with reference semantics.
	 * Take this function to add complex/decorated point types.
	 * @param point Pointer to the point. The point cloud will take over ownership for the pointer and take care about automatic deletion.
	 */
	void addPointPtr(Point3D* point);

#ifdef USE_POINTER_VECTOR
	/**
	 * @brief Get the pointer to the point cloud
	 * @return Pointer to point cloud
	 */
    boost::ptr_vector<Point3D>* getPointCloud();

	/**
	 * @brief Set the pointer to the point cloud
	 * @param pointCloud Pointer to new point cloud
	 */
    void setPointCloud(boost::ptr_vector<Point3D>* pointCloud);

#else
    std::vector<Point3D>* getPointCloud();
    void setPointCloud(std::vector<Point3D>* pointCloud);
#endif

    /**
     * @brief Get the number of points in the point cloud
     * @return Size of point cloud (= number of stored points)
     */
    unsigned int getSize();

    /**
     * @brief Stores the point cloud into a ply file (Stanford polygon file format)
     * @param filename Specifies the name of the file .e.g. point_cloud.ply
     */
    void storeToPlyFile(std::string filename);

    /**
     * @brief Stores the point cloud into a sipmle text file (x y z)
     * @param filename Specifies the name of the file .e.g. point_cloud.txt
     */
    void storeToTxtFile(std::string filename);

    /**
     * @brief Reads data from a simple text file (x y z) and appends it to this point cloud
     * @param filename Specifies the name of the file .e.g. point_cloud.txt
     */
    void readFromTxtFile(std::string filename);

	/**
	 * @brief Overridden >> operator.
	 *
	 * Reads a point cloud from a stream an stores it. E.g. e.g. std::cin >> pointCloudObj;
	 *
	 * @param inStream The input stream
	 * @param pointCloud Reference to point where input is stored
	 * @return Input stream
	 */
	friend istream& operator>>(istream &inStream, PointCloud3D &pointCloud);

	/**
	 * @brief Overridden << operator.
	 *
	 * Writes a point cloud to a stream e.g. std::cout << pointCloudObj;
	 *
	 * @param outStream The output stream
	 * @param pointCloud Reference to point which data is forwarded to output stream
	 * @return Output stream
	 */
	friend ostream& operator<<(ostream &outStream, PointCloud3D &pointCloud);

	/**
	 * @brief Applies a homogeneous transformation to the point cloud
	 *
	 * @param[in] transformation The homogeneous transformation matrix that will be applied
	 */
	void homogeneousTransformation(IHomogeneousMatrix44* transformation);

protected:

#ifdef USE_POINTER_VECTOR

	///Pointer to vector which represents a Cartesian point cloud
	boost::ptr_vector<Point3D>* pointCloud;

#else
	std::vector<Point3D>* pointCloud;
#endif

};

}

#endif /* BRICS_3D_CARTESIANPOINTCLOUD_H_ */

/* EOF */

