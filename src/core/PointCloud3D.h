/**
 * @file
 * PointCloud.h
 * 
 * @author: Sebastian Blumenthal
 * @date: Aug 24, 2009
 * @version: 0.1
 */

#ifndef CARTESIANPOINTCLOUD_H_
#define CARTESIANPOINTCLOUD_H_

#include <vector>
#include <string>

#include "core/Point3D.h"

namespace BRICS_3D {

/**
 * @brief Class to represent a Cartesian 3D point cloud
 */
class PointCloud3D {
public:

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
	 * @brief Get the pointer to the point cloud
	 * @return Pointer to point cloud
	 */
    std::vector<Point3D>* getPointCloud();

	/**
	 * @brief Set the pointer to the point cloud
	 * @param pointCloud Pointer to new point cloud
	 */
    void setPointCloud(std::vector<Point3D>* pointCloud);

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

	///Pointer to vector which represents a Cartesian point cloud
	std::vector<Point3D>* pointCloud;
};

}

#endif /* CARTESIANPOINTCLOUD_H_ */

/* EOF */

