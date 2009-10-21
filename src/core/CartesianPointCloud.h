/**
 * @file
 * brics_3dpm 
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

using namespace std;

/**
 * @namespace brics
 */
namespace brics {

/**
 * @struct CartesianPoint3D
 * @brief Simple struct to represent a 3D point in the Cartesian system
 * @author Sebastian Blumenthal
 **/
struct CartesianPoint3D {

	/**
	 * @brief Standard constuctor
	 */
	CartesianPoint3D() {};

	/**
	 * @brief Create and initialize a 3D point
	 * @param xParam X coordinate in Cartesian system
	 * @param yParam Y coordinate in Cartesian system
	 * @param zParam Z coordinate in Cartesian system (height)
	 */
	CartesianPoint3D(double xParam, double yParam, double zParam) {
		x = xParam;
		y = yParam;
		z = zParam;
	};

	/**
	 * @brief Copy constructor
	 */
	CartesianPoint3D(CartesianPoint3D* point) {
		x = point->x;
		y = point->y;
		z = point->z;
	};

	/**
	 * @brief Copy raw data to input buffer (array)
	 * @param[in] pointBuffer Pointer to buffer, where to store the raw data
	 */
	void getRawData(double* pointBuffer) {
		pointBuffer[0] = x;
		pointBuffer[1] = y;
		pointBuffer[2] = z;
	};

	/// X coordinate in Cartesian system
	double x;
	/// Y coordinate in Cartesian system
	double y;
	/// Z coordinate in Cartesian system (heigth)
	double z;
};

/**
 * @class CartesianPointCloud
 * @brief Class to represent a Cartesian 3D point cloud
 * @author Sebastian Blumenthal
 * @date Aug 25, 2009
 */
class CartesianPointCloud {
public:

	/**
	 * @brief Standard constuctor
	 */
	CartesianPointCloud();

	/**
	 * @brief Standard destructor
	 */
	virtual ~CartesianPointCloud();

	/**
	 * @brief Add a point to the poin cloud
	 * @param point Point that will be added
	 */
	void addPoint(Point3D point);

	/**
	 * @brief Get the reference to the point cloud
	 * @return Pointer to point cloud
	 */
    vector<Point3D> *getPointCloud();

	/**
	 * @brief Set the reference to the point cloud
	 * @param pointCloud Pointer to new point cloud
	 */
    void setPointCloud(vector<Point3D> *pointCloud);

    /**
     * @brief Get the number of points in the point cloud
     * @return Size of point cloud (= number of stored points)
     */
    int getSize();

    /**
     * @brief Strores the point cloud into a ply file (Stanford polygon file format)
     * @param filename Specifiess the name of the file .e.g. point_cloud.ply
     */
    void storeToPlyFile(string filename);

    /**
     * @brief Strores the point cloud into a sipmle text file (x y z)
     * @param filename Specifiess the name of the file .e.g. point_cloud.txt
     */
    void storeToTxtFile(string filename);

private:

	///Pointer to vector which represents a Cartesian point cloud
	vector<Point3D> *pointCloud;
};

}

#endif /* CARTESIANPOINTCLOUD_H_ */

/* EOF */

