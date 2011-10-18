/*
 * SimplePointCloudGeneratorCube.h
 *
 *  Created on: Oct 5, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef SIMPLEPOINTCLOUDGENERATORCUBE_H_
#define SIMPLEPOINTCLOUDGENERATORCUBE_H_

#include "ISimplePointCloudGenerator.h"

#include "core/PointCloud3D.h"

namespace BRICS_3D {

class SimplePointCloudGeneratorCube : public ISimplePointCloudGenerator {
private:

	/**
	 * Length of each side in meters
	 */
	double cubeSideLength;

	/**
	 * no of point in each row in a cube-side.
	 * This will define the point-density
	 */
	int pointsOnEachSide;

	/**
	 * Number of faces in the Cube
	 */
	int numOfFaces;

	/**
	 * The centroid of the generated cube
	 */
	double origin[3];



public:


	SimplePointCloudGeneratorCube();


	virtual ~SimplePointCloudGeneratorCube();


	/**
	 * Generates a 3D model of a simple shape
	 * @param generatedPointCloud Pointcloud representing the requested 3D model
	 */
	void generatePointCloud(BRICS_3D::PointCloud3D *generatedPointCloud);


	/**
	 *
	 * @return Length of each size in meters
	 */
	double getCubeSideLength() const;


	/**
	 *
	 * @return  Number of faces in the Cube
	 */
    int getNumOfFaces() const;


    /**
     *
     * @return No of point in each row in a cube-side.
     */
    int getPointsOnEachSide() const;

    /**
     * Gives the centre of the generated cube
     * @param x
     * @param y
     * @param z
     */
     void getOrigin(double *x, double *y, double *z);


    /**
     *
     * @param cubeSideLength Length of each size in meters
     */
    void setCubeSideLength(double cubeSideLength);


    /**
     *
     * @param numOfFaces Number of faces in the Cube
     */
    void setNumOfFaces(int numOfFaces);


    /**
     *
     * @param origin  The centroid of the generated cube
     */
    void setOrigin(double origin[3]);


    /**
     *
     * @param pointsOnEachSide No of point in each row in a cube-side.
     */
    void setPointsOnEachSide(int pointsOnEachSide);




};

}

#endif /* SIMPLEPOINTCLOUDGENERATORCUBE_H_ */
