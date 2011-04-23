/*
 * IObjectModelUsingNormals.h
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#ifndef IOBJECTMODELUSINGNORMALS_H_
#define IOBJECTMODELUSINGNORMALS_H_
#include <vector>
//using namespace std;
namespace BRICS_3D {



class IObjectModelUsingNormals {

	typedef double pointNormal[3];
	typedef std::vector<pointNormal> pointCloudNormals;

protected:

	/** \brief The relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal. */
	double normalDistanceWeight;

	/** \brief A pointer to vector of normals. */
	pointCloudNormals *normals;

public:
	IObjectModelUsingNormals(){};
	virtual ~IObjectModelUsingNormals(){};


    /** \brief Set the normal angular distance weight.
      * \param w the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point
      * normals and the plane normal.  (The Euclidean distance will have weight 1-w.)
      */
    inline void
      setNormalDistanceWeight (double w)
    {
      normalDistanceWeight = w;
    }


    /** \brief Get the normal angular distance weight. */
    inline double
      getNormalDistanceWeight ()
    {
      return (normalDistanceWeight);
    }


    /** \brief Provide a pointer to the input dataset that contains the point normals of the XYZ dataset.
      * \param normals the const boost shared pointer to a PointCloud message
      */
    inline void
      setInputNormals (pointCloudNormals *normals)
    {
      this->normals = normals;
    }


    /** \brief Get a pointer to the normals of the input XYZ point cloud dataset. */
    inline pointCloudNormals*
    getInputNormals ()
    {
      return (this->normals);
    }



};

}

#endif /* IOBJECTMODELUSINGNORMALS_H_ */
