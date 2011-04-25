/*
 * IObjectModelUsingNormals.h
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#ifndef IOBJECTMODELUSINGNORMALS_H_
#define IOBJECTMODELUSINGNORMALS_H_
#include <vector>
#include "algorithm/segmentation/objectModels/IObjectModel.h"
//using namespace std;
namespace BRICS_3D {



class IObjectModelUsingNormals : public IObjectModel{

	typedef double pointNormal[3];
	typedef std::vector<pointNormal> pointCloudNormals;

protected:

	/** \brief The relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal. */
	double normalDistanceWeight;

	/** \brief A pointer to vector of normals. */
	pointCloudNormals *normals;

	/** \brief Axis along which we need to search for a model perpendicular to */
	Eigen::Vector3f axis;

	/** \brief Set the angle epsilon (delta) threshold */
	double epsAngle;

public:
	IObjectModelUsingNormals(){};
	virtual ~IObjectModelUsingNormals(){};


	/** \brief Set the angle epsilon (delta) threshold.
	 * \param ea the maximum allowed difference between the model normal and the given axis.
	 */
	inline void
	setEpsAngle (double ea)
	{
		this->epsAngle = ea;
	}


	/** \brief Get the epsilon (delta) model angle threshold. */
	inline double
	getEpsAngle ()
	{
		return (this->epsAngle);
	}

	/** \brief Set the axis along which we need to search for a model perpendicular to.
	 * \param ax the axis along which we need to search for a model perpendicular to
	 */
	inline void
	setAxis (const Eigen::Vector3f &ax)
	{
		this->axis = ax;
	}


	/** \brief Get the axis along which we need to search for a model perpendicular to. */
	inline Eigen::Vector3f
	getAxis ()
	{
		return (this->axis);
	}

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
