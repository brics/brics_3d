/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
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
//ToDo Replace using concrete classes in algoritm/featureExtraction
#ifndef NORMALESTIMATION_H_
#define NORMALESTIMATION_H_

#include "core/PointCloud3D.h"
#include "algorithm/segmentation/features/ROSnormal3D.h"
#include "core/NormalSet3D.h"

#include "algorithm/nearestNeighbor/INearestPoint3DNeighbor.h"

namespace brics_3d {

class NormalEstimation {

private:

	/** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
	 * from NormalEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
	double vpx, vpy, vpz;

	/** \brief Number of k-nearest neighbours to be used */
	int k_neighbours;

	/** \brief Represents the point-cloud to be processed*/
	PointCloud3D* inputPointCloud;

	/** \brief Nearest neighbour searching method */
	INearestPoint3DNeighbor *nnSearchMethod;
public:
	NormalEstimation(){
		this->vpx = 0;
		this->vpy = 0;
		this->vpz = 0;
		this->k_neighbours = 10;

	}

	~NormalEstimation(){};


	/** \brief Get the nearst neighborhood search method. */
	inline INearestPoint3DNeighbor*
	getNNSearchMethod ()
	{
		return (this->nnSearchMethod);
	}


	/** \brief Set the nearst neighborhood search method. */
	inline void
	setSearchMethod (INearestPoint3DNeighbor *nnSearchMethod)
	{	assert(this->inputPointCloud!=NULL);
		this->nnSearchMethod = nnSearchMethod;
		this->nnSearchMethod->setData(this->inputPointCloud);
	}


	/** \brief Provide a pointer to the input dataset
	 * \param cloud the const boost shared pointer to a PointCloud message
	 */
	inline void
	setInputCloud (PointCloud3D* cloud)
	{
		this->inputPointCloud = cloud;
	}

	/** \brief Get a pointer to the input point cloud dataset. */
	inline PointCloud3D* getInputCloud () { return (inputPointCloud); }


	/** \brief Set the number of nearest neighbours to be used
	 * \param k_neighbours: The number of neighbours to be used
	 */
	inline void setkneighbours(int kNeighbours){
		this->k_neighbours = kNeighbours;
	}


	/** \brief Get the number of nearest neighbours to be used
	 */
	inline int getkneighbours(){
		return this->k_neighbours;
	}

	/** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and return it as a 3D vector.
	 * \param cloud the input point cloud
	 * \param indices the point cloud indices that need to be used
	 * \param centroid the output centroid
	 */
	inline void
	compute4DCentroid (PointCloud3D *cloud,Eigen::Vector4d &centroid)
	{
		// Initialize to 0
		centroid.setZero ();

		// For each point in the cloud
		int cp = 0;
		for (size_t i = 0; i < cloud->getSize(); ++i)
		{
			// Check if the point is invalid
			if (isnan ((*cloud->getPointCloud())[i].getX()) || isnan ((*cloud->getPointCloud())[i].getY())
					|| isnan ((*cloud->getPointCloud())[i].getZ()))
				continue;

			double x = (*cloud->getPointCloud())[i].getX();
			centroid += Eigen::Vector4d::MapAligned (&x);
			cp++;
		}
		centroid /= cp;
	}


	/** \brief Set the viewpoint.
	 * \param vpx the X coordinate of the viewpoint
	 * \param vpy the Y coordinate of the viewpoint
	 * \param vpz the Z coordinate of the viewpoint
	 */
	inline void
	setViewPoint (double vpx, double vpy, double vpz)
	{
		this->vpx = vpx;
		this->vpy = vpy;
		this->vpz = vpz;
	}


	/** \brief Get the viewpoint. */
	inline void
	getViewPoint (double &vpx, double &vpy, double &vpz)
	{
		vpx = this->vpx;
		vpy = this->vpy;
		vpz = this->vpz;
	}



	/** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
	 * setSearchSurface () and the spatial locator in setSearchMethod ()
	 * \note In situations where not enough neighbors are found, the normal and curvature values are set to -1.
	 * \param output the resultant point cloud model dataset that contains surface normals and curvatures
	 */
	void
	computeFeature (NormalSet3D *normalSet)
	{
		// Allocate enough space to hold the results
		// \note This resize is irrelevant for a radiusSearch ().
		//using the default number of nearrest-neighbours used.
		//ToDo enable configuration of 'k'
		std::vector<int> nn_indices;
//		nn_indices.resize(k_neighbours);
		double curvature;
		// Iterating over the entire index vector
		nnSearchMethod->setData(this->inputPointCloud);

//		std::vector<Point3D>* points;
//		points = inputPointCloud->getPointCloud();

		for (size_t idx = 0; idx < this->inputPointCloud->getSize(); ++idx)
		{

			nnSearchMethod->findNearestNeighbors(&(*inputPointCloud->getPointCloud())[idx], &nn_indices, k_neighbours);

			if (nn_indices.size()==0)
			{
				normalSet->getNormals()->data()[idx].setX(std::numeric_limits<double>::quiet_NaN ());
				normalSet->getNormals()->data()[idx].setY(std::numeric_limits<double>::quiet_NaN ());
				normalSet->getNormals()->data()[idx].setZ(std::numeric_limits<double>::quiet_NaN ());
				curvature = 0;
				continue;
			}

			double nx,ny,nz;
			computePointNormal (this->inputPointCloud, nn_indices,
					nx,ny,nz,curvature);

			Normal3D tempNormal;
			tempNormal.setX(nx);
			tempNormal.setY(ny);
			tempNormal.setZ(nz);

			normalSet->addNormal(tempNormal);
			flipNormalTowardsViewpoint ((*this->inputPointCloud->getPointCloud())[idx], vpx, vpy, vpz,
					normalSet->getNormals()->data()[idx]);

		}
	}

	  /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
	    * \param point a given point
	    * \param vp_x the X coordinate of the viewpoint
	    * \param vp_y the X coordinate of the viewpoint
	    * \param vp_z the X coordinate of the viewpoint
	    * \param normal the plane normal to be flipped
	    */
	inline void
	    flipNormalTowardsViewpoint (Point3D &point, double vp_x, double vp_y, double vp_z, Normal3D &normal)
	  {
	    // See if we need to flip any plane normals
	    vp_x -= point.getX();
	    vp_y -= point.getY();
	    vp_z -= point.getZ();

	    // Dot product between the (viewpoint - point) and the plane normal
	    double cos_theta = (vp_x * normal.getX() + vp_y * normal.getY() + vp_z * normal.getZ());

	    // Flip the plane normal
	    if (cos_theta < 0)
	    {
	      normal.setX(-1*normal.getX());
	      normal.setY(-1*normal.getY());
	      normal.setZ(-1*normal.getZ());
	      // Hessian form (D = nc . p_plane (centroid here) + p)
	      normal.setZ(-1 * (
	    		  normal.getX() * point.getX() +
	    		  normal.getY() * point.getY() +
	    		  normal.getZ() * point.getZ()));
	    }
	  }

};

}

#endif /* NORMALESTIMATION_H_ */
