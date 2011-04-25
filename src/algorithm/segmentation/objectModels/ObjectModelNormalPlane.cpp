/*
 * ObjectModelNormalPlane.cpp
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#include "ObjectModelNormalPlane.h"

namespace BRICS_3D {

void ObjectModelNormalPlane::computeRandomModel (int &iterations, Eigen::VectorXf &model_coefficients, bool &isDegenerate,
		bool &modelFound){

	std::vector<int> samples;
	std::vector<int> selection;
	getSamples(iterations,selection);

	if (selection.size () == 0){
		isDegenerate = false;
		modelFound = false;
		return;
	} else {

		isDegenerate = true;

	}

	if (!computeModelCoefficients (selection, model_coefficients)){
		modelFound = false;
		return;
	} else {
		modelFound = true;
		return;
	}
}

void ObjectModelNormalPlane::getSamples(int &iterations, std::vector<int> &samples){
	points = inputPointCloud->getPointCloud();
	// We're assuming that indices_ have already been set in the constructor
	//ToDo ROS_ASSERT (this->indices_->size () != 0);

	samples.resize (3);
	double trand = inputPointCloud->getSize() / (RAND_MAX + 1.0);

	// Get a random number between 1 and max_indices
	int idx = (int)(rand () * trand);
	// Get the index
	samples[0] = idx;

	// Get a second point which is different than the first
	do
	{
		idx = (int)(rand () * trand);
		samples[1] = idx;
		iterations++;
	} while (samples[1] == samples[0]);
	iterations--;
	// Get the values at the two points
	Eigen::Vector4f p0, p1, p2;
	// SSE friendly data check
	p1 = Eigen::Vector4f (this->points->data()[samples[1]].getX(), this->points->data()[samples[1]].getY(), this->points->data()[samples[1]].getZ(), 0);
	p0 = Eigen::Vector4f (this->points->data()[samples[0]].getX(), this->points->data()[samples[0]].getY(), this->points->data()[samples[0]].getZ(), 0);

	// Compute the segment values (in 3d) between p1 and p0
	p1 -= p0;

	Eigen::Vector4f dy1dy2;
	int iter = 0;
	do
	{
		// Get the third point, different from the first two
		do
		{
			idx = (int)(rand () * trand);
			samples[2] = idx;
			iterations++;
		} while ( (samples[2] == samples[1]) || (samples[2] == samples[0]) );
		iterations--;

		// SSE friendly data check
		p2 = Eigen::Vector4f (this->points->data()[samples[2]].getX(), this->points->data()[samples[2]].getY(), this->points->data()[samples[2]].getZ(), 0);

		// Compute the segment values (in 3d) between p2 and p0
		p2 -= p0;

		dy1dy2 = p1.cwise () / p2;
		++iter;
		if (iter > MAX_ITERATIONS_COLLINEAR )
		{
			//ROS_WARN ("[pcl::SampleConsensusModelPlane::getSamples] WARNING: Could not select 3 non collinear points in %d iterations!", MAX_ITERATIONS_COLLINEAR);
			cout<< "cannot find collinear points"<<endl;
			break;
		}
		iterations++;
	}
	while ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) );
	iterations--;

}

bool ObjectModelNormalPlane::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients){
	// Need 3 samples
	//ToDo ROS_ASSERT (samples.size () == 3);

	Eigen::Vector4f p0, p1, p2;
	// SSE friendly data check
	p0 = Eigen::Vector4f (this->points->data()[samples[0]].getX(), this->points->data()[samples[0]].getY(), this->points->data()[samples[0]].getZ(), 0);
	p1 = Eigen::Vector4f (this->points->data()[samples[1]].getX(), this->points->data()[samples[1]].getY(), this->points->data()[samples[1]].getZ(), 0);
	p2 = Eigen::Vector4f (this->points->data()[samples[2]].getX(), this->points->data()[samples[2]].getY(), this->points->data()[samples[2]].getZ(), 0);

	// Compute the segment values (in 3d) between p1 and p0
	p1 -= p0;
	// Compute the segment values (in 3d) between p2 and p0
	p2 -= p0;

	// Avoid some crashes by checking for collinearity here
	Eigen::Vector4f dy1dy2 = p1.cwise () / p2;
	if ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) )          // Check for collinearity
		return (false);

	// Compute the plane coefficients from the 3 given points in a straightforward manner
	// calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
	model_coefficients.resize (4);
	model_coefficients[0] = p1[1] * p2[2] - p1[2] * p2[1];
	model_coefficients[1] = p1[2] * p2[0] - p1[0] * p2[2];
	model_coefficients[2] = p1[0] * p2[1] - p1[1] * p2[0];
	model_coefficients[3] = 0;

	// Normalize
	model_coefficients.normalize ();

	// ... + d = 0
	//Todo original: model_coefficients[3] = -1 * (model_coefficients.template start<4>().dot (p0));
	model_coefficients[3] = -1 * (model_coefficients.dot (p0));

	return (true);
}

void ObjectModelNormalPlane::optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
		Eigen::VectorXf &optimized_coefficients){
	// Needs a valid set of model coefficients
	//ToDo Check for this ROS_ASSERT (model_coefficients.size () == 4);

	if (inliers.size () == 0)
	{
		//ToDo ROS_ERROR ("[pcl::SampleConsensusModelPlane::optimizeModelCoefficients] Inliers vector empty! Returning the same coefficients.");
		optimized_coefficients = model_coefficients;
		return;
	}

	// Need at least 3 points to estimate a plane
	if (inliers.size () < 4)
	{
		//ToDo ROS_ERROR ("[pcl::SampleConsensusModelPlane::optimizeModelCoefficients] Not enough inliers found to support a model! Returning the same coefficients.");
		optimized_coefficients = model_coefficients;
		return;
	}

	Eigen::Vector4f plane_parameters;
	float curvature;

	// Use Least-Squares to fit the plane through all the given sample points and find out its coefficients

	computePointNormal (inputPointCloud, inliers, plane_parameters, curvature);
	optimized_coefficients = plane_parameters;
}


/** \brief Compute all distances from the inliers of the plane model to a given plane model.
 * \param estimated inliers to the model
 * \param model_coefficients the coefficients of a plane model that we need to compute distances to
 * \param distances the resultant estimated distances
 */

void
ObjectModelNormalPlane::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,  std::vector<double> &distances) {
	// Needs a valid model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 4);

	distances.resize (inliers.size());

	for (size_t i = 0; i < inliers.size (); ++i)
	{
		// Calculate the distance from the point to the plane normal as the dot product
		// D = (P-A).N/|N|
		distances[i]=fabs (model_coefficients[0] * this->points->data()[inliers[i]].getX() +
				model_coefficients[1] * this->points->data()[inliers[i]].getY() +
				model_coefficients[2] * this->points->data()[inliers[i]].getZ() +
				model_coefficients[3]);
	}
}

/** \brief Create a new point cloud with inliers projected onto the plane model.
 * \param inliers the data inliers that we want to project on the plane model
 * \param model_coefficients the *normalized* coefficients of a plane model
 * \param projected_points the resultant projected points
 * \param copy_data_fields set to true if we need to copy the other data fields
 */
void
ObjectModelNormalPlane:: projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
		PointCloud3D* projectedPointCloud)
{
	// Needs a valid set of model coefficients
	//Todo ROS_ASSERT (model_coefficients.size () == 4);

	std::vector<Point3D> *projectedPoints;
	projectedPoints = projectedPointCloud->getPointCloud();


	Eigen::Vector4f mc = Eigen::Vector4f (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);

	// Iterate through the 3d points and calculate the distances from them to the plane
	for (size_t i = 0; i < inliers.size (); ++i)
	{
		// Calculate the distance from the point to the plane
		float distance_to_plane = model_coefficients[0] * this->points->data()[inliers[i]].getX() +
				model_coefficients[1] * this->points->data()[inliers[i]].getY() +
				model_coefficients[2] * this->points->data()[inliers[i]].getZ() +
				model_coefficients[3];
		// Calculate the projection of the point on the plane
		Point3D point;
		projectedPoints->data()[inliers[i]].setX(this->points->data()[inliers[i]].getX() - distance_to_plane * model_coefficients[0]);
		projectedPoints->data()[inliers[i]].setY( this->points->data()[inliers[i]].getY() - distance_to_plane * model_coefficients[1]);
		projectedPoints->data()[inliers[i]].setZ( this->points->data()[inliers[i]].getZ() - distance_to_plane * model_coefficients[2]);

	}
}

/** \brief Verify whether a subset of indices verifies the given plane model coefficients.
 * \param indices the data indices that need to be tested against the plane model
 * \param model_coefficients the plane model coefficients
 * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
 */
bool
ObjectModelNormalPlane::doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold)
{
	// Needs a valid set of model coefficients
	//Todo ROS_ASSERT (model_coefficients.size () == 4);

	for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
		if (fabs (model_coefficients[0] * this->points->data()[*it].getX() +
				model_coefficients[1] * this->points->data()[*it].getY() +
				model_coefficients[2] * this->points->data()[*it].getZ() +
				model_coefficients[3]) > threshold)
			return (false);

	return (true);
}


void
  ObjectModelNormalPlane::selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers){

    // Needs a valid set of model coefficients
    //ToDo ROS_ASSERT (model_coefficients.size () == 4);
    if (!this->normals && this->normals->size()!=this->inputPointCloud->getSize())
    {
      cout<<"[ObjectModelNormalPlane::getDistancesToModel] No input dataset containing normals was given!";
      return;
    }

    Eigen::Vector4f coeff = model_coefficients;
    coeff[3] = 0;

    // Check against template, if given
    if (epsAngle > 0.0)
    {
      double angle_diff = fabs (getAngle3D (axis, coeff));
      angle_diff = fmin (angle_diff, M_PI - angle_diff);
      if (angle_diff > epsAngle)
      {
        inliers.resize (0);
        return;
      }
    }
    if (epsDistance > 0.0)
    {
      if (fabs (-model_coefficients[3] - distanceFromOrigin) > epsDistance)
      {
        inliers.resize (0);
        return;
      }
    }

    int nr_p = 0;
    inliers.resize (this->inputPointCloud->getSize());
    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
    {
      // Calculate the distance from the point to the plane normal as the dot product
      // D = (P-A).N/|N|
      Eigen::Vector4f p = Eigen::Vector4f (this->points->data()[i].getX(),
    		  this->points->data()[i].getY(), this->points->data()[i].getZ(), 0);

      Eigen::Vector4f n = Eigen::Vector4f (this->normals->data()[i][0],
    		  this->normals->data()[i][1], this->normals->data()[i][2], 0);

      double d_euclid = fabs (coeff.dot (p) + model_coefficients[3]);

      // Calculate the angular distance between the point normal and the plane normal
      double d_normal = fabs (getAngle3D (n, coeff));
      d_normal = fmin (d_normal, M_PI - d_normal);

      if (fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight) * d_euclid)
    		  < threshold)
      {
        // Returns the indices of the points whose distances are smaller than the threshold
        inliers[nr_p] = i;
        nr_p++;
      }
    }
    inliers.resize (nr_p);

}

void
  ObjectModelNormalPlane::getDistancesToModel (const Eigen::VectorXf &model_coefficients,
		  std::vector<double> &distances){

    // Needs a valid set of model coefficients
    //ToDo ROS_ASSERT (model_coefficients.size () == 4);

    if (!this->normals && this->normals->size()!=this->inputPointCloud->getSize())
    {
      cout<<"[ObjectModelNormalPlane::getDistancesToModel] No input dataset containing normals was given!";
      return;
    }

    distances.resize (this->inputPointCloud->getSize());

    Eigen::Vector4f coeff = model_coefficients;
    coeff[3] = 0;

    // Check against template, if given
    if (epsAngle > 0.0)
    {
      double angle_diff = fabs (getAngle3D (axis, coeff));
      angle_diff = fmin (angle_diff, M_PI - angle_diff);
      if (angle_diff > epsAngle)
      {
        distances.resize (0);
        return;
      }
    }
    if (epsDistance > 0.0)
    {
      if (fabs (-model_coefficients[3] - distanceFromOrigin) > epsDistance)
      {
        distances.resize (0);
        return;
      }
    }

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
    {
      // Calculate the distance from the point to the plane normal as the dot product
      // D = (P-A).N/|N|
      Eigen::Vector4f p = Eigen::Vector4f (this->points->data()[i].getX(),
    		  this->points->data()[i].getY(), this->points->data()[i].getZ(), 0);

      Eigen::Vector4f n = Eigen::Vector4f (this->normals->data()[i][0],
    		  this->normals->data()[i][1], this->normals->data()[i][2], 0);
      double d_euclid = fabs (coeff.dot (p) + model_coefficients[3]);

      // Calculate the angular distance between the point normal and the plane normal
      double d_normal = fabs (getAngle3D (n, coeff));
      d_normal = fmin (d_normal, M_PI - d_normal);

      distances[i] = fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight) * d_euclid);
    }

}


}
