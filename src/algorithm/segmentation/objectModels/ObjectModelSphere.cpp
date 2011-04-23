/*
 * ObjectModelSphere.cpp
 *
 *  Created on: Apr 22, 2011
 *      Author: reon
 */

#include "ObjectModelSphere.h"

namespace BRICS_3D {

/** \brief Get 4 random points (3 non-collinear) as data samples and return them as point indices.
 * \param iterations the internal number of iterations used by SAC methods
 * \param samples the resultant model samples
 * \note assumes unique points!
 * \note Two different points could be enough in theory, to infere some sort of a center and a radius,
 *       but in practice, we might end up with a lot of points which are just 'close' to one another.
 *       Therefore we have two options:
 *       a) use normal information (good but I wouldn't rely on it in extremely noisy point clouds, no matter what)
 *       b) get two more points and uniquely identify a sphere in space (3 unique points define a circle)
 */
void
ObjectModelSphere:: getSamples (int &iterations, std::vector<int> &samples)
{
	// We're assuming that indices_ have already been set in the constructor
	//ToDo ROS_ASSERT (this->indices_->size () != 0);

	samples.resize (4);
	double trand = this->inputPointCloud->getSize() / (RAND_MAX + 1.0);

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
			cout<<"[SampleConsensusModelSphere::getSamples] WARNING: Could not select 3 non collinear points in "<< MAX_ITERATIONS_COLLINEAR<<" iterations!"<<endl;
			break;
		}
		iterations++;
	}
	while ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) );
	iterations--;

	// Need to improve this: we need 4 points, 3 non-collinear always, and the 4th should not be in the same plane as the other 3
	// otherwise we can encounter degenerate cases
	do
	{
		samples[3] = (int)(rand () * trand);
		iterations++;
	} while ( (samples[3] == samples[2]) || (samples[3] == samples[1]) || (samples[3] == samples[0]) );
	iterations--;

}


/** \brief Check whether the given index samples can form a valid sphere model, compute the model coefficients
 * from these samples and store them internally in model_coefficients. The sphere coefficients are: x, y, z, R.
 * \param samples the point indices found as possible good candidates for creating a valid model
 * \param model_coefficients the resultant model coefficients
 */
bool
ObjectModelSphere::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
	// Need 4 samples
	//ToDO ROS_ASSERT (samples.size () == 4);

	Eigen::Matrix4f temp;
	for (int i = 0; i < 4; i++)
	{
		temp (i, 0) = this->points->data()[samples[i]].getX();
		temp (i, 1) = this->points->data()[samples[i]].getY();
		temp (i, 2) = this->points->data()[samples[i]].getZ();
		temp (i, 3) = 1;
	}
	float m11 = temp.determinant ();
	if (m11 == 0)
		return (false);             // the points don't define a sphere!

	for (int i = 0; i < 4; ++i)
		temp (i, 0) = (this->points->data()[samples[i]].getX()) * (this->points->data()[samples[i]].getX()) +
		(this->points->data()[samples[i]].getY()) * (this->points->data()[samples[i]].getY()) +
		(this->points->data()[samples[i]].getZ()) * (this->points->data()[samples[i]].getZ());
	float m12 = temp.determinant ();

	for (int i = 0; i < 4; ++i)
	{
		temp (i, 1) = temp (i, 0);
		temp (i, 0) = this->points->data()[samples[i]].getX();
	}
	float m13 = temp.determinant ();

	for (int i = 0; i < 4; ++i)
	{
		temp (i, 2) = temp (i, 1);
		temp (i, 1) = this->points->data()[samples[i]].getY();
	}
	float m14 = temp.determinant ();

	for (int i = 0; i < 4; ++i)
	{
		temp (i, 0) = temp (i, 2);
		temp (i, 1) = this->points->data()[samples[i]].getX();
		temp (i, 2) = this->points->data()[samples[i]].getY();
		temp (i, 3) = this->points->data()[samples[i]].getZ();
	}
	float m15 = temp.determinant ();

	// Center (x , y, z)
	model_coefficients.resize (4);
	model_coefficients[0] = 0.5 * m12 / m11;
	model_coefficients[1] = 0.5 * m13 / m11;
	model_coefficients[2] = 0.5 * m14 / m11;
	// Radius
	model_coefficients[3] = sqrt (
			model_coefficients[0] * model_coefficients[0] +
			model_coefficients[1] * model_coefficients[1] +
			model_coefficients[2] * model_coefficients[2] - m15 / m11);

	return (true);
}


/** \brief Compute all distances from the cloud data to a given sphere model.
 * \param model_coefficients the coefficients of a sphere model that we need to compute distances to
 * \param distances the resultant estimated distances
 */
void
ObjectModelSphere:: getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
	// Needs a valid model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 4);

	distances.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
		// Calculate the distance from the point to the sphere as the difference between
		//dist(point,sphere_origin) and sphere_radius
		distances[i] = fabs (sqrt (
				( this->points->data()[i].getX() - model_coefficients[0] ) *
				( this->points->data()[i].getX() - model_coefficients[0] ) +

				( this->points->data()[i].getY() - model_coefficients[1] ) *
				( this->points->data()[i].getY() - model_coefficients[1] ) +

				( this->points->data()[i].getZ() - model_coefficients[2] ) *
				( this->points->data()[i].getZ() - model_coefficients[2] )
		) - model_coefficients[3]);
}

void
ObjectModelSphere::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
		std::vector<double> &distances) {

	// Needs a valid model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 4);

	distances.resize (inliers.size());

	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < inliers.size() ; ++i)
		// Calculate the distance from the point to the sphere as the difference between
		//dist(point,sphere_origin) and sphere_radius
		distances[i] = fabs (sqrt (
				( this->points->data()[inliers[i]].getX() - model_coefficients[0] ) *
				( this->points->data()[inliers[i]].getX() - model_coefficients[0] ) +

				( this->points->data()[inliers[i]].getY() - model_coefficients[1] ) *
				( this->points->data()[inliers[i]].getY() - model_coefficients[1] ) +

				( this->points->data()[inliers[i]].getZ() - model_coefficients[2] ) *
				( this->points->data()[inliers[i]].getZ() - model_coefficients[2] )
		) - model_coefficients[3]);
}


/** \brief Select all the points which respect the given model coefficients as inliers.
 * \param model_coefficients the coefficients of a sphere model that we need to compute distances to
 * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
 * \param inliers the resultant model inliers
 */
void
ObjectModelSphere::selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers)
{
	// Needs a valid model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 4);

	int nr_p = 0;
	inliers.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the sphere as the difference between
		// dist(point,sphere_origin) and sphere_radius
		if (fabs (sqrt (
				( this->points->data()[i].getX() - model_coefficients[0] ) *
				( this->points->data()[i].getX() - model_coefficients[0] ) +

				( this->points->data()[i].getY() - model_coefficients[1] ) *
				( this->points->data()[i].getY() - model_coefficients[1] ) +

				( this->points->data()[i].getZ() - model_coefficients[2] ) *
				( this->points->data()[i].getZ() - model_coefficients[2] )
		) - model_coefficients[3]) < threshold)
		{
			// Returns the indices of the points whose distances are smaller than the threshold
			inliers[nr_p] = i;
			nr_p++;
		}
	}
	inliers.resize (nr_p);
}

/** \brief Recompute the sphere coefficients using the given inlier set and return them to the user.
 * @note: these are the coefficients of the sphere model after refinement (eg. after SVD)
 * \param inliers the data inliers found as supporting the model
 * \param model_coefficients the initial guess for the optimization
 * \param optimized_coefficients the resultant recomputed coefficients after non-linear optimization
 */
void
ObjectModelSphere::optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
		Eigen::VectorXf &optimized_coefficients)
{

	//ToDo Requires cminpack. currently not supported
		cout<< "INFO: Currently model coe-fficient optimization is not supported for ObjectModelCircle"<<endl;
		optimized_coefficients = model_coefficients;
		return;


	/*boost::mutex::scoped_lock lock (tmp_mutex_);

	int n_unknowns = 4;      // 4 unknowns
	// Needs a set of valid model coefficients
	ROS_ASSERT (model_coefficients.size () == n_unknowns);

	if (inliers.size () == 0)
	{
		ROS_ERROR ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] Inliers vector empty! Returning the same coefficients.");
		optimized_coefficients = model_coefficients;
		return;
	}

	// Need at least 4 samples
	ROS_ASSERT (inliers.size () > 4);

	tmp_inliers_ = &inliers;

	int m = inliers.size ();

	double *fvec = new double[m];

	int iwa[n_unknowns];

	int lwa = m * n_unknowns + 5 * n_unknowns + m;
	double *wa = new double[lwa];

	// Set the initial solution
	double x[n_unknowns];
	for (int d = 0; d < n_unknowns; ++d)
		x[d] = model_coefficients[d];   // initial guess

	// Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
	double tol = sqrt (dpmpar (1));

	// Optimize using forward-difference approximation LM
	int info = lmdif1 (&pcl::SampleConsensusModelSphere<PointT>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);

	// Compute the L2 norm of the residuals
	ROS_DEBUG ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g \nFinal solution: %g %g %g %g",
			info, enorm (m, fvec), model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3], x[0], x[1], x[2], x[3]);

	optimized_coefficients = Eigen::Vector4f (x[0], x[1], x[2], x[3]);

	free (wa); free (fvec);*/
}

/** \brief Create a new point cloud with inliers projected onto the sphere model.
  * \param inliers the data inliers that we want to project on the sphere model
  * \param model_coefficients the coefficients of a sphere model
  * \param projected_points the resultant projected points
  * \param copy_data_fields set to true if we need to copy the other data fields
  * \todo implement this.
  */
void
 ObjectModelSphere::projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
                 PointCloud3D *projected_points)
{
  cout<<"[SampleConsensusModelSphere::projectPoints] Not implemented yet.";
  projected_points = this->inputPointCloud;
}

/** \brief Verify whether a subset of indices verifies the given sphere model coefficients.
  * \param indices the data indices that need to be tested against the sphere model
  * \param model_coefficients the sphere model coefficients
  * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
  */
bool
  ObjectModelSphere::doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold)
{
  // Needs a valid model coefficients
  //ToDo ROS_ASSERT (model_coefficients.size () == 4);

  for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
    // Calculate the distance from the point to the sphere as the difference between
    //dist(point,sphere_origin) and sphere_radius
    if (fabs (sqrt (
                    ( this->points->data()[*it].getX() - model_coefficients[0] ) *
                    ( this->points->data()[*it].getX() - model_coefficients[0] ) +
                    ( this->points->data()[*it].getY() - model_coefficients[1] ) *
                    ( this->points->data()[*it].getY() - model_coefficients[1] ) +
                    ( this->points->data()[*it].getZ() - model_coefficients[2] ) *
                    ( this->points->data()[*it].getZ() - model_coefficients[2] )
                   ) - model_coefficients[3]) > threshold)
      return (false);

  return (true);
}


}
