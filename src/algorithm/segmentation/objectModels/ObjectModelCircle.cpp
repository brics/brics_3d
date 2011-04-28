/*
 * ObjectModelCircle.cpp
 *
 *  Created on: Apr 22, 2011
 *      Author: reon
 */

#include "ObjectModelCircle.h"


namespace BRICS_3D {


void ObjectModelCircle::computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate,
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

void
ObjectModelCircle::getSamples (int &iterations, std::vector<int> &samples)
{
	// We're assuming that indices_ have already been set in the constructor
	//ToDo ROS_ASSERT (this->indices_->size () != 0);

	samples.resize (3);
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

	this->points = inputPointCloud->getPointCloud();

	// Get the values at the two points
	Eigen::Vector2d p0 = Eigen::Vector2d (this->points->data()[samples[0]].getX(), this->points->data()[samples[0]].getY());
	Eigen::Vector2d p1 = Eigen::Vector2d (this->points->data()[samples[1]].getX(), this->points->data()[samples[1]].getY());

	// Compute the segment values (in 2d) between p1 and p0
	p1 -= p0;

	Eigen::Vector2d dy1dy2;
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

		Eigen::Vector2d p2 = Eigen::Vector2d (this->points->data()[samples[2]].getX(), this->points->data()[samples[2]].getY());

		// Compute the segment values (in 2d) between p2 and p0
		p2 -= p0;

		dy1dy2 = p1.cwise () / p2;
		++iter;
		if (iter > MAX_ITERATIONS_COLLINEAR )
		{
			cout<<"[SampleConsensusModelCircle::getSamples] WARNING: Could not select 3 non collinear points in"<< MAX_ITERATIONS_COLLINEAR <<" iterations!";
			break;
		}
		iterations++;
	}
	while ( (dy1dy2[0] == dy1dy2[1]) );
	iterations--;
}

bool
ObjectModelCircle::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients)
{
	// Need 3 samples
	//ToDo ROS_ASSERT (samples.size () == 3);

	model_coefficients.resize (3);

	Eigen::Vector2d p0 (this->points->data()[samples[0]].getX(), this->points->data()[samples[0]].getY());
	Eigen::Vector2d p1 (this->points->data()[samples[1]].getX(), this->points->data()[samples[1]].getY());
	Eigen::Vector2d p2 (this->points->data()[samples[2]].getX(), this->points->data()[samples[2]].getY());

	Eigen::Vector2d u = (p0 + p1) / 2.0;
	Eigen::Vector2d v = (p1 + p2) / 2.0;

	Eigen::Vector2d p1p0dif = p1 - p0;
	Eigen::Vector2d p2p1dif = p2 - p1;
	Eigen::Vector2d uvdif   = u - v;

	Eigen::Vector2d m = Eigen::Vector2d (- p1p0dif[0] / p1p0dif[1], - p2p1dif[0] / p2p1dif[1]);

	// Center (x, y)
	model_coefficients[0] = (m[0] * u[0] -  m[1] * v[0]  - uvdif[1] )             / (m[0] - m[1]);
	model_coefficients[1] = (m[0] * m[1] * uvdif[0] +  m[0] * v[1] - m[1] * u[1]) / (m[0] - m[1]);

	// Radius
	model_coefficients[2] = sqrt ((model_coefficients[0] - p0[0]) * (model_coefficients[0] - p0[0]) +
			(model_coefficients[1] - p0[1]) * (model_coefficients[1] - p0[1]));
	return (true);
}

/** \brief Compute all distances from the cloud data to a given 2D circle model.
 * \param model_coefficients the coefficients of a 2D circle model that we need to compute distances to
 * \param distances the resultant estimated distances
 */
void
ObjectModelCircle:: getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances)
{
	// Needs a valid model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 3);

	distances.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the circle
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
		// Calculate the distance from the point to the circle as the difference between
		// dist(point,circle_origin) and circle_radius
		distances[i] = fabs (sqrt (
				( this->points->data()[i].getX() - model_coefficients[0] ) *
				( this->points->data()[i].getX() - model_coefficients[0] ) +

				( this->points->data()[i].getY() - model_coefficients[1] ) *
				( this->points->data()[i].getY() - model_coefficients[1] )
		) - model_coefficients[2]);
}


void
ObjectModelCircle::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
		std::vector<double> &distances) {

	// Needs a valid set of model coefficients
	//Todo ROS_ASSERT (model_coefficients.size () == 3);

	distances.resize (inliers.size());

	for (size_t i = 0; i < inliers.size (); ++i)
	{
		// Calculate the distance from the point to the circle as the difference between
		// dist(point,circle_origin) and circle_radius
		distances[i]=fabs (sqrt (
				( this->points->data()[inliers[i]].getX() - model_coefficients[0] ) *
				( this->points->data()[inliers[i]].getX() - model_coefficients[0] ) +

				( this->points->data()[inliers[i]].getY() - model_coefficients[1] ) *
				( this->points->data()[inliers[i]].getY() - model_coefficients[1] )
		) - model_coefficients[2]);
	}
}

void
ObjectModelCircle::selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold, std::vector<int> &inliers)
{
	// Needs a valid model coefficients
	//Todo ROS_ASSERT (model_coefficients.size () == 3);

	int nr_p = 0;
	inliers.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the sphere as the difference between
		// dist(point,sphere_origin) and sphere_radius
		float distance = fabs (sqrt (
				( this->points->data()[i].getX() - model_coefficients[0] ) *
				( this->points->data()[i].getX() - model_coefficients[0] ) +

				( this->points->data()[i].getY() - model_coefficients[1] ) *
				( this->points->data()[i].getY() - model_coefficients[1] )
		) - model_coefficients[2]);
		if (distance < threshold)
		{
			// Returns the indices of the points whose distances are smaller than the threshold
			inliers[nr_p] = i;
			nr_p++;
		}
	}
	inliers.resize (nr_p);
}

void
ObjectModelCircle::optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
		Eigen::VectorXd &optimized_coefficients)
{

	//ToDo Requires cminpack. currently not supported
	cout<< "INFO: Currently model coe-fficient optimization is not supported for ObjectModelCircle"<<endl;
	optimized_coefficients = model_coefficients;
	return;


	/*  boost::mutex::scoped_lock lock (tmp_mutex_);

  int n_unknowns = 3;      // 3 unknowns
  // Needs a set of valid model coefficients
 //ToDo ROS_ASSERT (model_coefficients.size () == n_unknowns);

  if (inliers.size () == 0)
  {
    cout<<"[SampleConsensusModelCircle::optimizeModelCoefficients] Inliers vector empty! Returning the same coefficients.";
    optimized_coefficients = model_coefficients;
    return;
  }

  // Need at least 3 samples
  //ToDo ROS_ASSERT (inliers.size () > 3);
  if (inliers.size () < 3)
    {
      cout<<"[SampleConsensusModelCircle::optimizeModelCoefficients] Inliers vector empty! Returning the same coefficients.";
      optimized_coefficients = model_coefficients;
      return;
    }

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
  int info = lmdif1 (&pcl::SampleConsensusModelCircle2D<PointT>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);

  // Compute the L2 norm of the residuals
  //ROS_DEBUG ("[pcl::SampleConsensusModelCircle2D::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g \nFinal solution: %g %g %g",
    //         info, enorm (m, fvec), model_coefficients[0], model_coefficients[1], model_coefficients[2], x[0], x[1], x[2]);

  optimized_coefficients = Eigen::Vector3f (x[0], x[1], x[2]);

  free (wa); free (fvec);
	 */
}

void
ObjectModelCircle::projectPoints (const std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
		PointCloud3D *projectedPointCloud)
{
	// Needs a valid set of model coefficients
	//Todo ROS_ASSERT (model_coefficients.size () == 3);
	std::vector<Point3D> *projectedPoints = projectedPointCloud->getPointCloud();
	// Iterate through the 3d points and calculate the distances from them to the circle
	for (size_t i = 0; i < inliers.size (); ++i)
	{
		float dx = this->points->data()[inliers[i]].getX() - model_coefficients[0];
		float dy = this->points->data()[inliers[i]].getY() - model_coefficients[1];
		float a = sqrt ( (model_coefficients[2] * model_coefficients[2]) / (dx * dx + dy * dy) );

		projectedPoints->data()[i].setX(a * dx + model_coefficients[0]);
		projectedPoints->data()[i].setY(a * dy + model_coefficients[1]);
	}
}

bool
ObjectModelCircle:: doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients, double threshold)
{
	// Needs a valid model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 3);

	for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
		// Calculate the distance from the point to the sphere as the difference between
		//dist(point,sphere_origin) and sphere_radius
		if (fabs (sqrt (
				( this->points->data()[*it].getX() - model_coefficients[0] ) *
				( this->points->data()[*it].getX() - model_coefficients[0] ) +
				( this->points->data()[*it].getY() - model_coefficients[1] ) *
				( this->points->data()[*it].getY() - model_coefficients[1] )
		) - model_coefficients[2]) > threshold)
			return (false);

	return (true);
}


}

