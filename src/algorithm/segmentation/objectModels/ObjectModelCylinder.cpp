/*
 * ObjectModelCylinder.cpp
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#include "ObjectModelCylinder.h"

namespace BRICS_3D {

void ObjectModelCylinder::computeRandomModel (int &iterations, Eigen::VectorXf &model_coefficients, bool &isDegenerate,
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
ObjectModelCylinder::getSamples (int &iterations, std::vector<int> &samples)
{
	// We're assuming that indices_ have already been set in the constructor
	//ToDo ROS_ASSERT (this->indices_->size () != 0);

	samples.resize (2);
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

	//        std::cerr << samples[0] << " " << samples[1] << std::endl;
}

bool ObjectModelCylinder::computeModelCoefficients (const std::vector<int> &samples,
		Eigen::VectorXf &model_coefficients){
	// Need 2 samples
	//ToDo ROS_ASSERT (samples.size () == 2);

	if (!this->normals && this->normals->size()!=this->inputPointCloud->getSize())
	{
		cout<<"[ObjectModelCylinder::computeModelCoefficients] No input dataset containing normals was given!";
		return (false);
	}

	Eigen::Vector4f p1 = Eigen::Vector4f (this->points->data()[samples[0]].getX(),
			this->points->data()[samples[0]].getY(), this->points->data()[samples[0]].getZ(), 0);

	Eigen::Vector4f p2 = Eigen::Vector4f (this->points->data()[samples[1]].getX(),
			this->points->data()[samples[1]].getY(), this->points->data()[samples[1]].getZ(), 0);

	Eigen::Vector4f n1 = Eigen::Vector4f (this->normals->data()[samples[0]][0],
			this->normals->data()[samples[0]][1], this->normals->data()[samples[0]][2], 0);

	Eigen::Vector4f n2 = Eigen::Vector4f (this->normals->data()[samples[1]][0],
			this->normals->data()[samples[1]][1], this->normals->data()[samples[1]][2], 0);

	Eigen::Vector4f w = n1 + p1 - p2;

	double a = n1.dot (n1);
	double b = n1.dot (n2);
	double c = n2.dot (n2);
	double d = n1.dot (w);
	double e = n2.dot (w);
	double denominator = a*c - b*b;
	double sc, tc;
	// Compute the line parameters of the two closest points
	if (denominator < 1e-8)          // The lines are almost parallel
	{
		sc = 0.0;
		tc = (b > c ? d / b : e / c);  // Use the largest denominator
	}
	else
	{
		sc = (b*e - c*d) / denominator;
		tc = (a*e - b*d) / denominator;
	}

	// point_on_axis, axis_direction
	Eigen::Vector4f line_pt  = p1 + n1 + sc * n1;
	Eigen::Vector4f line_dir = p2 + tc * n2 - line_pt;
	line_dir.normalize ();

	model_coefficients.resize (7);
	model_coefficients.start<3> ()    = line_pt.start<3> ();
	model_coefficients.segment<3> (3) = line_dir.start<3> ();
	// cylinder radius
	model_coefficients[6] = pointToLineDistance (p1, line_pt, line_dir);

	if (model_coefficients[6] > this->radiusMax || model_coefficients[6] < this->radiusMin)
		return (false);

	return (true);
}



void ObjectModelCylinder::optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
		Eigen::VectorXf &optimized_coefficients){

	//ToDo Requires cminpack. currently not supported
	cout<< "INFO: Currently model coe-fficient optimization is not supported for ObjectModelCylinder"<<endl;
	optimized_coefficients = model_coefficients;
	return;

	/*boost::mutex::scoped_lock lock (tmp_mutex_);

     int n_unknowns = 7;      // 4 unknowns
     // Needs a set of valid model coefficients
     ROS_ASSERT (model_coefficients.size () == n_unknowns);

     if (inliers.size () == 0)
     {
       ROS_ERROR ("[pcl::%s::optimizeModelCoefficients] Cannot re-fit 0 inliers!", getName ().c_str ());
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
     int info = lmdif1 (&pcl::SampleConsensusModelCylinder<PointT, PointNT>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);

     // Compute the L2 norm of the residuals
     ROS_DEBUG ("[pcl::%s::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g",
                getName ().c_str (), info, enorm (m, fvec), model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3],
                model_coefficients[4], model_coefficients[5], model_coefficients[6], x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

     optimized_coefficients.resize (n_unknowns);
     for (int d = 0; d < n_unknowns; ++d)
       optimized_coefficients[d] = x[d];

     free (wa); free (fvec);*/
}


void ObjectModelCylinder::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances){

	// Needs a valid model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 7);

	distances.resize (this->inputPointCloud->getSize());

	Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

	double ptdotdir = line_pt.dot (line_dir);

	double dirdotdir = 1.0 / line_dir.dot (line_dir);
	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Aproximate the distance from the point to the cylinder as the difference between
		// dist(point,cylinder_axis) and cylinder radius
		// @note need to revise this.
		Eigen::Vector4f pt = Eigen::Vector4f (this->points->data()[i].getX(),
				this->points->data()[i].getY(), this->points->data()[i].getZ(), 0);

		Eigen::Vector4f n = Eigen::Vector4f (this->normals->data()[i][0],
				this->normals->data()[i][1],
				this->normals->data()[i][2], 0);

		double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

		// Calculate the point's projection on the cylinder axis
		double k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
		Eigen::Vector4f pt_proj = line_pt + k * line_dir;
		Eigen::Vector4f dir = pt - pt_proj;
		dir.normalize ();

		// Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
		double d_normal = fabs (getAngle3D (n, dir));
		d_normal = fmin (d_normal, M_PI - d_normal);

		distances[i] = fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight)
				* d_euclid);
	}
}


void ObjectModelCylinder::selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold,
		std::vector<int> &inliers){
	// Needs a valid model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 7);

	int nr_p = 0;
	inliers.resize (this->inputPointCloud->getSize());

	Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	double ptdotdir = line_pt.dot (line_dir);
	double dirdotdir = 1.0 / line_dir.dot (line_dir);
	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Aproximate the distance from the point to the cylinder as the difference between
		// dist(point,cylinder_axis) and cylinder radius
		Eigen::Vector4f pt = Eigen::Vector4f (this->points->data()[i].getX(),
				this->points->data()[i].getY(),
				this->points->data()[i].getZ(), 0);

		Eigen::Vector4f n = Eigen::Vector4f (this->normals->data()[i][0],
				this->normals->data()[i][1], this->normals->data()[i][2], 0);

		double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

		// Calculate the point's projection on the cylinder axis
		double k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
		Eigen::Vector4f pt_proj = line_pt + k * line_dir;
		Eigen::Vector4f dir = pt - pt_proj;
		dir.normalize ();

		// Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
		double d_normal = fabs (getAngle3D (n, dir));
		d_normal = fmin (d_normal, M_PI - d_normal);

		if (fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight) * d_euclid) < threshold)
		{
			// Returns the indices of the points whose distances are smaller than the threshold
			inliers[nr_p] = i;
			nr_p++;
		}
	}
	inliers.resize (nr_p);
}

void ObjectModelCylinder::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
		std::vector<double> &distances){

	distances.resize (inliers.size());

	Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

	double ptdotdir = line_pt.dot (line_dir);

	double dirdotdir = 1.0 / line_dir.dot (line_dir);
	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < inliers.size(); ++i)
	{
		// Aproximate the distance from the point to the cylinder as the difference between
		// dist(point,cylinder_axis) and cylinder radius
		// @note need to revise this.
		Eigen::Vector4f pt = Eigen::Vector4f (this->points->data()[inliers[i]].getX(),
				this->points->data()[inliers[i]].getY(), this->points->data()[inliers[i]].getZ(), 0);

		Eigen::Vector4f n = Eigen::Vector4f (this->normals->data()[inliers[i]][0],
				this->normals->data()[inliers[i]][1],
				this->normals->data()[inliers[i]][2], 0);

		double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

		// Calculate the point's projection on the cylinder axis
		double k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
		Eigen::Vector4f pt_proj = line_pt + k * line_dir;
		Eigen::Vector4f dir = pt - pt_proj;
		dir.normalize ();

		// Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
		double d_normal = fabs (getAngle3D (n, dir));
		d_normal = fmin (d_normal, M_PI - d_normal);

		distances[i] = fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight)
				* d_euclid);
	}
}


void ObjectModelCylinder::projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
		PointCloud3D* projectedPointCloud){

	// Needs a valid set of model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 7);

    Eigen::Vector4f line_pt = Eigen::Vector4f (model_coefficients[0], model_coefficients[1],
    		model_coefficients[2], 0);
    Eigen::Vector4f line_dir = Eigen::Vector4f (model_coefficients[3], model_coefficients[4],
    		model_coefficients[5], 0);
    double ptdotdir = line_pt.dot (line_dir);
    double dirdotdir = 1.0 / line_dir.dot (line_dir);


	// Iterate through the 3d points and calculate the distances from them to the cylinder
	for (size_t i = 0; i < inliers.size (); ++i)
	{
		Eigen::Vector4f p = Eigen::Vector4f (this->points->data()[inliers[i]].getX(),
				this->points->data()[inliers[i]].getY(), this->points->data()[inliers[i]].getZ(), 0);

		double k = (p.dot (line_dir) - ptdotdir) * dirdotdir;
		Eigen::Vector4f pp = line_pt + k * line_dir;

		Eigen::Vector4f dir = p - pp;
		dir.normalize ();

		// Calculate the projection of the point onto the cylinder
		pp += dir * model_coefficients[6];
		// Calculate the projection of the point on the cylinder
		std::vector<Point3D> *projectedPoints = projectedPointCloud->getPointCloud();
		projectedPoints->data()[inliers[i]].setX(pp[0]);
		projectedPoints->data()[inliers[i]].setY(pp[1]);
		projectedPoints->data()[inliers[i]].setZ(pp[2]);
	}

}


bool ObjectModelCylinder::doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients,
		double threshold){
    // Needs a valid model coefficients
    //ToDo ROS_ASSERT (model_coefficients.size () == 7);

    Eigen::Vector4f pt;
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
    {
      // Aproximate the distance from the point to the cylinder as the difference between
      // dist(point,cylinder_axis) and cylinder radius
      // @note need to revise this.
      pt = Eigen::Vector4f (this->points->data()[*it].getX(), this->points->data()[*it].getY(),
    		  this->points->data()[*it].getZ(), 0);
      if (fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]) > threshold)
        return (false);
    }

    return (true);
}


double
ObjectModelCylinder::pointToLineDistance (const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients)
{
	Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	// Calculate the distance from the point to the line
	// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
	Eigen::Vector4f r, p_t;
	r = line_pt + line_dir;
	p_t = r - pt;

	Eigen::Vector3f c = p_t.start<3> ().cross (line_dir.start<3> ());
	return (sqrt (c.dot (c) / line_dir.dot (line_dir)));
}

double
ObjectModelCylinder::pointToLineDistance (const Eigen::Vector4f &pt, const Eigen::Vector4f &line_pt, const Eigen::Vector4f &line_dir)
{
	// Calculate the distance from the point to the line
	// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
	Eigen::Vector4f r, p_t;
	r = line_pt + line_dir;
	p_t = r - pt;

	Eigen::Vector3f c = p_t.start<3> ().cross (line_dir.start<3> ());
	return (sqrt (c.dot (c) / line_dir.dot (line_dir)));
}



}
