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

#include "ObjectModelCylinder.h"

namespace BRICS_3D {

void ObjectModelCylinder::computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate,
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

	assert(this->inputPointCloud!=NULL);

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
		Eigen::VectorXd &model_coefficients){

	assert (samples.size () == 2);

	if (!this->normals && this->normals->getSize()!=this->inputPointCloud->getSize())
	{
		cout<<"[ObjectModelCylinder::computeModelCoefficients] No input dataset containing normals was given!";
		return (false);
	}

	Eigen::Vector4d p1 = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[samples[0]].getX(),
			(*inputPointCloud->getPointCloud())[samples[0]].getY(), (*inputPointCloud->getPointCloud())[samples[0]].getZ(), 0);

	Eigen::Vector4d p2 = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[samples[1]].getX(),
			(*inputPointCloud->getPointCloud())[samples[1]].getY(), (*inputPointCloud->getPointCloud())[samples[1]].getZ(), 0);

	Eigen::Vector4d n1 = Eigen::Vector4d (this->normals->getNormals()->data()[samples[0]].getX(),
			this->normals->getNormals()->data()[samples[0]].getY(),
			this->normals->getNormals()->data()[samples[0]].getZ(), 0);

	Eigen::Vector4d n2 = Eigen::Vector4d (this->normals->getNormals()->data()[samples[1]].getX(),
			this->normals->getNormals()->data()[samples[1]].getY(),
			this->normals->getNormals()->data()[samples[1]].getZ(), 0);

	Eigen::Vector4d w = n1 + p1 - p2;

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
	Eigen::Vector4d line_pt  = p1 + n1 + sc * n1;
	Eigen::Vector4d line_dir = p2 + tc * n2 - line_pt;
	line_dir.normalize ();

	model_coefficients.resize (7);
#ifdef EIGEN3
	model_coefficients.head<3> ()    = line_pt.head<3> ();
	model_coefficients.segment<3> (3) = line_dir.head<3> ();
#else
	model_coefficients.start<3> ()    = line_pt.start<3> ();
	model_coefficients.segment<3> (3) = line_dir.start<3> ();
#endif

	// cylinder radius
	model_coefficients[6] = pointToLineDistance (p1, line_pt, line_dir);

	if (model_coefficients[6] > this->radiusMax || model_coefficients[6] < this->radiusMin)
		return (false);

	return (true);
}




void ObjectModelCylinder::getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances){

	assert (model_coefficients.size () == 7);

	distances.resize (this->inputPointCloud->getSize());

	Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

	double ptdotdir = line_pt.dot (line_dir);

	double dirdotdir = 1.0 / line_dir.dot (line_dir);
	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Aproximate the distance from the point to the cylinder as the difference between
		// dist(point,cylinder_axis) and cylinder radius
		// Todo to be revised
		Eigen::Vector4d pt = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[i].getX(),
				(*inputPointCloud->getPointCloud())[i].getY(), (*inputPointCloud->getPointCloud())[i].getZ(), 0);

		Eigen::Vector4d n = Eigen::Vector4d (this->normals->getNormals()->data()[i].getX(),
				this->normals->getNormals()->data()[i].getY(),
				this->normals->getNormals()->data()[i].getZ(), 0);

		double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

		// Calculate the point's projection on the cylinder axis
		double k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
		Eigen::Vector4d pt_proj = line_pt + k * line_dir;
		Eigen::Vector4d dir = pt - pt_proj;
		dir.normalize ();

		// Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
		double d_normal = fabs (getAngle3D (n, dir));
		d_normal = fmin (d_normal, M_PI - d_normal);

		distances[i] = fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight)
				* d_euclid);
	}
}


void ObjectModelCylinder::selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
		std::vector<int> &inliers){
	assert (model_coefficients.size () == 7);

	int nr_p = 0;
	inliers.resize (this->inputPointCloud->getSize());

	Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	double ptdotdir = line_pt.dot (line_dir);
	double dirdotdir = 1.0 / line_dir.dot (line_dir);
	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Aproximate the distance from the point to the cylinder as the difference between
		// dist(point,cylinder_axis) and cylinder radius
		Eigen::Vector4d pt = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[i].getX(),
				(*inputPointCloud->getPointCloud())[i].getY(),
				(*inputPointCloud->getPointCloud())[i].getZ(), 0);

		Eigen::Vector4d n = Eigen::Vector4d (this->normals->getNormals()->data()[i].getX(),
				this->normals->getNormals()->data()[i].getY(),
				this->normals->getNormals()->data()[i].getZ(), 0);

		double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

		// Calculate the point's projection on the cylinder axis
		double k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
		Eigen::Vector4d pt_proj = line_pt + k * line_dir;
		Eigen::Vector4d dir = pt - pt_proj;
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


void ObjectModelCylinder::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
		std::vector<double> &distances){

	distances.resize (inliers.size());

	Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

	double ptdotdir = line_pt.dot (line_dir);

	double dirdotdir = 1.0 / line_dir.dot (line_dir);
	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < inliers.size(); ++i)
	{
		// Aproximate the distance from the point to the cylinder as the difference between
		// dist(point,cylinder_axis) and cylinder radius
		// @note need to revise this.
		Eigen::Vector4d pt = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[inliers[i]].getX(),
				(*inputPointCloud->getPointCloud())[inliers[i]].getY(), (*inputPointCloud->getPointCloud())[inliers[i]].getZ(), 0);

		Eigen::Vector4d n = Eigen::Vector4d (this->normals->getNormals()->data()[inliers[i]].getX(),
				this->normals->getNormals()->data()[inliers[i]].getY(),
				this->normals->getNormals()->data()[inliers[i]].getZ(), 0);

		double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

		// Calculate the point's projection on the cylinder axis
		double k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
		Eigen::Vector4d pt_proj = line_pt + k * line_dir;
		Eigen::Vector4d dir = pt - pt_proj;
		dir.normalize ();

		// Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
		double d_normal = fabs (getAngle3D (n, dir));
		d_normal = fmin (d_normal, M_PI - d_normal);

		distances[i] = fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight)
				* d_euclid);
	}
}



bool ObjectModelCylinder::doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients,
		double threshold){

    assert (model_coefficients.size () == 7);

    Eigen::Vector4d pt;
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
    {
      // Aproximate the distance from the point to the cylinder as the difference between
      // dist(point,cylinder_axis) and cylinder radius
      // @note need to revise this.
      pt = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[*it].getX(), (*inputPointCloud->getPointCloud())[*it].getY(),
    		  (*inputPointCloud->getPointCloud())[*it].getZ(), 0);
      if (fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]) > threshold)
        return (false);
    }

    return (true);
}


double
ObjectModelCylinder::pointToLineDistance (const Eigen::Vector4d &pt, const Eigen::VectorXd &model_coefficients)
{
	Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	// Calculate the distance from the point to the line
	// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
	Eigen::Vector4d r, p_t;
	r = line_pt + line_dir;
	p_t = r - pt;

#ifdef EIGEN3
	Eigen::Vector3d c = p_t.head<3> ().cross (line_dir.head<3> ());
#else
	Eigen::Vector3d c = p_t.start<3> ().cross (line_dir.start<3> ());
#endif
	return (sqrt (c.dot (c) / line_dir.dot (line_dir)));
}

double
ObjectModelCylinder::pointToLineDistance (const Eigen::Vector4d &pt, const Eigen::Vector4d &line_pt, const Eigen::Vector4d &line_dir)
{
	// Calculate the distance from the point to the line
	// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
	Eigen::Vector4d r, p_t;
	r = line_pt + line_dir;
	p_t = r - pt;

#ifdef EIGEN3
	Eigen::Vector3d c = p_t.head<3> ().cross (line_dir.head<3> ());
#else
	Eigen::Vector3d c = p_t.start<3> ().cross (line_dir.start<3> ());
#endif
	return (sqrt (c.dot (c) / line_dir.dot (line_dir)));
}



}
