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

#include "ObjectModelPlaneFromLines.h"

namespace BRICS_3D {

void ObjectModelPlaneFromLines::computeRandomModel (int &iterations, Eigen::VectorXf &model_coefficients, bool &isDegenerate, bool &modelFound){

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


void ObjectModelPlaneFromLines::getSamples (int &iterations, std::vector<int> &samples){

	points = inputPointCloud->getPointCloud();
	//ToDo check that the pointcloud size > 0
	samples.resize (4);
	double trand = inputPointCloud->getSize() / (RAND_MAX + 1.0);


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

#ifdef EIGEN3
		dy1dy2 = p1.array () / p2.array();
#else
		dy1dy2 = p1.cwise () / p2;
#endif
		++iter;
		if (iter > MAX_ITERATIONS_COLLINEAR )
		{
			cout<<"[SampleConsensusModelPlane::getSamples] WARNING: Could not select 3 non collinear points in"<<  MAX_ITERATIONS_COLLINEAR <<"iterations!";
			break;
		}
		iterations++;
	}
	while ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) );
	iterations--;

	//Select a fourth point which is not equal to the other three non-collinear points
	if(iter < MAX_ITERATIONS_COLLINEAR) {
		do
		{
			idx = (int)(rand () * trand);
			samples[3] = idx;
			iterations++;

			if (iter > MAX_ITERATIONS_COLLINEAR )
			{
				cout<<"[SampleConsensusModelPlane::getSamples] WARNING: Could not select 4 points in"<<  MAX_ITERATIONS_COLLINEAR <<"iterations!";
				break;
			}

		} while ( (samples[3] == samples[0]) || (samples[3] == samples[1]) || (samples[3] == samples[2]) );
		iterations--;
	}

}

bool
ObjectModelPlaneFromLines::computeModelCoefficients (const std::vector<int> &sample,
		Eigen::VectorXf &modelCoefficients){

	line line1,line2;

	//Create line1 from sample[0] and sample[1]
	line1.pbase = this->points->data()[sample[0]];
	line1.director.resize(3);
	line1.director[0]=this->points->data()[sample[1]].getX() - this->points->data()[sample[0]].getX();
	line1.director[1]=this->points->data()[sample[1]].getY() - this->points->data()[sample[0]].getY();
	line1.director[2]=this->points->data()[sample[1]].getZ() - this->points->data()[sample[0]].getZ();

	//Create line1 from sample[2] and sample[3]
	line2.pbase = this->points->data()[sample[2]];
	line2.director.resize(3);
	line2.director[0]=this->points->data()[sample[3]].getX() - this->points->data()[sample[2]].getX();
	line2.director[1]=this->points->data()[sample[3]].getY() - this->points->data()[sample[2]].getY();
	line2.director[2]=this->points->data()[sample[3]].getZ() - this->points->data()[sample[2]].getZ();



	//Compute the model co-efficient of the lines from these lines
	std::vector<double> normal;
	normal.resize(3);

	crossProduct3D(line1.director,line2.director,normal);

	modelCoefficients.resize(4);

	modelCoefficients[0] = normal[0];
	modelCoefficients[1] = normal[1];
	modelCoefficients[2] = normal[2];

	modelCoefficients[3]= -modelCoefficients[0] * line1.pbase.getX() -
			modelCoefficients[1] * line1.pbase.getY() -
			modelCoefficients[2] * line1.pbase.getZ();


	if (abs(modelCoefficients[0])<geometryEpsilon&&abs(modelCoefficients[1])<geometryEpsilon&&abs(modelCoefficients[2])<geometryEpsilon)	{
		//Lines are parallel
		if (contains(line1, line2.pbase))
			return false;

		//Use a line's director vector and both pBase's difference to create the plane.
		std::vector<double> baseDifference;
		baseDifference.resize(3);
		baseDifference[0]=line1.pbase.getX()-line2.pbase.getX();
		baseDifference[1]=line1.pbase.getY()-line2.pbase.getY();
		baseDifference[2]=line1.pbase.getZ()-line2.pbase.getZ();

		crossProduct3D(line1.director,baseDifference,normal);
		modelCoefficients[0] = normal[0];
		modelCoefficients[1] = normal[1];
		modelCoefficients[2] = normal[2];

		modelCoefficients[3]=-modelCoefficients[0] * line1.pbase.getX() -modelCoefficients[1] * line1.pbase.getY() -modelCoefficients[2] * line1.pbase.getZ();

	}	else {

		double  x = modelCoefficients[0]*line2.pbase.getX()+
				modelCoefficients[1]*line2.pbase.getY()+
				modelCoefficients[2]*line2.pbase.getZ()+modelCoefficients[3];
		if (abs(x)>=geometryEpsilon) {
			cout<<"Lines do not intersect"<<endl;
			return false;
		}
	}

	return true;
}


}
