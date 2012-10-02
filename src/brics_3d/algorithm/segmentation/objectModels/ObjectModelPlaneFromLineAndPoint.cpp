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

#include "ObjectModelPlaneFromLineAndPoint.h"

namespace brics_3d {


void ObjectModelPlaneFromLineAndPoint::computeRandomModel (int &iterations, Eigen::VectorXf &model_coefficients, bool &isDegenerate, bool &modelFound){

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


void ObjectModelPlaneFromLineAndPoint::getSamples (int &iterations, std::vector<int> &samples){

//	points = inputPointCloud->getPointCloud();

	samples.resize (4);
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
	p1 = Eigen::Vector4f ((*inputPointCloud->getPointCloud())[samples[1]].getX(), (*inputPointCloud->getPointCloud())[samples[1]].getY(), (*inputPointCloud->getPointCloud())[samples[1]].getZ(), 0);
	p0 = Eigen::Vector4f ((*inputPointCloud->getPointCloud())[samples[0]].getX(), (*inputPointCloud->getPointCloud())[samples[0]].getY(), (*inputPointCloud->getPointCloud())[samples[0]].getZ(), 0);

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
		p2 = Eigen::Vector4f ((*inputPointCloud->getPointCloud())[samples[2]].getX(), (*inputPointCloud->getPointCloud())[samples[2]].getY(), (*inputPointCloud->getPointCloud())[samples[2]].getZ(), 0);

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
			cout<<"[SampleConsensusModelPlaneFromLineAndPoint::getSamples] WARNING: Could not select 3 non collinear points in"<<  MAX_ITERATIONS_COLLINEAR <<"iterations!";
			break;
		}
		iterations++;
	}
	while ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) );
	iterations--;
}


bool
ObjectModelPlaneFromLineAndPoint::computeModelCoefficients (const std::vector<int> &sample,
		Eigen::VectorXf &modelCoefficients){

	line line1,line2;

	//Create line1 from sample[0] and sample[1]
	line1.pbase = (*inputPointCloud->getPointCloud())[sample[0]];
	line1.director.resize(3);
	line1.director[0]=(*inputPointCloud->getPointCloud())[sample[1]].getX() - (*inputPointCloud->getPointCloud())[sample[0]].getX();
	line1.director[1]=(*inputPointCloud->getPointCloud())[sample[1]].getY() - (*inputPointCloud->getPointCloud())[sample[0]].getY();
	line1.director[2]=(*inputPointCloud->getPointCloud())[sample[1]].getZ() - (*inputPointCloud->getPointCloud())[sample[0]].getZ();

	//Compute the model coefficients

	double dx1=(*inputPointCloud->getPointCloud())[sample[2]].getX()-line1.pbase.getX();
	double dy1=(*inputPointCloud->getPointCloud())[sample[2]].getY()-line1.pbase.getY();
	double dz1=(*inputPointCloud->getPointCloud())[sample[1]].getZ()-line1.pbase.getZ();

	modelCoefficients.resize(4);

	modelCoefficients[0]=dy1*line1.director[2]-dz1*line1.director[1];
	modelCoefficients[1]=dz1*line1.director[0]-dx1*line1.director[2];
	modelCoefficients[2]=dx1*line1.director[1]-dy1*line1.director[0];

	if (abs(modelCoefficients[0])<geometryEpsilon&&abs(modelCoefficients[1])<
			geometryEpsilon&&abs(modelCoefficients[2])<geometryEpsilon){
		cout<<"Point is contained in the line"<<endl;
		return false;
	}
	modelCoefficients[3]=-modelCoefficients[0]*(*inputPointCloud->getPointCloud())[sample[2]].getX()-
			modelCoefficients[1]*(*inputPointCloud->getPointCloud())[sample[2]].getY()-
			modelCoefficients[2]*(*inputPointCloud->getPointCloud())[sample[2]].getZ();

	return true;
}

}
