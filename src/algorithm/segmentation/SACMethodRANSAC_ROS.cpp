/*
 * SACMethodRANSAC_ROS.cpp
 *
 *  Created on: Apr 19, 2011
 *      Author: reon
 */

#include "SACMethodRANSAC_ROS.h"

namespace BRICS_3D {
//Todo default threshold =-1 and probability = 0.99

SACMethodRANSAC_ROS::SACMethodRANSAC_ROS(){
	// TODO Auto-generated constructor stub
	setDistanceThreshold(-1);
	setProbability(0.99);
	setMaxIterations(10000);
}

SACMethodRANSAC_ROS::~SACMethodRANSAC_ROS() {
	// TODO Auto-generated destructor stub
}

/*
SACMethodRANSAC_ROS::SACMethodRANSAC_ROS(IObjectModel *objectModel, PointCloud3D *pointCloud){
	setDistanceThreshold(-1);
	this->objectModel = objectModel;
	setProbability(0.99);
	setMaxIterations(10000);
	this->inputPointCloud = pointCloud;
}
SACMethodRANSAC_ROS::SACMethodRANSAC_ROS(IObjectModel *objectModel,double threshold, PointCloud3D *pointCloud){
	setDistanceThreshold(threshold);
	this->objectModel = objectModel;
	setProbability(0.99);
	setMaxIterations(10000);
	this->inputPointCloud = pointCloud;
}
*/

bool SACMethodRANSAC_ROS::computeModel(){

	// Warn and exit if no threshold was set
cout<<"[Checkpoint]: 1.b.i \n";
     if (this->threshold == -1)
     {
       cout<<"[RANSAC::computeModel] No threshold set!";
       return (false);
     }

cout<<"[Checkpoint]: 1.b.ii \n";
     this->iterations = 0;
     int n_best_inliers_count = -1;
     double k = 1.0;

     std::vector<int> best_model;
     std::vector<int> best_inliers, inliers;
     std::vector<int> selection;
     Eigen::VectorXf modelCoefficients;

     int n_inliers_count = 0;

cout<<"[Checkpoint]: 1.b.iii \n";
bool checkpointFlag = true;
     // Iterate
     while (this->iterations < k)
     {
if(checkpointFlag==true)cout<<"[Checkpoint]: 1.b.iv \n";
       // Get X samples which satisfy the model criteria
       this->objectModel->getSamples(this->iterations,selection);

if(checkpointFlag==true)cout<<"[Checkpoint]: 1.b.v \n";
       if (selection.size () == 0) break;

if(checkpointFlag==true)cout<<"[Checkpoint]: 1.b.vi \n";
       // Search for inliers in the point cloud for the current plane model M
       if (!this->objectModel->computeModelCoefficients (selection, modelCoefficients))
       {
         this->iterations++;
         continue;
       }

if(checkpointFlag==true)cout<<"[Checkpoint]: 1.b.vii \n";
       // Select the inliers that are within threshold_ from the model
       this->objectModel->selectWithinDistance (modelCoefficients, this->threshold, inliers);
       n_inliers_count = inliers.size ();

if(checkpointFlag==true)cout<<"[Checkpoint]: 1.b.viii \n";
       // Better match ?
       if (n_inliers_count > n_best_inliers_count)
       {
         n_best_inliers_count = n_inliers_count;

         // Save the current model/inlier/coefficients selection as being the best so far
         this->inliers            = inliers;
         this->model              = selection;
         this->modelCoefficients = modelCoefficients;

         // Compute the k parameter (k=log(z)/log(1-w^n))
         double w = (double)((double)n_inliers_count / (double)this->objectModel->getInputCloud()->getSize());//Todo change this if using indices getIndices ()->size ());
         double p_no_outliers = 1 - pow (w, (double)selection.size ());
         p_no_outliers = std::max (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
         p_no_outliers = std::min (1 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
         k = log (1 - this->probability) / log (p_no_outliers);
       }

if(checkpointFlag==true)cout<<"[Checkpoint]: 1.b.ix \n";
       this->iterations++;
       if (this->iterations > this->maxIterations)
       {
           cout<<"[RANSAC::computeModel] RANSAC reached the maximum number of trials";
         break;
       }
checkpointFlag=false;
     }

cout<<"[Checkpoint]: 1.b.x \n";
     if (this->model.size () == 0)
       return (false);
     return (true);
}

}
