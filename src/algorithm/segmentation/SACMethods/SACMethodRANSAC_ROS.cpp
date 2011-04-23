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
}

SACMethodRANSAC_ROS::~SACMethodRANSAC_ROS() {
	// TODO Auto-generated destructor stub
}

bool SACMethodRANSAC_ROS::computeModel(){

	// Warn and exit if no threshold was set
     if (this->threshold == -1)
     {
       cout<<"[RANSAC::computeModel] No threshold set!"<<endl;
       return (false);
     }

     this->iterations = 0;
     int n_best_inliers_count = -1;
     double k = 1.0;

     std::vector<int> best_model;
     std::vector<int> best_inliers, inliers;
     std::vector<int> selection;
     Eigen::VectorXf modelCoefficients;

     int n_inliers_count = 0;


     // Iterate
     while (this->iterations < k)
     {

       // Get X samples which satisfy the model criteria
       this->objectModel->getSamples(this->iterations,selection);


       if (selection.size () == 0) break;


       // Search for inliers in the point cloud for the current plane model M
       if (!this->objectModel->computeModelCoefficients (selection, modelCoefficients))
       {
         this->iterations++;
         continue;
       }


       // Select the inliers that are within threshold_ from the model
       this->objectModel->selectWithinDistance (modelCoefficients, this->threshold, inliers);
       n_inliers_count = inliers.size ();


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


       this->iterations++;
       if (this->iterations > this->maxIterations)
       {
           cout<<"[RANSAC::computeModel] RANSAC reached the maximum number of trials";
         break;
       }

     }


     if (this->model.size () == 0)
       return (false);
     return (true);
}

}
