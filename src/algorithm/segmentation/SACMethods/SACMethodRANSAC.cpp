/*
 * @file: SACMethodRANSAC.cpp
 *
 * @date: Apr 19, 2011
 * @author: reon
 */

#include "SACMethodRANSAC.h"

namespace BRICS_3D {


SACMethodRANSAC::SACMethodRANSAC(){}

SACMethodRANSAC::~SACMethodRANSAC() {}

bool SACMethodRANSAC::computeModel(){

	 // Warn and exit if no threshold was set
     if (this->threshold == -1)
     {
       cout<<"[RANSAC::computeModel] No threshold set!"<<endl;
       return (false);
     }


     this->iterations = 0;
     int noMaxInliersFound = -DBL_MAX;
     double k = 1.0;

     std::vector<int> bestModel;
     std::vector<int> bestModelIndices, inliers;
     Eigen::VectorXd estimatedModelCoefficients;

     int noInliersCurrentModel = 0;


     while (this->iterations < k)
     {

       //Compute a random model from the input pointcloud
       bool isDegenerate = false;
       bool modelFound = false;
       this->objectModel->computeRandomModel(this->iterations,estimatedModelCoefficients,isDegenerate,modelFound);



       if (!isDegenerate) break;

       if(!modelFound){

    	   this->iterations++;
    	   continue;
       }



       // The inliers within the user-defined threshold are selected
       this->objectModel->selectWithinDistance (estimatedModelCoefficients, this->threshold, inliers);
       noInliersCurrentModel = inliers.size ();


       //Select the best model estimation. The number of inliers in the model
       //will be maximized
       if (noInliersCurrentModel > noMaxInliersFound)
       {
         noMaxInliersFound = noInliersCurrentModel;

         // Save the current model/inlier/coefficients selection as being the best so far
         this->inliers            = inliers;
         this->modelCoefficients = estimatedModelCoefficients;

         // Compute the k parameter (k=log(z)/log(1-w^n))
         double w = (double)((double)noInliersCurrentModel / (double)this->objectModel->getInputCloud()->getSize());
         double pNoOutliers = 1 - pow (w, (double)this->objectModel->getNumberOfSamplesRequired());
         pNoOutliers = std::max (std::numeric_limits<double>::epsilon (), pNoOutliers);       // Avoid division by -Inf
         pNoOutliers = std::min (1 - std::numeric_limits<double>::epsilon (), pNoOutliers);   // Avoid division by 0.
         k = log (1 - this->probability) / log (pNoOutliers);
       }


       this->iterations++;

       if (this->iterations > this->maxIterations)
       {
           cout<<"[RANSAC::computeModel] RANSAC reached the maximum number of trials";
         break;
       }

     }


     if (this->inliers.size() == 0)
       return (false);
     return (true);
}

}
