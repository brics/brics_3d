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

#include "util/OSGPointCloudVisualizer.h"
#include "core/PointCloud3D.h"

#include "util/PCLTypecaster.h"
#include "algorithm/segmentation/EuclideanClustering.h"
#include "algorithm/segmentation/EuclideanClusteringPCL.h"
#include "algorithm/segmentation/RGBColorBasedEuclideanClustering.h"
#include "algorithm/filtering/ColorBasedROIExtractorHSV.h"
#include "algorithm/featureExtraction/Centroid3D.h"
#include "core/ColorSpaceConvertor.h"


#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>

using namespace std;
class ExtractObjectClusters{

private:
	pcl::Grabber* interface;
	int count;
	brics_3d::PCLTypecaster pclTypecaster;
	brics_3d::ColorBasedROIExtractorHSV roiExtractor;
	brics_3d::EuclideanClustering clusterExtractor;
	brics_3d::EuclideanClusteringPCL pclClusterExtractor;
	brics_3d::RGBColorBasedEuclideanClustering rgbColorBasedClusterExtractor;
	brics_3d::ColorSpaceConvertor colorSpaceConvertor;
	brics_3d::Centroid3D centroid3DEstimator;
	Eigen::Vector3d centroid;
	bool processingDone;
public:

	brics_3d::OSGPointCloudVisualizer viewer;

	void processData(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
		if(!processingDone){
			processingDone = true;


			// The magic starts here:P
			cout << "[CHEAT] Recieved Kinect Cloud Size " << cloud->size() << endl;

			//Converting from PCL to BRICS_3D datatype
			brics_3d::PointCloud3D *inCloud = new brics_3d::PointCloud3D();
			brics_3d::PointCloud3D *inCloudVis = new brics_3d::PointCloud3D();
			pclTypecaster.convertToBRICS3DDataType(cloud,inCloudVis);




			clock_t startTime = clock();
			pclTypecaster.convertToBRICS3DDataType(cloud,inCloud);
			cout << "[CHEAT] Converted Input Cloud Size " << inCloud->getSize() << "\tExectution Time "
					<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;




			//Extracting the HSV based Region of Interest
			brics_3d::PointCloud3D *extractedRoiHSV = new brics_3d::PointCloud3D();
			startTime = clock();
			roiExtractor.filter(inCloud,extractedRoiHSV);
			cout << "[CHEAT] ROI Extracted Cloud Size " << extractedRoiHSV->getSize() << "\t\tExectution Time "
					<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;



			//Extracting the HSV based Region of Interest
			brics_3d::PointCloud3D *extractedRoiHSVColored = new brics_3d::PointCloud3D();
			roiExtractor.filter(inCloud,extractedRoiHSVColored);
			cout << "[CHEAT] ROI Extracted Colored-Cloud Size " << extractedRoiHSVColored->getSize() << "\t\tExectution Time "
								<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;


			//Extracting Color based regions
			vector<brics_3d::PointCloud3D*> colorBasedClusters;
			startTime = clock();
			rgbColorBasedClusterExtractor.setPointCloud(extractedRoiHSVColored);
			rgbColorBasedClusterExtractor.setMinClusterSize(100);
			rgbColorBasedClusterExtractor.setMaxClusterSize(10000);
			rgbColorBasedClusterExtractor.setToleranceRgbSpace(100);
			rgbColorBasedClusterExtractor.setToleranceEuclideanDistance(100);
			rgbColorBasedClusterExtractor.segment();
			rgbColorBasedClusterExtractor.getExtractedClusters(colorBasedClusters);
			cout << "[CHEAT] No of colored clusters extracted: " << colorBasedClusters.size() << "\t\tExectution Time "
					<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;




			//Extracting the Object Clusters
			vector<brics_3d::PointCloud3D*> pclExtractedClusters;
			startTime = clock();
			pclClusterExtractor.setPointCloud(extractedRoiHSV);
			pclClusterExtractor.segment();
			pclClusterExtractor.getExtractedClusters(pclExtractedClusters);
			cout << "[CHEAT] No of clusters extracted (PCL): " << pclExtractedClusters.size() << "\t\tExectution Time "
					<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;



			//Extracting the Object Clusters
			//		vector<brics_3d::PointCloud3D*> extractedClusters;
			//		startTime = clock();
			//		clusterExtractor.extractClusters(extractedRoiHSV,&extractedClusters);
			//		cout << "[CHEAT] No of clusters extracted : " << extractedClusters.size() << "\t\tExectution Time "
			//				<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;

			//Obtaining the inial 3D pose
//			for (size_t i=0; i< pclExtractedClusters.size(); i++){
//				centroid = centroid3DEstimator.computeCentroid(pclExtractedClusters[i]);
//				cout << "[CHEAT] Pose of Object-" << i+1 << " is : ["<<centroid[0] << ", "
//						<< centroid[1] << ", " << centroid[2] << "]" << endl;
//			}

			//Visualizing the output
			viewer.clearButLast();
			viewer.addPointCloud(inCloudVis);
			viewer.addPointCloud(extractedRoiHSV, 1, 0, 0, 0);



			delete inCloud;
			delete inCloudVis;
			delete extractedRoiHSV;
			delete extractedRoiHSVColored;

			processingDone = false;
			exit(0);
		}
	}

	void callback (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
		processData(cloud);
	}


	void keepLooking () {

		//initialize the algorithms
		//Setting Approximate HSV Limits for Green-Objects
		roiExtractor.setMinS(132);
		roiExtractor.setMaxS(198);
		roiExtractor.setMinH(65);
		roiExtractor.setMaxH(130);

		//Setting up cluster extraction limits
		clusterExtractor.setMinClusterSize(100);
		clusterExtractor.setMaxClusterSize(10000);
		clusterExtractor.setClusterTolerance(0.02);

		pclClusterExtractor.setMinClusterSize(100);
		pclClusterExtractor.setMaxClusterSize(10000);
		pclClusterExtractor.setClusterTolerance(0.02);


		interface = new pcl::OpenNIGrabber();
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind (&ExtractObjectClusters::callback, this, _1);
		interface->registerCallback (f);
		interface->start ();

		processingDone=false;

		cout << "[CHEAT] Initialization Done" << endl;
	}

};

int main(){

	ExtractObjectClusters objectClusterExtractor;
	objectClusterExtractor.keepLooking();

	while (!objectClusterExtractor.viewer.done())
	{
		sleep (1);
	}


}

