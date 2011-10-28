/*
 * extract_object_clusters.cpp
 *
 *  Created on: Oct 26, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "util/OSGPointCloudVisualizer.h"
#include "core/PointCloud3D.h"
#include "core/ColoredPointCloud3D.h"

#include "util/PCLTypecaster.h"
#include "algorithm/segmentation/EuclideanClustering3D.h"
#include "algorithm/segmentation/EuclideanClustering.h"
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
	BRICS_3D::PCLTypecaster pclTypecaster;
	BRICS_3D::ColorBasedROIExtractorHSV roiExtractor;
	BRICS_3D::EuclideanClustering3D clusterExtractor;
	BRICS_3D::EuclideanClustering pclClusterExtractor;
	BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;
	BRICS_3D::Centroid3D centroid3DEstimator;
	Eigen::Vector3d centroid;
	bool processingDone;
public:

	BRICS_3D::OSGPointCloudVisualizer viewer;

	void processData(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
		if(!processingDone){
			processingDone = true;


			// The magic starts here:P
			cout << "[CHEAT] Recieved Kinect Cloud Size " << cloud->size() << endl;

			//Converting from PCL to BRICS_3D datatype
			BRICS_3D::ColoredPointCloud3D *inCloud = new BRICS_3D::ColoredPointCloud3D();
			BRICS_3D::PointCloud3D *inCloudVis = new BRICS_3D::ColoredPointCloud3D();
			pclTypecaster.convertToBRICS3DDataType(cloud,inCloudVis);

			clock_t startTime = clock();
			pclTypecaster.convertToBRICS3DDataType(cloud,inCloud);
//			cout << "[CHEAT] Converted Input Cloud Size " << inCloud->getSize() << "\tExectution Time "
//					<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;

			//Extracting the HSV based Region of Interest
			BRICS_3D::PointCloud3D *extractedRoiHSV = new BRICS_3D::PointCloud3D();
			startTime = clock();
			roiExtractor.extractColorBasedROI(inCloud,extractedRoiHSV);
//			cout << "[CHEAT] ROI Extracted Cloud Size " << extractedRoiHSV->getSize() << "\t\tExectution Time "
//					<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;
			//Extracting the Object Clusters
			vector<BRICS_3D::PointCloud3D*> pclExtractedClusters;
			startTime = clock();
			pclClusterExtractor.extractClusters(extractedRoiHSV,&pclExtractedClusters);
//			cout << "[CHEAT] No of clusters extracted (PCL): " << pclExtractedClusters.size() << "\t\tExectution Time "
//					<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;


			//Extracting the Object Clusters
			//		vector<BRICS_3D::PointCloud3D*> extractedClusters;
			//		startTime = clock();
			//		clusterExtractor.extractClusters(extractedRoiHSV,&extractedClusters);
			//		cout << "[CHEAT] No of clusters extracted : " << extractedClusters.size() << "\t\tExectution Time "
			//				<< double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." <<endl;

			//Obtaining the inial 3D pose
			for (size_t i=0; i< pclExtractedClusters.size(); i++){
				centroid = centroid3DEstimator.computeCentroid(pclExtractedClusters[i]);
//				cout << "[CHEAT] Pose of Object-" << i+1 << " is : ["<<centroid[0] << ", "
//						<< centroid[1] << ", " << centroid[2] << "]" << endl;
			}

			//Visualizing the output

//			viewer.addPointCloud(inCloudVis);
			viewer.addPointCloud(extractedRoiHSV, 1, 0, 0, 0);
			viewer.clearButLast();


			delete inCloud;
			delete inCloudVis;
//			delete extractedRoiHSV;
//			pclExtractedClusters.clear();

			processingDone = false;
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

