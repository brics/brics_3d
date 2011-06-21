/**
 * @file 
 * detection_test.cpp
 *
 * @date: Jun 20, 2011
 * @author: sblume
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
using namespace pcl;

class ObjectRecognizer {
public:

	typedef PointXYZ PointT;
//    typedef pcl::PointCloud<PointXYZ> PointCloud;
//    typedef PointCloud::Ptr PointCloudPtr;
//    typedef PointCloud::ConstPtr PointCloudConstPtr;

	ObjectRecognizer(){
		maxObjectPoints = 200;
		minObjectPoints = 10;
	};
	~ObjectRecognizer(){};

	void setInputCloud (PointCloud<PointXYZ>::Ptr cloud) {
		inputCloud = cloud;
	}
	bool compute(std::vector<Eigen::Matrix4d>& transformationMatrices){

		transformationMatrices.clear();
		cerr << "PointCloud before filtering has: " << inputCloud->points.size () << " data points." << endl; //*

		// Create the filtering object: downsample the dataset using a leaf size of 1cm
		VoxelGrid<PointXYZ> vg;
		PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ>);
		vg.setInputCloud (inputCloud);
		vg.setLeafSize (0.01, 0.01, 0.01);
		vg.filter (*cloud_filtered);
		cerr << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << endl; //*

		// Create the segmentation object for the planar model and set all the parameters
		SACSegmentation<PointXYZ> seg;
		PointIndices::Ptr inliers (new PointIndices);
		ModelCoefficients::Ptr coefficients (new ModelCoefficients);
		PointCloud<PointXYZ>::Ptr cloud_plane (new PointCloud<PointXYZ> ());
		PointCloud<PointXYZ>::Ptr cloud_cylinder (new PointCloud<PointXYZ> ());
		PCDWriter writer;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (SACMODEL_PLANE);
		seg.setMethodType (SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.02);

		int i=0, nr_points = cloud_filtered->points.size ();
		//    while (cloud_filtered->points.size () > 0.3 * nr_points)
		//    {
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment (*inliers, *coefficients); //*
		if (inliers->indices.size () == 0)
		{
			cout << "Could not estimate a planar model for the given dataset." << endl;
			//            break;
		}

		// Extract the planar inliers from the input cloud
		ExtractIndices<PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Write the planar inliers to disk
		extract.filter (*cloud_plane); //*
		cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_filtered); //*
		//   }

		// Creating the KdTree object for the search method of the extraction
		KdTree<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ>);
		tree->setInputCloud (cloud_filtered);

		vector<PointIndices> cluster_indices;
		EuclideanClusterExtraction<PointXYZ> ec;
		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud( cloud_filtered);
		ec.extract (cluster_indices);

		pcl::SACSegmentationFromNormals<PointXYZ, pcl::Normal> cylinder_seg;
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

		int j = 0;
		double minimumInlierRatio = 0.5;
		double inlierRatio;
		for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			PointCloud<PointXYZ>::Ptr cloud_cluster (new PointCloud<PointXYZ>);
			for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

			cerr << "PointCloud representing the Cluster" << j << ": " << cloud_cluster->points.size () << " data points." << endl;
			stringstream ss;
			ss << "cloud_cluster_" << j << ".pcd";
			writer.write<PointXYZ> (ss.str (), *cloud_cluster, false); //*
			//        pcl::compute3DCentroid()


			if (minObjectPoints <= cloud_cluster->points.size () && cloud_cluster->points.size () <= maxObjectPoints) {

				//Normal estimation
				//       PointCloud<PointNormal>::Ptr cloud_cluster_withNormals (new PointCloud<PointNormal>);
				pcl::PointCloud<pcl::Normal>::Ptr cloud_cluster_normals (new pcl::PointCloud<pcl::Normal>);
				pcl::NormalEstimation<PointXYZ, Normal> ne;
				ne.setSearchMethod (tree);
				ne.setInputCloud (cloud_cluster);
				ne.setKSearch (50);
				ne.compute (*cloud_cluster_normals);

				// Segmentation
				cylinder_seg.setOptimizeCoefficients (true);
				cylinder_seg.setModelType (SACMODEL_CYLINDER);
				cylinder_seg.setMethodType (SAC_RANSAC);
				cylinder_seg.setMaxIterations (100);
				cylinder_seg.setDistanceThreshold (0.02);

				cylinder_seg.setInputCloud(cloud_cluster);
				cylinder_seg.setInputNormals(cloud_cluster_normals);
				cylinder_seg.segment (*inliers, *coefficients);
				if (inliers->indices.size () == 0)
				{
					cout << "Could not estimate a cylinder for model model for the given dataset." << endl;
					//continue;
				}


				// Extract the planar inliers from the input cloud
				ExtractIndices<PointXYZ> extract;
				extract.setInputCloud (cloud_cluster);
				extract.setIndices (inliers);
				extract.setNegative (false);

				// Write the cylindrical inliers to disk
				extract.filter (*cloud_cylinder); //*

				inlierRatio = static_cast<double>(cloud_cylinder->points.size()) / static_cast<double>(cloud_cluster->points.size());
				cout << "inlierRatio = " << inlierRatio << endl;

				if (inlierRatio >= minimumInlierRatio ) {

					cerr << "PointCloud representing the a cylinder in a cluster" << j << ": " << cloud_cylinder->points.size () << " data points." << endl;
					ss.str("");
					ss << "cloud_cylinder_" << j << ".pcd";
					writer.write<PointXYZ> (ss.str (), *cloud_cylinder, false); //*
				}

			}
			j++;
		}

		return true;
	}


	// Hints for the object
	double maxObjectPoints;
	double minObjectPoints;
private:
	//const PointCloudConstPtr inputCloud;
	PointCloud<PointXYZ>::Ptr inputCloud;
};

int
main (int argc, char** argv)
{
	bool useDataSet = true;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	if (useDataSet) {
	// Read in the cloud data
    PCDReader reader;

//    reader.read ("/home/sblume/workspace/pcl_test/table_scene_lms400.pcd", *cloud);
    reader.read ("./data/at_work2/kinect_pointcloud0.pcd", *cloud);
    cerr << "PointCloud before filtering has: " << cloud->points.size () << " data points." << endl; //*
	} else {

	}


    ObjectRecognizer objRec;
    std::vector<Eigen::Matrix4d> resultFrames;
    objRec.setInputCloud(cloud);
    objRec.compute(resultFrames);


    return (0);
}

/* EOF */
