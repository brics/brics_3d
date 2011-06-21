/**
 * @file 
 * kinect_test.cpp
 *
 * @date: Jun 15, 2011
 * @author: sblume
 */

#include "util/OSGPointCloudVisualizer.h"
#include "core/PointCloud3D.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <util/Benchmark.h>
//#include <pcl/io/openni_camera/openni_depth_image.h>

class KinectTest {
public:
	KinectTest(){
		count = 0;
	}

	void convertPCLToBRICS3D (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputPointCloud, BRICS_3D::PointCloud3D* outputPointCloud) {
		//assert != 0
		(*outputPointCloud->getPointCloud()).resize(inputPointCloud->points.size());
		for (int i = 0; i < static_cast<int>(inputPointCloud->points.size()); i = i+1) {
//			outputPointCloud->addPoint(BRICS_3D::Point3D (inputPointCloud->points[i].x, inputPointCloud->points[i].y, inputPointCloud->points[i].z));
			(*outputPointCloud->getPointCloud())[i].setX(inputPointCloud->points[i].x);
			(*outputPointCloud->getPointCloud())[i].setY(inputPointCloud->points[i].y);
			(*outputPointCloud->getPointCloud())[i].setZ(inputPointCloud->points[i].z);
		}


	}

	void callback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{
//		if (count > 2) {
		if (viewer.done()) {
			std::cout << "done" << std::endl;
			interface->stop();
			return;
		}

		std::cout << "received image" << std::endl;
		BRICS_3D::PointCloud3D* viewerCloud;
		viewerCloud = new BRICS_3D::PointCloud3D();
		BRICS_3D::Point3D point(1,1,1);

		convertPCLToBRICS3D(cloud, viewerCloud);
		viewer.addPointCloud(viewerCloud);
		viewer.clearButLast();
		delete viewerCloud;
		count++;
	}

	BRICS_3D::OSGPointCloudVisualizer viewer;
	pcl::Grabber* interface;

	int count;

	void run () {
	    interface = new pcl::OpenNIGrabber();
	    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&KinectTest::callback, this, _1);
	    interface->registerCallback (f);
	    interface->start ();
	}

};




int main(int argc, char **argv) {

	KinectTest kinectTest;
	kinectTest.run();

    while (!kinectTest.viewer.done())
    {
      sleep (1);
    }

}



/* EOF */
