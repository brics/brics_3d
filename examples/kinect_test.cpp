/**
 * @file 
 * kinect_test.cpp
 *
 * @date: Jun 15, 2011
 * @author: sblume
 */

#include "util/OSGPointCloudVisualizer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_camera/openni_depth_image.h>

void callback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
         std::cout << "received image" << std::endl;
     }

int main(int argc, char **argv) {

    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&SimpleKinectViewer::callback, _1);
    interface->registerCallback (f);
    interface->start ();

}


/* EOF */
