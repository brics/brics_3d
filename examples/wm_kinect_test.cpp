/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Sebastian Blumenthal
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>

#include <core/PointCloud3D.h>
#include <core/Logger.h>
#include <util/PCLTypecaster.h>
#include <worldModel/WorldModel.h>
#include <worldModel/sceneGraph/PointCloud.h>
#include <worldModel/sceneGraph/OSGVisualizer.h>

using BRICS_3D::Logger;

class WorldModelKinectTest {
public:
	WorldModelKinectTest(){
		BRICS_3D::Logger::setMinLoglevel(BRICS_3D::Logger::LOGDEBUG); // More debug output

		count = 0;
		wm = new BRICS_3D::WorldModel();
		wmObserver = new BRICS_3D::RSG::OSGVisualizer();
		wm->scene.attachUpdateObserver(wmObserver); //enable visualization
		lastPointCloudId = 0;
	}

	virtual ~WorldModelKinectTest(){
		delete wm;
		delete wmObserver;
	}

	void callback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{
//		if (count > 2) {
		if (wmObserver->done()) {
			LOG(INFO) <<  "Done.";
			interface->stop();
			return;
		}

		LOG(INFO) <<  "Receiving new point cloud";

		/* Create new BRICS_3D data structures */
		BRICS_3D::PointCloud3D::PointCloud3DPtr newPointCloud(new BRICS_3D::PointCloud3D());
		BRICS_3D::RSG::PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr newPointCloudContainer(new BRICS_3D::RSG::PointCloud<BRICS_3D::PointCloud3D>());
		newPointCloudContainer->data=newPointCloud;
		converter.convertToBRICS3DDataType(cloud, newPointCloud.get());

		/* Add new point cloud to world model */
		unsigned int curretPointCloudId = 0;
		vector<BRICS_3D::RSG::Attribute> tmpAttributes;
		tmpAttributes.push_back(Attribute("name","raw_point_cloud"));
		wm->scene.addGeometricNode(wm->getRootNodeId(), curretPointCloudId, tmpAttributes, newPointCloudContainer, TimeStamp(0.0));

		/* Delete the point cloud from previous cycle */
		wm->scene.deleteNode(lastPointCloudId);
		lastPointCloudId = curretPointCloudId;

		count++;
	}

	/// Kinect interface
	pcl::Grabber* interface;

	/// Helper tool to convert point clouds
	BRICS_3D::PCLTypecaster converter;

	/// The world model that stores all 3D data.
	BRICS_3D::WorldModel* wm;

	/// Visualization tool for world model
	BRICS_3D::RSG::OSGVisualizer* wmObserver;

	/// Hint which point cloud is is from previous cycle.
	unsigned int lastPointCloudId;

	/// For stats & debugging
	int count;

	void run () {
	    interface = new pcl::OpenNIGrabber();
	    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&WorldModelKinectTest::callback, this, _1);
	    interface->registerCallback (f);
	    interface->start ();
	}

};

int main(int argc, char **argv) {

	WorldModelKinectTest wmKinectTest;
	wmKinectTest.run();

    while (!wmKinectTest.wmObserver->done())
    {
      sleep (1);
    }

}

/* EOF */
