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

#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/TriangleMeshExplicit.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/PointCloud3DIterator.h>
#include <brics_3d/algorithm/filtering/BoxROIExtractor.h>
#include <brics_3d/algorithm/segmentation/RegionBasedSACSegmentation.h>
#include <brics_3d/algorithm/segmentation/EuclideanClustering.h>
#include <brics_3d/algorithm/segmentation/EuclideanClusteringPCL.h>
#include <brics_3d/algorithm/meshGeneration/DelaunayTriangulationOSG.h>
#include <brics_3d/util/PCLTypecaster.h>
#include <brics_3d/util/PCLPointCloudIterator.h>
#include <brics_3d/util/Timer.h>
#include <brics_3d/util/Benchmark.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/PointCloud.h>
#include <brics_3d/worldModel/sceneGraph/PointCloudPCLBinding.h> //Required to visualize PCL point clouds.
#include <brics_3d/worldModel/sceneGraph/Mesh.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>

using brics_3d::Logger;

//typedef pcl::PointXYZ MyPointType;
typedef pcl::PointXYZRGB MyPointType;

/**
 * This class (a) demonstrates how to uses _iterators_ in
 * conjunction with the world model and (b) compares
 * iterators for BRICS_3D an PCL.
 */
class WorldModelKinectIteratorTest {
public:
	WorldModelKinectIteratorTest(){
		brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG); // More debug output

		benchmark = new brics_3d::Benchmark("wm_iterator_test_benchmark");
		benchmark->output << "# Timings [ms] for: 1.) Conversion 2.) BRICS_3D iteration 3.) PCL iteration 4.) addGeometricNode 5.) complete cycle time " << std::endl;


		count = 0;
		wm = new brics_3d::WorldModel();
		wmObserver = new brics_3d::rsg::OSGVisualizer();
		wm->scene.attachUpdateObserver(wmObserver); //enable visualization
		wm->scene.advertiseRootNode(); // Don't forget this one! Otherwise the observers cannot correctly handle the updates.
		lastPointCloudId = 0;
		lastFilteredPointCloudId = 0;
		lastMeshId = 0;
		roiDiff = 0.05; //[m]

		/* init some ROI Box */
		double xHalfSeize = 2.0; //[m]
		double yHalfSeize = 2.0; //[m]
		double zHalfSeize = 2.0; //[m]

		//HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0.1,0.1,0));
		Eigen::AngleAxis<double> rotation(0 /*M_PI_2/4.0*/, Eigen::Vector3d(0,0,1));
		Transform3d transformation;
		transformation = rotation;
		transformation.translate(Eigen::Vector3d(0,0,1.0));
		brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44(&transformation));

		unsigned int tfBox1Id = 0;
		unsigned int Box1Id = 0;
		vector<brics_3d::rsg::Attribute> tmpAttributes;
		tmpAttributes.clear();
		tmpAttributes.push_back(brics_3d::rsg::Attribute("name","roi_box_tf"));
		wm->scene.addTransformNode(wm->getRootNodeId(), tfBox1Id, tmpAttributes, transform, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));

		brics_3d::rsg::Box::BoxPtr box1(new brics_3d::rsg::Box(xHalfSeize, yHalfSeize, zHalfSeize));
		tmpAttributes.clear();
		tmpAttributes.push_back(brics_3d::rsg::Attribute("name","roi_box"));
		wm->scene.addGeometricNode(tfBox1Id, Box1Id, tmpAttributes, box1, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
		LOG(DEBUG) << "ROI Box added with ID " << Box1Id;

	}

	virtual ~WorldModelKinectIteratorTest(){
		delete wm;
		delete wmObserver;
		delete benchmark;
	}

	void callback (const pcl::PointCloud<MyPointType>::ConstPtr &cloud)
	{
//		if (count > 2) {
		if (wmObserver->done()) {
			LOG(INFO) <<  "Done.";
			interface->stop();
			return;
		}

		LOG(INFO) <<  "Receiving new point cloud";
		totalTimer.reset();

		/* Create new BRICS_3D data structures */
		brics_3d::PointCloud3D::PointCloud3DPtr newPointCloud(new brics_3d::PointCloud3D());
		brics_3d::PointCloud3D::PointCloud3DPtr emptyPointCloud(new brics_3d::PointCloud3D());
		brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr newPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
		newPointCloudContainer->data=newPointCloud;
		timer.reset();
		converter.convertToBRICS3DDataType(cloud, newPointCloud.get());
		benchmark->output << timer.getElapsedTime() << " ";
		LOG(INFO)  << "Converison PCL -> BRICS_3D took: " << timer.getElapsedTime() << "[ms]";
		if(newPointCloudContainer->getPointCloudIterator() != 0) {
			brics_3d::IPoint3DIterator::IPoint3DIteratorPtr itr = newPointCloudContainer->getPointCloudIterator();
			LOG(INFO) << "Found iterator for type " << itr->getPointCloudTypeName();
		}

		pcl::PointCloud<MyPointType>::Ptr newPclPointCloud(new pcl::PointCloud<MyPointType>());
		newPclPointCloud->points.resize(0);
		pcl::PointCloud<MyPointType>::Ptr newPclPointCloud2(new pcl::PointCloud<MyPointType>());
		newPclPointCloud2->points.resize(0);
		brics_3d::rsg::PointCloud<const pcl::PointCloud<MyPointType> >::PointCloudPtr newPclPointCloudContainer(new brics_3d::rsg::PointCloud<const pcl::PointCloud<MyPointType> >());
		newPclPointCloudContainer->data = cloud;// newPointCloud;
		if(newPclPointCloudContainer->getPointCloudIterator() != 0) {
			brics_3d::IPoint3DIterator::IPoint3DIteratorPtr itr = newPclPointCloudContainer->getPointCloudIterator();
			LOG(INFO) << "Found iterator for type " << itr->getPointCloudTypeName();
		}

		brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identityTransform(new brics_3d::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0, 0, 0));
		brics_3d::PointCloud3DIterator* pcIt = new brics_3d::PointCloud3DIterator();
		pcIt->insert(newPointCloud, identityTransform);
		pcIt->insert(emptyPointCloud, identityTransform);

		brics_3d::PCLPointCloudIterator<MyPointType>* it = new brics_3d::PCLPointCloudIterator<MyPointType>();
		it->insert(newPclPointCloud, identityTransform);
		it->insert(cloud, identityTransform);
		it->insert(newPclPointCloud2, identityTransform);

		int itcount = 0;
		LOG(INFO)  << "brics_3d iteration...";
		timer.reset();
		for (pcIt->begin(); !pcIt->end(); pcIt->next()) {
//			LOG(DEBUG)  << pcIt->getX() << ", " << pcIt->getY() << ", " << pcIt->getZ();
			pcIt->getX();
			pcIt->getY();
			pcIt->getZ();
			pcIt->getRawData();
			itcount++;
		}
		benchmark->output << timer.getElapsedTime() << " ";
		LOG(INFO)  << "took for " << itcount << " iterations " << timer.getElapsedTime() << "[ms]";

		itcount  = 0;
		LOG(INFO)  << "pcl iteration...";
		timer.reset();
		for (it->begin(); !it->end(); it->next()) {
//			LOG(DEBUG)  << it->getX() << ", " << it->getY() << ", " << it->getZ();
			it->getX();
			it->getY();
			it->getZ();
			it->getRawData();
//			if(it->getRawData()->asColoredPoint3D() != 0) {
//				LOG(DEBUG) << "Point is decorated with color layer.";
//			} else {
//				LOG(DEBUG) << "Point is not decorated with color layer.";
//			}
			itcount++;
		}
		benchmark->output << timer.getElapsedTime() << " ";
		LOG(INFO)  << "took for " << itcount << " iterations " << timer.getElapsedTime() << "[ms]";

		LOG(INFO)  << "iterations done...";
		delete pcIt;
		delete it;

		/* Add new point cloud to world model */
		unsigned int currentPointCloudId = 0;
		vector<brics_3d::rsg::Attribute> tmpAttributes;
		tmpAttributes.push_back(brics_3d::rsg::Attribute("name","raw_point_cloud"));
		timer.reset();
		wm->scene.addGeometricNode(wm->getRootNodeId(), currentPointCloudId, tmpAttributes, newPclPointCloudContainer, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
//		wm->scene.addGeometricNode(wm->getRootNodeId(), currentPointCloudId, tmpAttributes, newPointCloudContainer, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
		benchmark->output << timer.getElapsedTime() << " ";
		LOG(INFO)  << "Adding to WM (+ visualization) took: " << timer.getElapsedTime() << "[ms]";



		/* sample query */
		brics_3d::rsg::TimeStamp resultTimeStamp;
		brics_3d::rsg::Shape::ShapePtr shape;
		wm->scene.getGeometry(currentPointCloudId, shape, resultTimeStamp);
		brics_3d::rsg::PointCloud<const pcl::PointCloud<MyPointType> >::PointCloudPtr pclPointcloud(new brics_3d::rsg::PointCloud<const pcl::PointCloud<MyPointType> >());
		pclPointcloud = boost::dynamic_pointer_cast<brics_3d::rsg::PointCloud<const pcl::PointCloud<MyPointType> > >(shape);
		if (pclPointcloud != 0) {
			LOG(INFO) << "pcl point cloud found.";
		} else {
			LOG(INFO) << "pcl point cloud not found.";
		}

		brics_3d::IPoint3DIterator::IPoint3DIteratorPtr pointCloudIterator = shape->getPointCloudIterator(); //as an alternative we can use an iterator...
		if (pointCloudIterator != 0) {
			LOG(INFO) << "Shape is a point cloud with type " << pointCloudIterator->getPointCloudTypeName();
		}


		/* Delete the point clouds from previous cycle (if any). Actually visualization looks nicer when deletion of old data occurs right after adding new one */
		wm->scene.deleteNode(lastPointCloudId);
		//wm->scene.deleteNode(lastFilteredPointCloudId);

		/* query world model for relevant box ROI data */
		vector<unsigned int> resultIds;
		brics_3d::rsg::Shape::ShapePtr resultShape;
		brics_3d::rsg::TimeStamp resultTime;
		tmpAttributes.clear();
		tmpAttributes.push_back(brics_3d::rsg::Attribute("name","roi_box"));
		wm->scene.getNodes(tmpAttributes, resultIds); // find node
		assert(resultIds.size() == 1);
		unsigned int boxResultId = resultIds[0];
		LOG(DEBUG) << "Found ID for label roi_box " << boxResultId;

		wm->scene.getGeometry(boxResultId, resultShape, resultTime); // retrieve geometric data
		brics_3d::rsg::Box::BoxPtr resultBox;
		resultBox = boost::dynamic_pointer_cast<brics_3d::rsg::Box>(resultShape);
		assert(resultBox != 0);

		brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44());
		wm->scene.getTransformForNode(boxResultId, wm->scene.getRootId(), brics_3d::rsg::TimeStamp(timer.getCurrentTime()), transform); // get transform data to that node

		/* Create a point cloud based on a previously stored box ROI */
		brics_3d::BoxROIExtractor boxFilter(resultBox->getSizeX(),resultBox->getSizeY(),resultBox->getSizeZ()); // NOTE: each value describes range [origin-value/2, origin+value/2]
		boxFilter.setBoxOrigin(transform);
		brics_3d::PointCloud3D::PointCloud3DPtr boxROIPointCloud(new brics_3d::PointCloud3D());
		brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr boxROIPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
		boxROIPointCloudContainer->data = boxROIPointCloud;
//		boxFilter.filter(newPointCloudContainer->data.get(), boxROIPointCloudContainer->data.get());
		LOG(INFO) << "ROI has " << boxROIPointCloudContainer->data->getSize() << " points";
//		boxROIPointCloudContainer->data->storeToTxtFile("roi_box_filtered_point_cloud.txt");

		tmpAttributes.clear();
		tmpAttributes.push_back(brics_3d::rsg::Attribute("name","roi_box_filtered_point_cloud"));
		unsigned int currentFilteredPointCloudId = 0;
		wm->scene.addGeometricNode(wm->getRootNodeId(), currentFilteredPointCloudId, tmpAttributes, boxROIPointCloudContainer, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
		wm->scene.deleteNode(lastFilteredPointCloudId);


		unsigned int currentMeshId = 0;

		/* just for the fun: move the ROI */
		wm->scene.getNodeParents(boxResultId, resultIds); // we know that the box has only one parent: a transform node
		assert(resultIds.size() == 1);
		unsigned int tfResultId = resultIds[0];
		brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transformNew(new brics_3d::HomogeneousMatrix44());
		*(transformNew.get()) = *(transform.get());
		double* matrixData = transformNew->setRawData(); //get writable data array;

		if(matrixData[14] >= 2.0) { //just some forth and back toggling
			roiDiff *= -1.0;
		}
		if(matrixData[14] <= 0.0) {
			roiDiff *= -1.0;
		}

		matrixData[14] += roiDiff; //[m]
		wm->scene.setTransform(tfResultId, transformNew, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));

		/* Setting hints to delet the correct data in next cycle */
		lastPointCloudId = currentPointCloudId;
		lastFilteredPointCloudId = currentFilteredPointCloudId;
		lastMeshId = currentMeshId;

		count++;
		benchmark->output << totalTimer.getElapsedTime() << std::endl;
		LOG(INFO) << "Total processing cycle took:" << totalTimer.getElapsedTime() << "[ms]" << std::endl;
	}

	/// Kinect interface
	pcl::Grabber* interface;

	/// Helper tool to convert point clouds
	brics_3d::PCLTypecaster converter;

	/// The world model that stores all 3D data.
	brics_3d::WorldModel* wm;

	/// Visualization tool for world model
	brics_3d::rsg::OSGVisualizer* wmObserver;

	/// Hint which point cloud is is from previous cycle.
	unsigned int lastPointCloudId;
	unsigned int lastFilteredPointCloudId;
	unsigned int lastMeshId;

	/// For stats & debugging
	int count;

	/// For dynamic ROI
	double roiDiff;

	/// Various timers
	brics_3d::Timer timer;
	brics_3d::Timer totalTimer;

	brics_3d::Benchmark* benchmark;

	void run () {
	    interface = new pcl::OpenNIGrabber();
	    boost::function<void (const pcl::PointCloud<MyPointType>::ConstPtr&)> f = boost::bind (&WorldModelKinectIteratorTest::callback, this, _1);
	    interface->registerCallback (f);
	    interface->start ();
	}

};

int main(int argc, char **argv) {

	WorldModelKinectIteratorTest wmKinectTest;
	wmKinectTest.run();

    while (!wmKinectTest.wmObserver->done())
    {
      sleep (1);
    }

}

/* EOF */
