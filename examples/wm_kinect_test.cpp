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
#include <core/HomogeneousMatrix44.h>
#include <core/TriangleMeshExplicit.h>
#include <core/Logger.h>
#include <algorithm/filtering/BoxROIExtractor.h>
#include <algorithm/segmentation/RegionBasedSACSegmentation.h>
#include <algorithm/segmentation/EuclideanClustering.h>
#include <algorithm/segmentation/EuclideanClusteringPCL.h>
#include <algorithm/meshGeneration/DelaunayTriangulationOSG.h>
#include <util/PCLTypecaster.h>
#include <util/Timer.h>
#include <worldModel/WorldModel.h>
#include <worldModel/sceneGraph/PointCloud.h>
#include <worldModel/sceneGraph/Mesh.h>
#include <worldModel/sceneGraph/Box.h>
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
		BRICS_3D::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new BRICS_3D::HomogeneousMatrix44(&transformation));

		unsigned int tfBox1Id = 0;
		unsigned int Box1Id = 0;
		vector<BRICS_3D::RSG::Attribute> tmpAttributes;
		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","roi_box_tf"));
		wm->scene.addTransformNode(wm->getRootNodeId(), tfBox1Id, tmpAttributes, transform, BRICS_3D::RSG::TimeStamp(timer.getCurrentTime()));

		BRICS_3D::RSG::Box::BoxPtr box1(new BRICS_3D::RSG::Box(xHalfSeize, yHalfSeize, zHalfSeize));
		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","roi_box"));
		wm->scene.addGeometricNode(tfBox1Id, Box1Id, tmpAttributes, box1, BRICS_3D::RSG::TimeStamp(timer.getCurrentTime()));
		LOG(DEBUG) << "ROI Box added with ID " << Box1Id;

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
		unsigned int currentPointCloudId = 0;
		vector<BRICS_3D::RSG::Attribute> tmpAttributes;
		tmpAttributes.push_back(Attribute("name","raw_point_cloud"));
		wm->scene.addGeometricNode(wm->getRootNodeId(), currentPointCloudId, tmpAttributes, newPointCloudContainer, TimeStamp(timer.getCurrentTime()));

		/* Delete the point clouds from previous cycle (if any). Actually visualization looks nicer when deletion of old data occurs right after adding new one */
		wm->scene.deleteNode(lastPointCloudId);
		//wm->scene.deleteNode(lastFilteredPointCloudId);

		/* query world model for relevant box ROI data */
		vector<unsigned int> resultIds;
		Shape::ShapePtr resultShape;
		TimeStamp resultTime;
		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","roi_box"));
		wm->scene.getNodes(tmpAttributes, resultIds); // find node
		assert(resultIds.size() == 1);
		unsigned int boxResultId = resultIds[0];
		LOG(DEBUG) << "Found ID for label roi_box " << boxResultId;

		wm->scene.getGeometry(boxResultId, resultShape, resultTime); // retrieve geometric data
		BRICS_3D::RSG::Box::BoxPtr resultBox;
		resultBox = boost::dynamic_pointer_cast<BRICS_3D::RSG::Box>(resultShape);
		assert(resultBox != 0);

		BRICS_3D::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new BRICS_3D::HomogeneousMatrix44());
		wm->scene.getTransformForNode(boxResultId, BRICS_3D::RSG::TimeStamp(timer.getCurrentTime()), transform); // get transform data to that node

		/* Create a point cloud based on a previously stored box ROI */
		BRICS_3D::BoxROIExtractor boxFilter(resultBox->getSizeX(),resultBox->getSizeY(),resultBox->getSizeZ()); // NOTE: each value describes range [origin-value/2, origin+value/2]
		boxFilter.setBoxOrigin(transform);
		BRICS_3D::PointCloud3D::PointCloud3DPtr boxROIPointCloud(new BRICS_3D::PointCloud3D());
		BRICS_3D::RSG::PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr boxROIPointCloudContainer(new BRICS_3D::RSG::PointCloud<BRICS_3D::PointCloud3D>());
		boxROIPointCloudContainer->data = boxROIPointCloud;
		boxFilter.filter(newPointCloudContainer->data.get(), boxROIPointCloudContainer->data.get());
		LOG(INFO) << "ROI has " << boxROIPointCloudContainer->data->getSize() << " points";
//		boxROIPointCloudContainer->data->storeToTxtFile("roi_box_filtered_point_cloud.txt");

		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","roi_box_filtered_point_cloud"));
		unsigned int currentFilteredPointCloudId = 0;
		wm->scene.addGeometricNode(wm->getRootNodeId(), currentFilteredPointCloudId, tmpAttributes, boxROIPointCloudContainer, TimeStamp(timer.getCurrentTime()));
		wm->scene.deleteNode(lastFilteredPointCloudId);

		/* Get the dominant plane */
//		timer.reset();
//		Eigen::VectorXd modelCoefficients;
//		std::vector<int> inliers;
//		BRICS_3D::RegionBasedSACSegmentation sacSegmenter;
//
//		sacSegmenter.setPointCloud(boxROIPointCloudContainer->data.get());
//		sacSegmenter.setDistanceThreshold(0.02);
//		sacSegmenter.setMaxIterations(1000);
//		sacSegmenter.setMethodType(sacSegmenter.SAC_RANSAC);
//		sacSegmenter.setModelType(sacSegmenter.OBJMODEL_PLANE);
//		sacSegmenter.setProbability(0.99);
//		sacSegmenter.segment();
//		sacSegmenter.getInliers(inliers);
//		sacSegmenter.getModelCoefficients(modelCoefficients);
//
//		cout << "Timer: Plane segmantation took " << timer.getElapsedTime() << "[ms]" << endl;
//		cout << "Found Inliers: " << inliers.size() << endl;
//		cout << "The model-coefficients are: " << endl << modelCoefficients << endl;

//		/* create some mesh */
//		BRICS_3D::ITriangleMesh::ITriangleMeshPtr newMesh(new BRICS_3D::TriangleMeshExplicit());
//		BRICS_3D::RSG::Mesh<BRICS_3D::ITriangleMesh>::MeshPtr newMeshContainer(new BRICS_3D::RSG::Mesh<BRICS_3D::ITriangleMesh>());
//		newMeshContainer->data = newMesh;
//
//		BRICS_3D::DelaunayTriangulationOSG meshGenerator;
//		meshGenerator.triangulate(boxROIPointCloudContainer->data.get(), newMeshContainer->data.get());
//		LOG(INFO) << "Number of generated triangles: " << newMeshContainer->data->getSize();
//		tmpAttributes.clear();
//		tmpAttributes.push_back(Attribute("name","mesh_1"));
		unsigned int currentMeshId = 0;
//		wm->scene.addGeometricNode(wm->scene.getRootId(), currentMeshId, tmpAttributes, newMeshContainer, TimeStamp(timer.getCurrentTime()));
//		wm->scene.deleteNode(lastMeshId);

		/* just for the fun: move the ROI */
		wm->scene.getNodeParents(boxResultId, resultIds); // we know that the box has only one parent: a transform node
		assert(resultIds.size() == 1);
		unsigned int tfResultId = resultIds[0];
		BRICS_3D::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transformNew(new BRICS_3D::HomogeneousMatrix44());
		*(transformNew.get()) = *(transform.get());
		double* matrixData = transformNew->setRawData(); //get writable data array;

		if(matrixData[14] >= 2.0) { //just some forth and back toggling
			roiDiff *= -1.0;
		}
		if(matrixData[14] <= 0.0) {
			roiDiff *= -1.0;
		}

		matrixData[14] += roiDiff; //[m]
		wm->scene.setTransform(tfResultId, transformNew, BRICS_3D::RSG::TimeStamp(timer.getCurrentTime()));

		/* Setting hints to delet the correct data in next cycle */
		lastPointCloudId = currentPointCloudId;
		lastFilteredPointCloudId = currentFilteredPointCloudId;
		lastMeshId = currentMeshId;

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
	unsigned int lastFilteredPointCloudId;
	unsigned int lastMeshId;

	/// For stats & debugging
	int count;

	/// For dynamic ROI
	double roiDiff;

	BRICS_3D::Timer timer;

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
