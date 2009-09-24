/**
 * @file
 * icp_test.cpp
 * 
 * @brief Simple test file to experiment with ICP from IVT library
 *
 * @author: Sebastian Blumenthal
 * @date: Aug 31, 2009
 * @version: 0.1
 */

#include <iostream>
#include <util/DepthImageLoader.h>
#include <core/CartesianPointCloud.h>
#include <algorithm/DepthImageToPointCloudTransformation.h>

#include "Math/Math3d.h"
#include "Tracking/ICP.h"

using namespace std;
using namespace brics;

//#include "Image/ImageProcessor.h"
//#include "Image/ByteImage.h"
//#include <stdio.h>

int main(int argc, char **argv) {

	/* check argument */
	if (argc != 3) {
		cerr << "Usage: " << argv[0] << " <filename>" << endl;
		cerr << "Try for example: " << argv[0]
				<< " ../examples/test_data/depth_images/zcam_param1c.pgm"
				<< " ../examples/test_data/depth_images/zcam_param4a.pgm" << endl;
		return -1;
	}
	string filename1 = argv[1];
	string filename2 = argv[2];
	cout << filename1 << endl;

	/* get depth images */
	IplImage* depthImage1;
	IplImage* depthImage2;
	DepthImageLoader *depthImgageLoader = new DepthImageLoader();
	depthImage1 = depthImgageLoader->loadDepthImage(filename1);
	depthImgageLoader->diplayDepthImage();
	depthImage2 = depthImgageLoader->loadDepthImage(filename2);
	depthImgageLoader->diplayDepthImage();

	/* convert to point clouds */
	CartesianPointCloud *pointCloud1 = new CartesianPointCloud();
	CartesianPointCloud *pointCloud2 = new CartesianPointCloud();
	DepthImageToPointCloudTransformation *img2cloudTramsformer =
			new DepthImageToPointCloudTransformation();
	img2cloudTramsformer->transformDepthImageToPointCloud(depthImage1,
			pointCloud1, 10);
	img2cloudTramsformer->transformDepthImageToPointCloud(depthImage2,
			pointCloud2, 10);
	cout << "Size of point cloud1: " << pointCloud1->getSize() << endl;
	cout << "Size of point cloud2: " << pointCloud2->getSize() << endl;
	pointCloud1->storeToPlyFile("scan001.ply");
	pointCloud2->storeToPlyFile("scan002.ply");

	/*
	 * Test1: compute for same point cloud
	 */
	//	Vec3d pSourcePoints[] = { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 } };
	//	Vec3d pTargetPoints[] = { { 0, 0, 42 }, { 1, 0, 42 }, { 2, 0, 42 } };
	//	int nPoints = 3;
	int nPoints1 = pointCloud1->getSize();
	Vec3d pSourcePoints1[nPoints1];
	Vec3d pTargetPoints1[nPoints1]; //yes both are the same... => no translation/rotation

	/* feed point cloud into ICP structs */
	for (int i = 0; i < nPoints1; ++i) {
		pSourcePoints1[i].x = (*pointCloud1->getPointCloud())[i].x;
		pSourcePoints1[i].y = (*pointCloud1->getPointCloud())[i].y;
		pSourcePoints1[i].z = (*pointCloud1->getPointCloud())[i].z;

		pTargetPoints1[i].x = (*pointCloud1->getPointCloud())[i].x;
		pTargetPoints1[i].y = (*pointCloud1->getPointCloud())[i].y;
		pTargetPoints1[i].z = (*pointCloud1->getPointCloud())[i].z;
	}

	Mat3d resultRotation1;
	Vec3d resutTranslation1;

	CICP *icp = new CICP();
	icp->CalculateOptimalTransformation(pSourcePoints1, pTargetPoints1, nPoints1,
			resultRotation1, resutTranslation1);

	cout << "Results for Test1 (same point clouds) " << endl;
	cout << "rotation: ";
	cout << resultRotation1.r1 << ", ";
	cout << resultRotation1.r2 << ", ";
	cout << resultRotation1.r3 << ", ";
	cout << resultRotation1.r4 << ", ";
	cout << resultRotation1.r5 << ", ";
	cout << resultRotation1.r6 << ", ";
	cout << resultRotation1.r7 << ", ";
	cout << resultRotation1.r8 << ", ";
	cout << resultRotation1.r9 << endl;

	cout << "translation: ";
	cout << resutTranslation1.x << ", ";
	cout << resutTranslation1.y << ", ";
	cout << resutTranslation1.z << endl;

	/*
	 * Test2: compute for different point clouds
	 */
	int nPoints2 = pointCloud1->getSize(); //TODO: which one is correct? size 1 or 2??!?
	Vec3d pSourcePoints2[pointCloud1->getSize()];
	Vec3d pTargetPoints2[pointCloud2->getSize()];

	/* feed point cloud into ICP structs */
	for (int i = 0; i < pointCloud1->getSize(); ++i) {
		pSourcePoints2[i].x = (*pointCloud1->getPointCloud())[i].x;
		pSourcePoints2[i].y = (*pointCloud1->getPointCloud())[i].y;
		pSourcePoints2[i].z = (*pointCloud1->getPointCloud())[i].z;
	}

	for (int i = 0; i < pointCloud2->getSize(); ++i) {
		pTargetPoints2[i].x = (*pointCloud2->getPointCloud())[i].x;
		pTargetPoints2[i].y = (*pointCloud2->getPointCloud())[i].y;
		pTargetPoints2[i].z = (*pointCloud2->getPointCloud())[i].z;
	}

	Mat3d resultRotation2;
	Vec3d resutTranslation2;

	//CICP *icp = new CICP();
	icp->CalculateOptimalTransformation(pSourcePoints2, pTargetPoints2, nPoints2,
			resultRotation2, resutTranslation2);

	cout << "Results for Test2 (different point clouds) " << endl;
	cout << "rotation: ";
	cout << resultRotation2.r1 << ", ";
	cout << resultRotation2.r2 << ", ";
	cout << resultRotation2.r3 << ", ";
	cout << resultRotation2.r4 << ", ";
	cout << resultRotation2.r5 << ", ";
	cout << resultRotation2.r6 << ", ";
	cout << resultRotation2.r7 << ", ";
	cout << resultRotation2.r8 << ", ";
	cout << resultRotation2.r9 << endl;

	cout << "translation: ";
	cout << resutTranslation2.x << ", ";
	cout << resutTranslation2.y << ", ";
	cout << resutTranslation2.z << endl;

	/* store results */
	CartesianPointCloud *pointCloudOut = new CartesianPointCloud();
	for (int i = 0; i < pointCloud1->getSize(); ++i) {
		pointCloudOut->addPoint(CartesianPoint3D(
				pSourcePoints2[i].x,
				pSourcePoints2[i].y,
				pSourcePoints2[i].z));
	}

	Vec3d tmpPoint;
	for (int i = 0; i < pointCloud2->getSize(); ++i) {
		//Math3d::TransformVec(pTargetPoints2[i], &resultRotation2, &resutTranslation2, &tmpPoint);
		//Math3d::TransformVecYZX()


		pointCloudOut->addPoint(CartesianPoint3D(
				pTargetPoints2[i].x,
				pTargetPoints2[i].y,
				pTargetPoints2[i].z));
	}
	assert (pointCloudOut->getSize() == (pointCloud1->getSize()+pointCloud2->getSize()));
	pointCloudOut->storeToTxtFile("test_point_cloud_icp.txt");
	pointCloudOut->storeToPlyFile("test_point_cloud_icp.ply");

	/* clean up */
	delete icp;
	delete pointCloud1;
	delete pointCloud2;
	delete depthImgageLoader;
	delete img2cloudTramsformer;

	return 0;
}

/* EOF */
