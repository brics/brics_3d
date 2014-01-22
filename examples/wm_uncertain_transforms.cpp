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

/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/CovarianceMatrix66.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/PointCloud.h>
#include <brics_3d/worldModel/sceneGraph/Mesh.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/Cylinder.h>
#include <brics_3d/worldModel/sceneGraph/DotGraphGenerator.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/UuidGenerator.h>
#include <brics_3d/worldModel/sceneGraph/SimpleIdGenerator.h>
#include <brics_3d/core/ParameterSet.h>


/* general includes */
#include <iostream>
#include <cstring>
#include <fstream>
#include <sstream>

using namespace std;
using namespace brics_3d;

int main(int argc, char **argv) {

	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	/* Graph structure:
	 *                 root
	 *                   |
	 *        -----------+--------------------------
	 *        |          |          |         |     |
	 *       tf1        tf2        tf4       tf5   tf7(w/o uncert.)
	 *       			 |          |         |
	 *       		    tf3         + ---+----+
	 *                                  tf6
	 */
	rsg::Id tf1Id;
	rsg::Id tf2Id;
	rsg::Id tf3Id;
	rsg::Id tf4Id;
	rsg::Id tf5Id;
	rsg::Id tf6Id;
	rsg::Id tf7Id;
	rsg::Id tf8Id;
	vector<rsg::Attribute> tmpAttributes;

	rsg::IIdGenerator* idGenerator = new rsg::SimpleIdGenerator();//rsg::UuidGenerator();
	WorldModel* wm = new WorldModel(idGenerator);

	rsg::OSGVisualizer* wmObserver = new rsg::OSGVisualizer();
	rsg::VisualizationConfiguration osgConfiguration; // optional configuration
	osgConfiguration.visualizeAttributes = true;
	osgConfiguration.visualizeIds = true;
	osgConfiguration.visualizeNameTag = true;
	osgConfiguration.abbreviateIds = true;
	osgConfiguration.visualizePoseUncertainty = true;
	wmObserver->setConfig(osgConfiguration);
	wm->scene.attachUpdateObserver(wmObserver); //enable visualization

	rsg::DotVisualizer* dbgObserver = new rsg::DotVisualizer(&wm->scene);
	rsg::VisualizationConfiguration dotConfiguration; // optional configuration
	dotConfiguration.abbreviateIds = true;
	dbgObserver->setConfig(dotConfiguration);
	wm->scene.attachUpdateObserver(dbgObserver);

	/* test data */
	Eigen::AngleAxis<double> rotation(0.25 * M_PI, Eigen::Vector3d(1,0,0/*1*/));
	Transform3d transformation;
	transformation = Eigen::Affine3d::Identity();
	transformation.translate(Eigen::Vector3d(2,2,0));
	transformation.rotate(rotation);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform234_b(new HomogeneousMatrix44(&transformation));
	LOG(INFO) << "transform234_b" << std::endl << *transform234_b;

	Eigen::AngleAxis<double> rotation2(-0.25 * M_PI, Eigen::Vector3d(1,0,0/*0,0,1*/));
	Transform3d transformation2;
	transformation2 = Eigen::Affine3d::Identity();
	transformation2.translate(Eigen::Vector3d(0,4,0));
	transformation2.rotate(rotation2);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform345_b(new HomogeneousMatrix44(&transformation2));
	LOG(INFO) << "transform345_b" << std::endl << *transform345_b;

	Eigen::AngleAxis<double> rotation3(-0.0 * M_PI, Eigen::Vector3d(0,0,1));
	Transform3d transformation3;
	transformation3 = Eigen::Affine3d::Identity();
	transformation3.translate(Eigen::Vector3d(0,3,0));
	transformation3.rotate(rotation3);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform3(new HomogeneousMatrix44(&transformation2)); //tf8
	LOG(INFO) << "transform3" << std::endl << *transform3;

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,0,0)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform234(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             2,0,0)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform345(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0,2,0)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             10,0.1,-0.2)); //Translation coefficients
//	0.809017 -0.587785 0 2
//	0.587785 0.809017 0 2
//	0 0 1 0
//	0 0 0 1

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform567(new HomogeneousMatrix44());
	HomogeneousMatrix44::xyzRollPitchYawToMatrix(13,0.2,-0.1, 0.5,0,0, transform567);
	double x,y,z,roll,pitch,yaw;
	HomogeneousMatrix44::matrixToXyzRollPitchYaw(transform567, x,y,z,roll,pitch,yaw);
	LOG(INFO) << "transform567: x,y,z,roll,pitch,yaw " << x << ", " << y<< ", " <<z<< ", " <<roll<< ", " <<pitch<< ", " <<yaw;

	HomogeneousMatrix44::matrixToXyzRollPitchYaw(transform234_b, x,y,z,roll,pitch,yaw);
	LOG(INFO) << "transform234_b: x,y,z,roll,pitch,yaw " << x << ", " << y<< ", " <<z<< ", " <<roll<< ", " <<pitch<< ", " <<yaw;

	HomogeneousMatrix44::matrixToXyzRollPitchYaw(transform345_b, x,y,z,roll,pitch,yaw);
	LOG(INFO) << "transform345_b: x,y,z,roll,pitch,yaw " << x << ", " << y<< ", " <<z<< ", " <<roll<< ", " <<pitch<< ", " <<yaw;

//	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform567(new HomogeneousMatrix44(0.809017,-0.587785,0,  //Rotation coefficients
//			                                                     0.587785,0.809017,0,
//	                                                             0,0,1,
//	                                                             13,0.2,-0.1)); //Translation coefficients
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform678(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0,2,0)); //Translation coefficients

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform789(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0,5.1,0)); //Translation coefficients

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform890(new HomogeneousMatrix44(1,0,0,  //Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             14,-4,0)); //Translation coefficients

	ITransformUncertainty::ITransformUncertaintyPtr uncertainty123(new CovarianceMatrix66(3, 0.001, 0.001, 0.0000, 0.000000, 0.00000));
//	ITransformUncertainty::ITransformUncertaintyPtr uncertainty234(new CovarianceMatrix66(0.001, 1, 0.01, 	0, 0, 0));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty234(new CovarianceMatrix66(0.002, 0.002, 1.002, 	0.000, 0.000/*?*/, 0.00));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty345(new CovarianceMatrix66(0.001, 0.001, 1, 	0, 0, 0));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty456(new CovarianceMatrix66(0.1,0.002,2.003, 2.0,4.001,5.0)); // merged 1
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty567(new CovarianceMatrix66(0.1,1.0,2.001, 0.0003,0.0013,5.0)); // merged 2
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty678(new CovarianceMatrix66(0.1,0.1,3.0, 0.94,0.95,0.96));
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty789(new CovarianceMatrix66(0.41,0.42,0.73, 0.94,0.95,0.96)); //update
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty3(new CovarianceMatrix66(3.00, 0.00,0.00, 0,0,0)); //tf8

	/* test compounding */
	CovarianceMatrix66::CovarianceMatrix66Ptr compoundUncertainty;
//	compoundUncertainty = compoundCovariance(transform234_b,
//			/*uncertaintyCov234*/ boost::dynamic_pointer_cast<CovarianceMatrix66>(uncertainty234),
//			transform345_b,
//			/*uncertaintyCov345*/  boost::dynamic_pointer_cast<CovarianceMatrix66>(uncertainty123));
	compoundUncertainty = compoundCovariance(transform345_b,
			/*uncertaintyCov234*/ boost::dynamic_pointer_cast<CovarianceMatrix66>(uncertainty123),
					transform234_b,
			/*uncertaintyCov345*/  boost::dynamic_pointer_cast<CovarianceMatrix66>(uncertainty234));
	LOG(INFO) << "check: transform234_b" << std::endl << *transform234_b;
	LOG(INFO) << "check: transform345_b" << std::endl << *transform345_b;


	/* test merging */
	CovarianceMatrix66::CovarianceMatrix66Ptr mergedUncertainty(new CovarianceMatrix66());
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr mergedMean(new HomogeneousMatrix44());
	mergeCovariance(transform456,
			boost::dynamic_pointer_cast<CovarianceMatrix66>(uncertainty456),
			transform567,
			boost::dynamic_pointer_cast<CovarianceMatrix66>(uncertainty567),
			mergedMean,
			mergedUncertainty);

	/* construct the scene graph */
	tmpAttributes.push_back(rsg::Attribute("name","transform123"));
	tmpAttributes.push_back(rsg::Attribute("dbgInfo","u_transform"));
	wm->scene.addUncertainTransformNode(wm->getRootNodeId(), tf1Id, tmpAttributes, transform123, uncertainty123, rsg::TimeStamp(0.0));
	tmpAttributes.clear();
	wm->scene.addUncertainTransformNode(wm->getRootNodeId(), tf2Id, tmpAttributes, transform234_b, uncertainty234, rsg::TimeStamp(0.0));
	wm->scene.addUncertainTransformNode(tf2Id, tf3Id, tmpAttributes, transform345_b, compoundUncertainty /*uncertainty345*/, rsg::TimeStamp(0.0));
	wm->scene.addUncertainTransformNode(wm->getRootNodeId(), tf4Id, tmpAttributes, transform456, uncertainty456, rsg::TimeStamp(0.0));
	wm->scene.addUncertainTransformNode(wm->getRootNodeId(), tf5Id, tmpAttributes, transform567, uncertainty567, rsg::TimeStamp(0.0));
	wm->scene.addUncertainTransformNode(tf4Id, tf6Id, tmpAttributes, mergedMean/*transform678*/, mergedUncertainty/*uncertainty678*/, rsg::TimeStamp(0.0));
	wm->scene.addParent(tf6Id, tf5Id);
	wm->scene.addTransformNode(wm->getRootNodeId(), tf7Id, tmpAttributes, transform890, rsg::TimeStamp(0.0));

	/* test accumulated compounding */
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;
	wm->scene.getTransformForNode(tf3Id, wm->getRootNodeId(), rsg::TimeStamp(1.0), resultTransform);
	CovarianceMatrix66::CovarianceMatrix66Ptr compoundUncertaintyAccumulated;
//	compoundUncertaintyAccumulated = compoundCovariance(resultTransform,
//			/*uncertaintyCov234*/ boost::dynamic_pointer_cast<CovarianceMatrix66>(compoundUncertainty),
//			transform3,
//			/*uncertaintyCov345*/  boost::dynamic_pointer_cast<CovarianceMatrix66>(uncertainty3));
	compoundUncertaintyAccumulated = compoundCovariance(transform3,
			/*uncertaintyCov234*/ boost::dynamic_pointer_cast<CovarianceMatrix66>(uncertainty3),
			resultTransform,
			/*uncertaintyCov345*/  boost::dynamic_pointer_cast<CovarianceMatrix66>(compoundUncertainty));
	wm->scene.addUncertainTransformNode(tf3Id, tf8Id, tmpAttributes, transform3, compoundUncertaintyAccumulated /*uncertainty345*/, rsg::TimeStamp(0.0));


	/* perform an update */
//	wm->scene.setUncertainTransform(tf6Id, transform789, uncertainty789, rsg::TimeStamp(1.0));

	/* perform queries */
//	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr resultTransform;
	wm->scene.getTransformForNode(tf2Id, wm->getRootNodeId(), rsg::TimeStamp(1.0), resultTransform);
	LOG(INFO) << "Transform root to tf2 is " << std::endl << *resultTransform;

	wm->scene.getTransformForNode(tf3Id, tf2Id, rsg::TimeStamp(1.0), resultTransform);
	LOG(INFO) << "Transform tf2 to tf3 is " << std::endl << *resultTransform;

	wm->scene.getTransformForNode(tf3Id, wm->getRootNodeId(), rsg::TimeStamp(1.0), resultTransform);
	LOG(INFO) << "Transform root to tf3 is " << std::endl << *resultTransform;


	while(!wmObserver->done()) { // wait until user closes the GUI
		//nothing here
	}

	delete wmObserver;
	delete wm;

	return 0;
}

/* EOF */
