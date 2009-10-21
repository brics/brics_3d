/**
 * @file
 * brics_3dpm 
 * slam6d_icp_test.cpp
 * 
 * @brief Simple test file to experiment with ICP from 6dslam library
 *
 * @author: Sebastian Blumenthal
 * @date: Sep 9, 2009
 * @version: 0.1
 */

#include <iostream>
#include <util/DepthImageLoader.h>
#include <core/CartesianPointCloud.h>
#include <algorithm/DepthImageToPointCloudTransformation.h>

using namespace brics;

#define WANT_STREAM ///< define the WANT stream :)
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ifstream;

#include "6dslam/src/scan.h"

#include "6dslam/src/icp6Dapx.h"
#include "6dslam/src/icp6Dsvd.h"
#include "6dslam/src/icp6Dquat.h"
#include "6dslam/src/icp6Dortho.h"
#include "6dslam/src/icp6Dhelix.h"
#include "6dslam/src/icp6D.h"
#include "6dslam/src/lum6Deuler.h"
#include "6dslam/src/lum6Dquat.h"
#include "6dslam/src/ghelix6DQ2.h"
#include "6dslam/src/elch6Deuler.h"
#include "6dslam/src/elch6Dquat.h"
#include "6dslam/src/elch6DunitQuat.h"
#include "6dslam/src/elch6Dslerp.h"
#include "6dslam/src/graphSlam6D.h"
#include "6dslam/src/gapx6D.h"
#include "6dslam/src/graph.h"
#include "6dslam/src/globals.icc"

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "..\Visual_Studio_Projects\6DSLAM\6D_SLAM\XGetopt.h"
#endif

#include <csignal>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

void matchGraph6Dautomatic(double cldist, int loopsize, vector <Scan *> allScans, icp6D *my_icp6D, bool meta_icp, bool use_cache, loopSlam6D *my_loopSlam6D, graphSlam6D *my_graphSlam6D, int nrIt, double epsilonSLAM, double mdml, double mdmll, double graphDist, bool &eP, reader_type type);

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
	cout << "Using " << filename1 << " and " << filename2 << endl;

	/* get depth images */
	IplImage* depthImage1;
	IplImage* depthImage2;
	DepthImageLoader *depthImgageLoader = new DepthImageLoader();
	depthImage1 = depthImgageLoader->loadDepthImage(filename1);
	depthImgageLoader->displayDepthImage();
	depthImage2 = depthImgageLoader->loadDepthImage(filename2);
	depthImgageLoader->displayDepthImage();

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

	/* create point cloud container */
	vector<CartesianPointCloud*> *pointClouds = new vector<CartesianPointCloud*>;
	pointClouds->push_back(pointCloud1);
	pointClouds->push_back(pointCloud2);

	/*
	 * NOTE: most stuff taken from slam6D.cc
	 */

	/* set ICP parameters */
	double red = 5; //paremetrize octree (-1.0 is none)
	double mdmll = -1.0, mdml = 25.0, mdm = 25.0;
	int rand = -1, mni = 50;
	int start = 0, end = -1;
	bool quiet = false;
	bool veryQuiet = false;
	int maxDist = -1;
	int minDist = -1;
	bool eP = true; // should we extrapolate the pose??
	bool meta = true; // match against meta scan, or against LAST scan only?
	int algo = 2;
	int mni_lum = -1;
	double cldist = 500;
	int clpairs = -1;
	int loopsize = 20;
	string net = "none";
	int anim = -1;
	double epsilonICP = 0.00001;
	double epsilonSLAM = 0.5;
	bool use_cache = false;
	bool exportPts = false;
	int loopSlam6DAlgo = 0;
	int lum6DAlgo = 0;
	bool exportPoints = true;
	double distLoop = 700.0;
	int iterLoop = 100;
	double graphDist = cldist;
	//reader_type type = UOS;
	reader_type type = DUMMY; //hack
	string dir = "/tmp/";

	//Scan::readScans(type, start, end, dir, maxDist, minDist, true);
	double eu[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	vector<Point> ptss;
	int _fileNr;
	Scan::readScans(type, start, end, dir, maxDist, minDist, false);
	//Scan *scanContainer = new Scan(eu, maxDist);

	/* feed all point clouds into the "Scan" container for 6DSLAM ICP */
	for (int i = 0; i < pointClouds->size(); ++i) {
		Scan *currentScan = new Scan(eu, maxDist);
		Point tmpPoint;
		cout << "Feeding points " << (*pointClouds)[i]->getSize() <<" into scan" << endl;
		for (int j = 0; j < (*pointClouds)[i]->getSize() ;++j) {
			tmpPoint.x = (*(*pointClouds)[i]->getPointCloud())[j].x;
			tmpPoint.y = (*(*pointClouds)[i]->getPointCloud())[j].y;
			tmpPoint.z = (*(*pointClouds)[i]->getPointCloud())[j].z;
			ptss.push_back(tmpPoint);
		}
		assert(ptss.size() == (*pointClouds)[i]->getSize());
		//currentScan->fileNr = i;//TODO?!?

		currentScan->setPoints(ptss);    // copy points
		ptss.clear(); // clear points
		cout << (*currentScan->get_points()).size() << endl;
		Scan::allScans.push_back(currentScan);
	}

	/*
	 * 6DSLAM standard stuff follows here:
	 */
	int end_reduction = (int)Scan::allScans.size();
	#ifdef _OPENMP
	#pragma omp parallel for schedule(dynamic)
	#endif
	for (int iterator = 0; iterator < end_reduction; iterator++) {
		if (red > 0) {
			cout << "Reducing Scan No. " << iterator << endl;
	} else {
			cout << "Copying Scan No. " << iterator << endl;
		}
	    // reduction filter for current scan!
	    Scan::allScans[iterator]->calcReducedPoints(red);
	}

	Scan::createTrees(use_cache);

	 icp6Dminimizer *my_icp6Dminimizer = 0;
	  switch (algo) {
	  case 1 :
	    my_icp6Dminimizer = new icp6D_APX(quiet);
	    break;
	  case 2 :
	    my_icp6Dminimizer = new icp6D_QUAT(quiet);
	    break;
	  case 3 :
	    my_icp6Dminimizer = new icp6D_SVD(quiet);
	    break;
	  case 4 :
	    my_icp6Dminimizer = new icp6D_ORTHO(quiet);
	    break;
	  case 5 :
	    my_icp6Dminimizer = new icp6D_HELIX(quiet);
	    break;
	  }

	  // match the scans and print the time used
	   long starttime = GetCurrentTimeInMilliSec();
	   if (mni_lum == -1 && loopSlam6DAlgo == 0) {
	     icp6D *my_icp = 0;
	     my_icp = new icp6D(my_icp6Dminimizer, mdm, mni, quiet, meta, rand, eP,
	 					    anim, epsilonICP, use_cache);
	     if (my_icp) my_icp->doICP(Scan::allScans);
	     delete my_icp;

	   } else if (clpairs > -1) {
	     //!!!!!!!!!!!!!!!!!!!!!!!!
	     icp6D *my_icp = new icp6D(my_icp6Dminimizer, mdm, mni, quiet, meta, rand, eP,
	 						anim, epsilonICP, use_cache);
	     my_icp->doICP(Scan::allScans);
	     graphSlam6D *my_graphSlam6D = new lum6DEuler(my_icp6Dminimizer, mdm, mdml, mdmll, mni, quiet, meta,
	 									    rand, eP, anim, epsilonICP, use_cache, epsilonSLAM);
	     my_graphSlam6D->matchGraph6Dautomatic(Scan::allScans, mni_lum, clpairs, loopsize);
	     //!!!!!!!!!!!!!!!!!!!!!!!!
	   } else {
	     graphSlam6D *my_graphSlam6D = 0;
	     switch (lum6DAlgo) {
	     case 1 :
	 	 my_graphSlam6D = new lum6DEuler(my_icp6Dminimizer, mdm, mdml, mdmll, mni, quiet, meta, rand, eP,
	 						anim, epsilonICP, use_cache, epsilonSLAM);
	 	 break;
	     case 2 :
	       my_graphSlam6D = new lum6DQuat(my_icp6Dminimizer, mdm, mdml, mdmll, mni, quiet, meta, rand, eP,
	 					    anim, epsilonICP, use_cache, epsilonSLAM);
	       break;
	     case 3 :
	       my_graphSlam6D = new ghelix6DQ2(my_icp6Dminimizer, mdm, mdml, mdmll, mni, quiet, meta, rand, eP,
	 						anim, epsilonICP, use_cache, epsilonSLAM);
	 	 break;
	     case 4 :
	       my_graphSlam6D = new gapx6D(my_icp6Dminimizer, mdm, mdml, mdmll, mni, quiet, meta, rand, eP,
	 					anim, epsilonICP, use_cache, epsilonSLAM);
	       break;
	     }
	     // Construct Network
	     if (net != "none") {
	 	 icp6D *my_icp = new icp6D(my_icp6Dminimizer, mdm, mni, quiet, meta, rand, eP,
	 						  anim, epsilonICP, use_cache);
	 	 my_icp->doICP(Scan::allScans);

	 	 Graph* structure;
	 	 structure = new Graph(net);
	 	 my_graphSlam6D->doGraphSlam6D(*structure, Scan::allScans, mni_lum);
	 	 mdml = mdmll;
	 	 my_graphSlam6D->doGraphSlam6D(*structure, Scan::allScans, mni_lum);
	     } else {
	       icp6D *my_icp = 0;
	       if(algo > 0) {
	         my_icp = new icp6D(my_icp6Dminimizer, mdm, mni, quiet, meta, rand, eP, anim, epsilonICP, use_cache);
	       }

	       loopSlam6D *my_loopSlam6D = 0;
	       switch(loopSlam6DAlgo) {
	         case 1:
	           my_loopSlam6D = new elch6Deuler(veryQuiet, my_icp6Dminimizer, distLoop, iterLoop, rand, eP, 10, epsilonICP, use_cache);
	           break;
	         case 2:
	           my_loopSlam6D = new elch6Dquat(veryQuiet, my_icp6Dminimizer, distLoop, iterLoop, rand, eP, 10, epsilonICP, use_cache);
	           break;
	         case 3:
	           my_loopSlam6D = new elch6DunitQuat(veryQuiet, my_icp6Dminimizer, distLoop, iterLoop, rand, eP, 10, epsilonICP, use_cache);
	           break;
	         case 4:
	           my_loopSlam6D = new elch6Dslerp(veryQuiet, my_icp6Dminimizer, distLoop, iterLoop, rand, eP, 10, epsilonICP, use_cache);
	           break;
	       }

	       matchGraph6Dautomatic(cldist, loopsize, Scan::allScans, my_icp, meta, use_cache, my_loopSlam6D, my_graphSlam6D, mni_lum, epsilonSLAM, mdml, mdmll, graphDist, eP, type);
	       delete my_icp;
	       if(loopSlam6DAlgo > 0) {
	         delete my_loopSlam6D;
	       }
	     }
	     delete my_graphSlam6D;
	   }

	   long endtime = GetCurrentTimeInMilliSec() - starttime;
	   cout << "Matching done in " << endtime << " milliseconds!!!" << endl;

	   CartesianPointCloud *registeredPoints = new CartesianPointCloud();
	   double x,y,z = 0.0;
	   if (exportPoints) {
	     cout << "Export all 3D Points" << endl;
	     ofstream redptsout("points.pts");
	     for(unsigned int i = 0; i < Scan::allScans.size(); i++) {
			 for (int j = 0; j < Scan::allScans[i]->get_points_red_size(); j++) {
				 redptsout << Scan::allScans[i]->get_points_red()[j][0] << " "
						 << Scan::allScans[i]->get_points_red()[j][1] << " "
						 << Scan::allScans[i]->get_points_red()[j][2] << endl;

				x = Scan::allScans[i]->get_points_red()[j][0];
				y = Scan::allScans[i]->get_points_red()[j][1];
				z =	Scan::allScans[i]->get_points_red()[j][2];
				registeredPoints->addPoint(Point3D(x,y,z));
			 }

	     }
	     redptsout.close();
	     redptsout.clear();
	     registeredPoints->storeToPlyFile("registeredPoints.ply");
	     registeredPoints->storeToTxtFile("registeredPoints.txt");
	   }

	   cout << "Saving registration information in .frames files" << endl;
	   vector <Scan*>::iterator Iter = Scan::allScans.begin();
	   for( ; Iter != Scan::allScans.end(); ) {
	     Iter = Scan::allScans.begin();
	     delete (*Iter);
	     cout << ".";
	     cout.flush();
	   }
	   Scan::allScans.clear();

	   delete my_icp6Dminimizer;

	   cout << endl << endl;
	   cout << "Normal program end." << endl;
}

/**
 * This function is does all the matching stuff
 * it iterates over all scans using the algorithm objects to calculate new poses
 * objects could be NULL if algorithm should not be used
 *
 * @param cldist maximal distance for closing loops
 * @param loopsize minimal loop size
 * @param allScans Contains all laser scans
 * @param my_icp6D the ICP implementation
 * @param meta_icp math ICP against a metascan
 * @param use_cache Indicates if cached versions of the search tree has to be build
 * @param my_loopSlam6D used loopoptimizer
 * @param my_graphSlam6D used global optimization
 * @param nrIt The number of iterations the global SLAM-algorithm will run
 * @param epsilonSLAM epsilon for global SLAM iteration
 * @param mdml maximal distance match for global SLAM
 * @param mdmll maximal distance match for global SLAM after all scans ar matched
 */
void matchGraph6Dautomatic(double cldist, int loopsize, vector <Scan *> allScans, icp6D *my_icp6D, bool meta_icp, bool use_cache, loopSlam6D *my_loopSlam6D, graphSlam6D *my_graphSlam6D, int nrIt, double epsilonSLAM, double mdml, double mdmll, double graphDist, bool &eP, reader_type type)
{
  double cldist2 = sqr(cldist);

  // list of scan for metascan
  vector < Scan* > metas;

  // graph for loop optimization
  graph_t g;

  int n = allScans.size();

  int loop_detection = 0;
  double dist, min_dist = -1;
  int first = 0, last = 0;

  allScans[0]->mergeCoordinatesWithRoboterPosition();

  for(int i = 1; i < n; i++) {
    cout << i << "/" << n << endl;

    add_edge(i-1, i, g);

    if(eP) {
      allScans[i]->mergeCoordinatesWithRoboterPosition(allScans[i-1]);
    } else {
      allScans[i]->mergeCoordinatesWithRoboterPosition();
    }

    //Hack to get all icp transformations into the .frames Files
    if(i == n-1 && my_icp6D != NULL && my_icp6D->get_anim() == -2) {
      my_icp6D->set_anim(-1);
    }

    /*if(i == 85 || i == 321 || i == 533) {
      my_icp6D->set_anim(1);
    }*/

    if(my_icp6D != NULL){
      cout << "ICP" << endl;
      // Matching strongly linked scans with ICPs
      if(meta_icp) {
        metas.push_back(allScans[i - 1]);
        Scan *meta_scan = new Scan(metas, use_cache);
        my_icp6D->match(meta_scan, allScans[i]);
        delete meta_scan;
      } else {
        switch(type) {
          case UOS_MAP:
          case UOS_MAP_FRAMES:
            my_icp6D->match(allScans[0], allScans[i]);
            break;
          case RTS_MAP:
            //untested (and could not work)
            //if(i < 220-22 && i > 250-22) match(allScans[0], CurrentScan);
            my_icp6D->match(allScans[0], allScans[i]);
            break;
          default:
            my_icp6D->match(allScans[i - 1], allScans[i]);
            break;
        }
      }
    } else {
      double id[16];
      M4identity(id);
      allScans[i]->transform(id, Scan::ICP, 0);
    }

    /*if(i == 85 || i == 321 || i == 533) {
      my_icp6D->set_anim(-2);
    }*/

    if(loop_detection == 1) {
      loop_detection = 2;
    }

    for(int j = 0; j < i - loopsize; j++) {
      dist = Dist2(allScans[j]->get_rPos(), allScans[i]->get_rPos());
      if(dist < cldist2) {
        loop_detection = 1;
        if(min_dist < 0 || dist < min_dist) {
          min_dist = dist;
          first = j;
          last = i;
        }
      }
    }

    if(loop_detection == 2) {
      loop_detection = 0;
      min_dist = -1;

      if(my_loopSlam6D != NULL) {
        cout << "Loop close: " << first << " " << last << endl;
        my_loopSlam6D->close_loop(allScans, first, last, g);
        add_edge(first, last, g);
      }

      if(my_graphSlam6D != NULL && mdml > 0) {
        int j = 0;
        double ret;
        do {
          // recalculate graph
          Graph *gr = new Graph(i + 1, cldist2, loopsize);
          cout << "Global: " << j << endl;
          ret = my_graphSlam6D->doGraphSlam6D(*gr, allScans, 1);
          delete gr;
          j++;
        } while (j < nrIt && ret > epsilonSLAM);
      }
    }
  }

  if(loop_detection == 1 && my_loopSlam6D != NULL) {
    cout << "Loop close: " << first << " " << last << endl;
    my_loopSlam6D->close_loop(allScans, first, last, g);
    add_edge(first, last, g);
  }

  if(my_graphSlam6D != NULL && mdml > 0.0) {
    int j = 0;
    double ret;
    do {
      // recalculate graph
      Graph *gr = new Graph(n, cldist2, loopsize);
      cout << "Global: " << j << endl;
      ret = my_graphSlam6D->doGraphSlam6D(*gr, allScans, 1);
      delete gr;
      j++;
    } while (j < nrIt && ret > epsilonSLAM);
  }

  if(my_graphSlam6D != NULL && mdmll > 0.0) {
    my_graphSlam6D->set_mdmll(mdmll);
    int j = 0;
    double ret;
    do {
      // recalculate graph
      Graph *gr = new Graph(n, sqr(graphDist), loopsize);
      cout << "Global: " << j << endl;
      ret = my_graphSlam6D->doGraphSlam6D(*gr, allScans, 1);
      delete gr;
      j++;
    } while (j < nrIt && ret > epsilonSLAM);
  }
}

/* EOF */
