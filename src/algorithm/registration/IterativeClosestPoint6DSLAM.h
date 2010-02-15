/**
 * @file 
 * IterativeClosestPoint6DSLAM.h
 *
 * @date: Nov 27, 2009
 * @author: sblume
 */

#ifndef ITERATIVECLOSESTPOINT6DSLAM_H_
#define ITERATIVECLOSESTPOINT6DSLAM_H_

#include "algorithm/registration/IIterativeClosestPoint.h"
#include "algorithm/registration/IIterativeClosestPointSetup.h"
#include <string>

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Wrapper class for 6DSLAM ICP implementation
 */
class IterativeClosestPoint6DSLAM : public IIterativeClosestPoint, public IIterativeClosestPointSetup {
public:

	/**
	 * @brief Standard constructor
	 */
	IterativeClosestPoint6DSLAM();

	/**
	 * @brief Standard destructor
	 */
	virtual ~IterativeClosestPoint6DSLAM();

	void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation);

private:

	double getConvergenceThreshold() const;

	int getMaxIterations() const;

	void setConvergenceThreshold(double convergenceThreshold);

	void setMaxIterations(int maxIterations);

	/// Defines the maximum amount of iterations for matching process
	int maxIterations;

	/// The threshold to define convergence.
	double convergenceThreshold;

	/* set ICP parameters */
	 ///Parameterize octree (-1.0 is none)
	double red;
	double mdmll;
	double	mdml;
	double	mdm;
	int rand;
	int mni;
	int start;
	int end;
	bool quiet;
	bool veryQuiet;
	int maxDist;
	int minDist;
	/// Should we extrapolate the pose??
	bool eP;
	/// Match against meta scan, or against LAST scan only?
	bool meta;
	int algo;
	int mni_lum;
	double cldist;
	int clpairs;
	int loopsize;
	std::string net;
	int anim;	/// Defines the maximum amount of iterations for matching process
	double epsilonICP;
	double epsilonSLAM;
	bool use_cache;
	bool exportPts;
	int loopSlam6DAlgo;
	int lum6DAlgo;
	bool exportPoints;
	double distLoop;
	int iterLoop;
	double graphDist;

};

}

#endif /* ITERATIVECLOSESTPOINT6DSLAM_H_ */

/* EOF */
