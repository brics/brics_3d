/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2014, KU Leuven
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

#ifndef RSG_VISUALIZATIONCONFIGURATION_H_
#define RSG_VISUALIZATIONCONFIGURATION_H_

namespace brics_3d {
namespace rsg {

/**
 * @brief A configuration aggregation on what a particular visualizer should display or not.
 * @ingroup sceneGraph
 */
class VisualizationConfiguration {
public:

	/**
	 * @brief Default constructor that will initialize the given default values.
	 */
	VisualizationConfiguration();

	/**
	 * @brief Default destructor.
	 */
	virtual ~VisualizationConfiguration();

	/// If yes the IDs will be shown
	bool visualizeIds;

	/// If yes IDs will be abbreviated. Will be ignored if visualizeIds = false.
	bool abbreviateIds;

	/// If yes show attribute containing "name" as key.
	bool visualizeNameTag;

	/// If yes show all attributes.
	bool visualizeAttributes;

	/// If yes show translation part of a transform node.
	bool visualizeTransformTranslation;

	/// If yes show rotation part of a transform node.
	bool visualizeTransformRotation;

	/// If yes show number of cache entries of a transform node.
	bool visualizeTransformUpdates;

	/// If yes show (latest) time stamp for a (uncertain) transform or geometric node.
	bool visualizeTimeStamp;

	/// If yes show covariance ellipsoid for an uncertain transform node.
	bool visualizePoseUncertainty;

	/// If yes show covariance ellipses with respict to parent node.
	/// Will be ignored if visualizePoseUncertainty = false.
	bool visualizeRelativePoseUncertainty;

	// Default values
	static const bool visualizeIdsDefault = true;
	static const bool abbreviateIdsDefault = true;
	static const bool visualizeNameTagDefault = false;
	static const bool visualizeAttributesDefault = true;
	static const bool visualizeTransformTranslationDefault = true;
	static const bool visualizeTransformRotationDefault = true;
	static const bool visualizeTransformUpdatesDefault = true;
	static const bool visualizeTimeStampDefault = false;
	static const bool visualizePoseUncertaintyDefault = false;
	static const bool visualizeRelativePoseUncertaintyDefault = false;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_VISUALIZATIONCONFIGURATION_H_ */

/* EOF */
