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

#include "VisualizationConfiguration.h"

namespace brics_3d {
namespace rsg {

VisualizationConfiguration::VisualizationConfiguration() {

	/* Apply default values: */
	this->visualizeIds = visualizeIdsDefault;
	this->abbreviateIds = abbreviateIdsDefault;
	this->visualizeNameTag = visualizeNameTagDefault;
	this->visualizeAttributes = visualizeAttributesDefault;
	this->visualizeTransformTranslation = visualizeTransformTranslationDefault;
	this->visualizeTransformRotation = visualizeTransformRotationDefault;
	this->visualizeTransformUpdates = visualizeTransformUpdatesDefault;
	this->visualizeTimeStamp = visualizeTimeStampDefault;
	this->visualizePoseUncertainty = visualizePoseUncertaintyDefault;
	this->visualizeRelativePoseUncertainty = visualizeRelativePoseUncertaintyDefault;
	this->visualizeNodes = visualizeNodesDefault;
	this->visualizeGroups = visualizeGroupsDefault;
	this->visualizeTransforms = visualizeTransformsDefault;
	this->visualizeGeometricNodes = visualizeGeometricNodesDefault;

}

VisualizationConfiguration::~VisualizationConfiguration() {

}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
