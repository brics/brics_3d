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

#ifndef RSG_FREQUENCYAWAREUPDATEFILTER_H_
#define RSG_FREQUENCYAWAREUPDATEFILTER_H_

#include <brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h>
#include <brics_3d/worldModel/sceneGraph/Attribute.h>
#include <brics_3d/util/Timer.h>

namespace brics_3d {
namespace rsg {

class FrequencyAwareUpdateFilter : public ISceneGraphUpdateObserver {
public:
	FrequencyAwareUpdateFilter();
	virtual ~FrequencyAwareUpdateFilter();

	/* implemetntations of observer interface */
	bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false);
    bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId = false);
	bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false);
	bool addRemoteRootNode(Id rootId, vector<Attribute> attributes);
	bool addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId = false);
	bool setNodeAttributes(Id id, vector<Attribute> newAttributes);
	bool setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
    bool setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp);
	bool deleteNode(Id id);
	bool addParent(Id id, Id parentId);
    bool removeParent(Id id, Id parentId);

    bool attachUpdateObserver(ISceneGraphUpdateObserver* observer);
    bool detachUpdateObserver(ISceneGraphUpdateObserver* observer);

	double getMaxGeometricNodeUpdateFrequency() const {
		return maxGeometricNodeUpdateFrequency;
	}

	void setMaxGeometricNodeUpdateFrequency(
			double maxGeometricNodeUpdateFrequency) {
		this->maxGeometricNodeUpdateFrequency = maxGeometricNodeUpdateFrequency;
		if(maxGeometricNodeUpdateFrequency > 0) {
			this->maxGeometricNodeUpdateDuration = Duration (1.0 / maxGeometricNodeUpdateFrequency, Units::Second);
		} else {
			this->maxGeometricNodeUpdateDuration = Duration (0, Units::Second);
		}
	}

	double getMaxTransformUpdateFrequency() const {
		return maxTransformUpdateFrequency;
	}

	void setMaxTransformUpdateFrequency(double maxTransformUpdateFrequency) {
		this->maxTransformUpdateFrequency = maxTransformUpdateFrequency;
		if (maxTransformUpdateFrequency > 0) {
			this->maxTransformUpdateDuration = Duration (1.0 / maxTransformUpdateFrequency, Units::Second);
		} else {
			this->maxTransformUpdateDuration = Duration (0);
		}
	}

private:

	/// Updates for setTransform and setUncertainTransform with a higher rate (API invocation) than this treshold are omitted.
	/// Negative values allow to send everithing.
	/// Unit is [Hz]
    double maxTransformUpdateFrequency;

	/// Updates for addGeometricNode with a higher rate  (API invocation) than this treshold are omitted.
	/// Negative values allow to send everithing.
	/// Unit is [Hz]
    double maxGeometricNodeUpdateFrequency;

    Duration maxTransformUpdateDuration;
    Duration maxGeometricNodeUpdateDuration;

    TimeStamp lastTransformUpdate;
    TimeStamp lastGeometricNodeUpdate;

    /// Set of observers that will be notified when the update function will be called and the LoD contraints are satisfied.
    std::vector<ISceneGraphUpdateObserver*> updateObservers;

	/// Timer for benchmarking (Same as used in world model).
    brics_3d::Timer timer;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_FREQUENCYAWAREUPDATEFILTER_H_ */

/* EOF */
