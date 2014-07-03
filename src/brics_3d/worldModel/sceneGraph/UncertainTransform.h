/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2013, KU Leuven
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

#ifndef RSG_UNCERTAINTRANSFORM_H_
#define RSG_UNCERTAINTRANSFORM_H_

#include "Transform.h"
#include "brics_3d/core/ITransformUncertainty.h"

namespace brics_3d {

namespace rsg {

typedef std::vector< std::pair<ITransformUncertainty::ITransformUncertaintyPtr, TimeStamp> >::iterator UncertaintyHistoryIterator;

/**
 * Specialaization for IHomogeneousMatrix44::IHomogeneousMatrix44Ptr type as
 * error case is supposed to return a (shared) null pointer rather than 0.
 */
template<>
inline ITransformUncertainty::ITransformUncertaintyPtr TemporalCache<ITransformUncertainty::ITransformUncertaintyPtr>::returnNullData() {
	return ITransformUncertainty::ITransformUncertaintyPtr(); // should be kind of null...
}

/**
 * @brief A node that expresses a geometric transformation accompanied by uncertenty information between its parents and children.
 *
 * The UncertainTransform enhances the Transform represenataion by uncertainty information
 * on the geometric relation (pose). The data will be cached similar to the transform data itself. Thus a new
 * cache is added. The time stamps for the tnrasform and the uncertainty cache associate the two
 * individual caches.
 *
 * It is possible to insert fresh transform data without uncertatinty - in that case the latest found
 * uncertainty data will be taken and associate with the trnasform data.
 *
 * @ingroup sceneGraph
 */
class UncertainTransform : public Transform {
public:

	typedef boost::shared_ptr<rsg::UncertainTransform> UncertainTransformPtr;
	typedef boost::shared_ptr<rsg::UncertainTransform const> UncertainTransformConstPtr;

	/**
	 * @brief Constructor.
	 * @param maxHistoryDuration Optional parameter to define the temporal cache size.
	 *        Default is 10[s]. This parameter will be feed forward to the Trnasform
	 *        cache (super class) as well.
	 */
	UncertainTransform(TimeStamp maxHistoryDuration = TimeStamp(10, Units::Second));

	/**
	 * @brief Default destructor
	 */
	virtual ~UncertainTransform();

    /**
     * @brief Add a new transform plus accompanying uncertainty data to the history.
     * @param newTransform The transform to be added.
     * @param newUncertainty The transform uncertainty associated with the newTransform to be added.
     * @param timeStamp The times stamp that is associated with the transform and the uncertainty.
     * @return False if an entry with this time stamp already exists. In this case data will not be inserted.
	 *         Otherwise true.
     */
    bool insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, ITransformUncertainty::ITransformUncertaintyPtr newUncertainty, TimeStamp timeStamp);

    /**
     * @brief Add a new transform without accompanying uncertainty data to the history.
     * This method overrides the method from the Transform class.
     *
     * @param newTransform The transform to be added.
     * @param timeStamp The times stamp that is associated with the transform.
     * @return False if an entry with this time stamp already exists. In this case data will not be inserted.
	 *         Otherwise true.
	 *
     * @note The individual caches might get out of sync. I.e. as the uncertainty cache does not get updated
     * it temporal range might lack behind until next invocation of
     * insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, ITransformUncertainty::ITransformUncertaintyPtr newUncertainty, TimeStamp timeStamp)
     *
     */
    bool insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, TimeStamp timeStamp);

    /**
     * @brief Retrieve a transform in the history cache whose time stamp matches best a given stamp.
     * @param timeStamp Time stamp to wich the temporal closest transform shall be found.
     * @return Shared pointer to the transform uncertainty. It will be null in case that no transform is found.
     */
    ITransformUncertainty::ITransformUncertaintyPtr getTransformUncertainty(TimeStamp timeStamp);

    /**
     * @brief Retrieve the latest/newest transform uncertainty.
     * The uncertainty data might be older the the transfrom it self.
     * @return Shared pointer to the transform uncertainty. It will be null in case that no transform is found.
     */
    ITransformUncertainty::ITransformUncertaintyPtr getLatestTransformUncertainty();

    /**
     * @brief Retrieve the latest times stamp from the internal uncertainty data cache.
     * The value might differ from Transform::getLatestTimeStamp()
     * @return The latest time stamp or 0.0 in case of an empty history cache.
     */
    TimeStamp getLatestTransformUncertaintyTimeStamp();

    /**
     * @brief Retrieve the oldest times stamp from the internal uncertainty data cache.
     * The value might differ from Transform::getOldestTimeStamp()
     * @return The oldest time stamp or 0.0 in case of an empty history cache.
     */
    TimeStamp getOldestTransformUncertaintyTimeStamp();

    /**
     * @brief Returns the number of elements that are stored in the uncertainty history cache.
     */
    unsigned int getCurrentUncertaintyHistoryLength();

    /**
     * @brief Setter for maxHistoryDuration wich defined the temporal cache size.
     *
     *
     * We have to override the Trsnsform::setMaxHistoryDuration() function
     * such that we dont miss new parameters for the second uncertainty cache.
     * @param maxHistoryDuration New cache size for _both_ caches - the transform and
     *        the uncertainty cache.
     */
    void setMaxHistoryDuration(TimeStamp maxHistoryDuration);

    /**
     * @brief Delete all data from the uncertainty history/cache that is older than the latestTimeStamp minus the duration of maxHistoryDuration.
     * @param latestTimeStamp @see Transform::deleteOutdatedTransforms()
     */
    void deleteOutdatedTransformUncertainties(TimeStamp latestTimeStamp);

    /**
     * Overrides method from Transform calass to ensure that the unsertainty caches is updated as well.
     */
    void deleteOutdatedTransforms(TimeStamp latestTimeStamp);

private:

    /**
     * Cache for timestamped uncertainty data. The time stamps allow to associate the
     * uncertainty date with the transforms in the transsform cache-
     */
    TemporalCache<ITransformUncertainty::ITransformUncertaintyPtr> uncertaintyHistory;

    /**
     *  Counter for how often this uncertain transform node has been updated via insertTransform.
     *  This counter might differ from updateCount of the Transform class in case
     *  insertTransform method data with and without (cf. super class) uncertainty
     *  is invoked.
     */
    unsigned int uncertaintyUpdateCount;

};

}

}

#endif /* RSG_UNCERTAINTRANSFORM_H_ */

/* EOF */
