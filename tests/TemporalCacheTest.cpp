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

#include "TemporalCacheTest.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( TemporalCacheTest );

void TemporalCacheTest::setUp() {

}

void TemporalCacheTest::tearDown() {

}

void TemporalCacheTest::testTimeStamps() {

	TimeStamp t1(0.0);
	CPPUNIT_ASSERT(t1 == t1);
	CPPUNIT_ASSERT(TimeStamp(0.0) == t1);
	CPPUNIT_ASSERT(TimeStamp(0.0) == TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) > TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) >= TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(0.0) < TimeStamp(1.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) >= TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) != TimeStamp(0.0));

	TimeStamp t2(0.0, Units::Second);
	CPPUNIT_ASSERT(t2 == t1);
	CPPUNIT_ASSERT(TimeStamp(0.0) == t2);
	CPPUNIT_ASSERT(TimeStamp(0.0, Units::Second) == TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) > TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) >= TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(0.0, Units::Second) < TimeStamp(1.0));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) >= TimeStamp(0.0));

	t1 += TimeStamp(10.0, Units::Second);
	CPPUNIT_ASSERT(TimeStamp(10.0) == t1);
	t1 -= TimeStamp(5.0, Units::Second);
	CPPUNIT_ASSERT(TimeStamp(5.0) == t1);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1.0, Units::Second).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-9, TimeStamp(1.0, Units::NanoSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-6, TimeStamp(1.0, Units::MicroSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, TimeStamp(1.0, Units::MilliSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(60.0, TimeStamp(1.0, Units::Minute).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3600.0, TimeStamp(1.0, Units::Hour).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(86400.0, TimeStamp(1.0, Units::Day).getSeconds(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, 1e9 * 1e-9, maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1e9, Units::NanoSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1e6, Units::MicroSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1e3, Units::MilliSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1.0/60.0, Units::Minute).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1.0/3600.0, Units::Hour).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1.0/86400.0, Units::Day).getSeconds(), maxTolerance);

	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) > TimeStamp(1e8, Units::NanoSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) < TimeStamp(1e10, Units::NanoSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) > TimeStamp(1e5, Units::MicroSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) < TimeStamp(1e7, Units::MicroSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) > TimeStamp(1e2, Units::MilliSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) < TimeStamp(1e4, Units::MilliSecond));
	CPPUNIT_ASSERT(TimeStamp(60.0, Units::Second) > TimeStamp(0.9, Units::Minute));
	CPPUNIT_ASSERT(TimeStamp(60.0, Units::Second) < TimeStamp(1.1, Units::Minute));
	CPPUNIT_ASSERT(TimeStamp(60.0*60.0, Units::Second) > TimeStamp(0.9, Units::Hour));
	CPPUNIT_ASSERT(TimeStamp(60.0*60.0, Units::Second) < TimeStamp(1.1, Units::Hour));
	CPPUNIT_ASSERT(TimeStamp(60.0*60.0*24.0, Units::Second) > TimeStamp(0.9, Units::Day));
	CPPUNIT_ASSERT(TimeStamp(60.0*60.0*24.0, Units::Second) < TimeStamp(1.1, Units::Day));

	TimeStamp stamp1(10); 					// No explicit unit type means SI units: 10[s]
	TimeStamp stamp2(10, Units::Second);
	TimeStamp stamp3(10e9, Units::NanoSecond);
	TimeStamp stamp4(10e6, Units::MicroSecond);
	TimeStamp stamp5(1e3, Units::MilliSecond);
	TimeStamp stamp6(10.0/60.0, Units::Minute);
	TimeStamp stamp7(10.0/3600.0, Units::Hour);
	TimeStamp stamp8(10.0/86400.0, Units::Day);
}

void TemporalCacheTest::testSimpleCache() {
	TemporalCache<double> simpleCache;

	double data;

	CPPUNIT_ASSERT_EQUAL(0u, simpleCache.getNumberOfCacheEntries());
	simpleCache.insertData(324.8, TimeStamp(0.0, Units::Second));
	CPPUNIT_ASSERT_EQUAL(1u, simpleCache.getNumberOfCacheEntries());
	data = simpleCache.getData(TimeStamp(0.0, Units::Second));
	CPPUNIT_ASSERT_DOUBLES_EQUAL(324.8, data, maxTolerance);

	TemporalCache<double> simpleCache2(simpleCache); // copy constructor
	CPPUNIT_ASSERT_EQUAL(1u, simpleCache2.getNumberOfCacheEntries());
	data = simpleCache2.getData(TimeStamp(0.0, Units::Second));
	CPPUNIT_ASSERT_DOUBLES_EQUAL(324.8, data, maxTolerance);

	/* flush the first cache */
	simpleCache.deleteOutdatedData(TimeStamp (1, Units::Minute));
	CPPUNIT_ASSERT_EQUAL(0u, simpleCache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT_EQUAL(1u, simpleCache2.getNumberOfCacheEntries());
	data = simpleCache2.getData(TimeStamp(0.0, Units::Second));
	CPPUNIT_ASSERT_DOUBLES_EQUAL(324.8, data, maxTolerance);
}

void TemporalCacheTest::testCacheInsertions() {
	TemporalCache<int> cache(TimeStamp(20, Units::Second)); // cache size = 20[s]
	int cacheEntry = 0; // We will put values in ascending order in the cache: 0,1,2,...

	/* initial condition */
	CPPUNIT_ASSERT_EQUAL(0u, cache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT(TimeStamp(0.0, Units::Second) == cache.getLatestTimeStamp()-cache.getOldestTimeStamp());

	/* insert some data */
	cache.insertData(cacheEntry, TimeStamp(-2, Units::Second)); // entry = 0
	CPPUNIT_ASSERT_EQUAL(1u, cache.getNumberOfCacheEntries());
	cache.insertData(++cacheEntry, TimeStamp(0, Units::Second)); // entry = 1
	cache.insertData(++cacheEntry, TimeStamp(10.0, Units::Second)); // entry = 2
	cache.insertData(++cacheEntry, TimeStamp(15.0, Units::Second)); // entry = 3
	CPPUNIT_ASSERT_EQUAL(4u, cache.getNumberOfCacheEntries());

	CPPUNIT_ASSERT(TimeStamp(20.0, Units::Second) == cache.getMaxHistoryDuration());
	CPPUNIT_ASSERT(cache.getOldestTimeStamp() == TimeStamp(-2, Units::Second));
	CPPUNIT_ASSERT(cache.getLatestTimeStamp() == TimeStamp(15, Units::Second));
	TimeStamp t1 = cache.getOldestTimeStamp();
	TimeStamp t2 = cache.getLatestTimeStamp();
	TimeStamp t3 = t2 - t1;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(17.0, t3.getSeconds(), maxTolerance); // 17s overall duration within cache
	CPPUNIT_ASSERT_DOUBLES_EQUAL(17.0, (cache.getLatestTimeStamp()-cache.getOldestTimeStamp()).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT(TimeStamp(17.0, Units::Second) == cache.getLatestTimeStamp()-cache.getOldestTimeStamp());

	/* access via itarators */
	int counter = 3;
	typedef std::vector<std::pair<int, TimeStamp> >::const_iterator IntCacheIterator;
	for (IntCacheIterator iterator = cache.begin(); iterator!=cache.end(); ++iterator) {
		//std::cout << "iterator (value, stamp): (" << iterator->first << "," << iterator->second.getSeconds() << ")" << std::endl;
		CPPUNIT_ASSERT_EQUAL(counter, iterator->first);
		//iterator->first = 0; // does not work because access is read only
		counter--;
	}

	counter = 0;
	typedef std::vector<std::pair<int, TimeStamp> >::const_reverse_iterator IntCacheReverseIterator;
	for (IntCacheReverseIterator iterator = cache.rbegin(); iterator!=cache.rend(); ++iterator) {
		//std::cout << "reverse iterator (value, stamp): (" << iterator->first << "," << iterator->second.getSeconds() << ")" << std::endl;
		CPPUNIT_ASSERT_EQUAL(counter, iterator->first);
		counter++;
	}

	/* some queries based on seconds within cache range */
	CPPUNIT_ASSERT_EQUAL(0, cache.getData(TimeStamp(-2, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(0, cache.getData(TimeStamp(-1.00001, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(1, cache.getData(TimeStamp(-1, Units::Second))); //ambigious case case -> the newer one will be taken
	CPPUNIT_ASSERT_EQUAL(1, cache.getData(TimeStamp(-0.999, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(1, cache.getData(TimeStamp(-0.999, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(1, cache.getData(TimeStamp(0, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(1, cache.getData(TimeStamp(4, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(2, cache.getData(TimeStamp(5, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(2, cache.getData(TimeStamp(10, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(2, cache.getData(TimeStamp(12, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(3, cache.getData(TimeStamp(12.5, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(3, cache.getData(TimeStamp(15, Units::Second)));

	/* queries beyond cache limits - still it will return the "closest element" */
	CPPUNIT_ASSERT_EQUAL(0, cache.getData(TimeStamp(-20, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(3, cache.getData(TimeStamp(20, Units::Second)));

	/* queries with other units */
	CPPUNIT_ASSERT_EQUAL(1, cache.getData(TimeStamp(0, Units::MilliSecond)));
	CPPUNIT_ASSERT_EQUAL(2, cache.getData(TimeStamp(10*1000, Units::MilliSecond)));
	CPPUNIT_ASSERT_EQUAL(1, cache.getData(TimeStamp(0, Units::Day)));
	CPPUNIT_ASSERT_EQUAL(3, cache.getData(TimeStamp(1, Units::Day)));



	/* insertion that will delete oldest entry */
	cache.insertData(++cacheEntry, TimeStamp(20.0, Units::Second)); // entry = 4
	CPPUNIT_ASSERT_EQUAL(4u, cache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT(cache.getOldestTimeStamp() == TimeStamp(0, Units::Second));
	CPPUNIT_ASSERT(cache.getLatestTimeStamp() == TimeStamp(20, Units::Second));
	CPPUNIT_ASSERT_EQUAL(1, cache.getData(TimeStamp(-2, Units::Second))); // entry 0 is deleted so it will be redirected to 1
	CPPUNIT_ASSERT_EQUAL(1, cache.getData(TimeStamp(0, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(2, cache.getData(TimeStamp(10, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(3, cache.getData(TimeStamp(15, Units::Second)));
	CPPUNIT_ASSERT_EQUAL(4, cache.getData(TimeStamp(20, Units::Second)));

	/* access via itarators */
	counter = 4;
	typedef std::vector<std::pair<int, TimeStamp> >::const_iterator IntCacheIterator;
	for (IntCacheIterator iterator = cache.begin(); iterator!=cache.end(); ++iterator) {
		//std::cout << "iterator (value, stamp): (" << iterator->first << "," << iterator->second.getSeconds() << ")" << std::endl;
		CPPUNIT_ASSERT_EQUAL(counter, iterator->first);
		counter--;
	}

	counter = 1;
	typedef std::vector<std::pair<int, TimeStamp> >::const_reverse_iterator IntCacheReverseIterator;
	for (IntCacheReverseIterator iterator = cache.rbegin(); iterator!=cache.rend(); ++iterator) {
		//std::cout << "reverse iterator (value, stamp): (" << iterator->first << "," << iterator->second.getSeconds() << ")" << std::endl;
		CPPUNIT_ASSERT_EQUAL(counter, iterator->first);
		counter++;
	}


	/* insertion that will delete all entrys  except for latest */
	cache.insertData(++cacheEntry, TimeStamp(100.0, Units::Second)); // entry = 5
	CPPUNIT_ASSERT_EQUAL(1u, cache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT_EQUAL(5, cache.getData(TimeStamp(100, Units::Second)));

	/* allign cache with "current time" */
	cache.deleteOutdatedData(TimeStamp(101, Units::Second)); // no effect
	CPPUNIT_ASSERT_EQUAL(1u, cache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT_EQUAL(5, cache.getData(TimeStamp(100, Units::Second)));

	/* flush cache */
	cache.deleteOutdatedData(cache.getLatestTimeStamp() + cache.getMaxHistoryDuration()); // limit case
	CPPUNIT_ASSERT_EQUAL(1u, cache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT_EQUAL(5, cache.getData(TimeStamp(100, Units::Second)));
	cache.deleteOutdatedData(cache.getLatestTimeStamp() + cache.getMaxHistoryDuration() + TimeStamp(1, Units::MicroSecond)); // limit case + a bit
	CPPUNIT_ASSERT_EQUAL(0u, cache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT_EQUAL(0, cache.getData(TimeStamp(100, Units::Second))); // 0 is error case

	/* restart with random order */
	cache.insertData(2, TimeStamp(2, Units::Second));
	cache.insertData(1, TimeStamp(1, Units::Second));
	cache.insertData(4, TimeStamp(4, Units::Second));
	cache.insertData(5, TimeStamp(5, Units::Second));
	cache.insertData(3, TimeStamp(3, Units::Second));

	CPPUNIT_ASSERT_EQUAL(5u, cache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT(cache.getOldestTimeStamp() == TimeStamp(1, Units::Second));
	CPPUNIT_ASSERT(cache.getLatestTimeStamp() == TimeStamp(5, Units::Second));

	counter = 1;
	typedef std::vector<std::pair<int, TimeStamp> >::const_reverse_iterator IntCacheReverseIterator;
	for (IntCacheReverseIterator iterator = cache.rbegin(); iterator!=cache.rend(); ++iterator) {
		//std::cout << "reverse iterator (value, stamp): (" << iterator->first << "," << iterator->second.getSeconds() << ")" << std::endl;
		CPPUNIT_ASSERT_EQUAL(counter, iterator->first);
		counter++;
	}

	/*
	 * something very special: create a new cache with the last two values shifted by 2[s]
	 */
	TimeStamp startForShift(3.5, Units::Second);
	TimeStamp shift(2.0 , Units::Second);
	TemporalCache<int> shiftedCache(cache.getMaxHistoryDuration());

	for (IntCacheReverseIterator iterator = cache.rbegin(); iterator!=cache.rend(); ++iterator) {
		//std::cout << "reverse iterator (value, stamp): (" << iterator->first << "," << iterator->second.getSeconds() << ")" << std::endl;
		if (iterator->second > startForShift) {
			shiftedCache.insertData(iterator->first, iterator->second + shift);
		} else {
			shiftedCache.insertData(iterator->first, iterator->second);
		}
	}

	CPPUNIT_ASSERT_EQUAL(5u, shiftedCache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT(shiftedCache.getOldestTimeStamp() == TimeStamp(1, Units::Second));
	CPPUNIT_ASSERT(shiftedCache.getLatestTimeStamp() == TimeStamp(7, Units::Second));

	counter = 1;
	for (IntCacheReverseIterator iterator = shiftedCache.rbegin(); iterator!=shiftedCache.rend(); ++iterator) {
		//std::cout << "reverse iterator (value, stamp): (" << iterator->first << "," << iterator->second.getSeconds() << ")" << std::endl;
		CPPUNIT_ASSERT_EQUAL(counter, iterator->first); // values are still the same, just soeme times stamps are shifted
		counter++;
	}

	shiftedCache.clear();
	CPPUNIT_ASSERT_EQUAL(0u, shiftedCache.getNumberOfCacheEntries());
	CPPUNIT_ASSERT(shiftedCache.getOldestTimeStamp() == TimeStamp(0.0, Units::Second));
	CPPUNIT_ASSERT(shiftedCache.getLatestTimeStamp() == TimeStamp(0.0, Units::Second));

}

void TemporalCacheTest::testCacheConfiguration() {
	TemporalCache<int> cache1; // cache size = 10[s]
	TemporalCache<int> cache2(TimeStamp(20, Units::Second)); // cache size = 20[s]
	TemporalCache<int> cache3(TimeStamp(-20, Units::Second)); // error -> will be ignored and thus reset set to default

	CPPUNIT_ASSERT(TimeStamp(10.0, Units::Second) == cache1.getMaxHistoryDuration());
	CPPUNIT_ASSERT(TimeStamp(20.0, Units::Second) == cache2.getMaxHistoryDuration());
	CPPUNIT_ASSERT(TimeStamp(0.0, Units::Second) == cache3.getMaxHistoryDuration());

	cache1.setMaxHistoryDuration(TimeStamp(-30.0, Units::Second)); // will be ignored
	CPPUNIT_ASSERT(TimeStamp(10.0, Units::Second) == cache1.getMaxHistoryDuration());

	cache2.setMaxHistoryDuration(TimeStamp(-30.0, Units::Second)); // will be ignored
	CPPUNIT_ASSERT(TimeStamp(20.0, Units::Second) == cache2.getMaxHistoryDuration());
	cache2.setMaxHistoryDuration(TimeStamp(30.0, Units::Second));
	CPPUNIT_ASSERT(TimeStamp(30.0, Units::Second) == cache2.getMaxHistoryDuration());
	cache2.setMaxHistoryDuration(TimeStamp(-30.0, Units::Second)); // will be ignored
	CPPUNIT_ASSERT(TimeStamp(30.0, Units::Second) == cache2.getMaxHistoryDuration());

	cache3.setMaxHistoryDuration(TimeStamp(-30.0, Units::Second)); // will be ignored
	CPPUNIT_ASSERT(TimeStamp(0.0, Units::Second) == cache3.getMaxHistoryDuration());
}

}

/* EOF */
