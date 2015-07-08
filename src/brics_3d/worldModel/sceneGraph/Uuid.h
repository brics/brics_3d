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

#ifndef RSG_UUID_H_
#define RSG_UUID_H_

#include <stdint.h>
#include <algorithm>
#include <string>

namespace brics_3d {
namespace rsg {

/**
 * @brief A universally unique identifier (UUID) according to RFC 4122
 *
 * Also sometimes refereed as globally unique identifier (GUID).
 * The code is inspired by boost:uuid
 *
 * @ingroup sceneGraph
 */
class Uuid {

public:

	// Helpers to treat array as iterators. This will bring in some nice std algorithms
    typedef uint8_t value_type;
    typedef uint8_t& reference;
    typedef uint8_t const& const_reference;
    typedef uint8_t* iterator;
    typedef uint8_t const* const_iterator;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    size_t size() const { return arraySize; }
	iterator begin() { return data; }
	const_iterator begin() const { return data; }
	iterator end() { return data+size(); }
	const_iterator end() const { return data+size(); }

	/* Constructors */
	Uuid();
	Uuid(Uuid & other);
	Uuid(const Uuid & other);
	Uuid(unsigned int id);
	virtual ~Uuid();

	/* Assignment */
	Uuid& operator=(Uuid & other);
	Uuid& operator=(const Uuid& other);

	/* Additional assignment for testability */
	Uuid& operator=(unsigned int other);

	/* (in) equality operators  */
	bool operator ==(const Uuid& other) const;
	bool operator !=(const Uuid& other) const;
	bool operator <(const Uuid& other) const;
	bool operator >(const Uuid& other) const;
	bool operator <=(const Uuid& other) const;
	bool operator >=(const Uuid& other) const;
	void swap(Uuid& other);
	void swap(Uuid& lhs, Uuid& rhs);

	/**
	 * @brief Test for null value.
	 * "The nil UUID is special form of UUID that is specified to have all
	 *  128 bits set to zero."
	 * @return True if value equals null/null definition.
	 */
	bool isNil() const;

	/**
	 * @brief Set data to nil/null value.
	 * @see isNil()
	 */
	void setToNil();

	/**
	 * Convert a UUID into a string.
	 * @return A string in hhhhhhhh-hhhh-hhhh-hhhh-hhhhhhhhhhhh format.
	 */
	std::string toString() const;

	/**
	 * Convert a string into a UUID.
	 * @param uuid Sting in hhhhhhhh-hhhh-hhhh-hhhh-hhhhhhhhhhhh format.
	 * @return True on success. False otherwise. On failure, the data is set to NIL.
	 */
	bool fromString(std::string uuid);

	/**
	 * @brief Overridden << operator.
	 *
	 * Writes a UUId to a stream
	 *
	 * @param outStream The output stream
	 * @param id Reference to UUID which data is forwarded to output stream
	 * @return Output stream
	 */
	friend std::ostream& operator<<(std::ostream& outStream, const Uuid& id);

	/// Size of array is 16. 16 * 8 = 128 bits as required by RFC 4122
    const static size_t arraySize = 16;

private:

	/**
	 * @brief Array holding the 128 bit value for an UUID
	 *
	 * The semantics is described in detail here: http://www.ietf.org/rfc/rfc4122.txt
	 *
	 * @code
	 *    0                   1                   2                   3
	 *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *    |                          time_low                             |
	 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *    |       time_mid                |         time_hi_and_version   |
	 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *    |clk_seq_hi_res |  clk_seq_low  |         node (0-1)            |
	 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *    |                         node (2-5)                            |
	 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 * @endcode
	 *
	 * That results in an array index mapping of:
	 *
	 * - index 0 = time_low
	 * - index 1 = time_low
	 * - index 2 = time_low
	 * - index 3 = time_low
	 * - index 4 = time_mid
	 * - index 5 = time_mid
	 * - index 6 = time_hi_and_version
	 * - index 7 = time_hi_and_version
	 * - index 8 = clk_seq_hi_res
	 * - index 9 = clk_seq_low
	 * - index 10 = node (0-1)
	 * - index 11 = node (0-1)
	 * - index 12 = node (2-5)
	 * - index 13 = node (2-5)
	 * - index 14 = node (2-5)
	 * - index 15 = node (2-5)
	 *
	 * Each field encoded with the Most Significant Byte first (network byte order).
	 *
	 * The string version looks like
	 * @code
	 * {xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}
	 *
	 * {time_low-tmid-th+v-clks-xxxxnodexxxx}
	 * @endcode
	 *
	 */
	uint8_t data[arraySize];

};

/** Helper function for backward compatibility
 *
 * The function will take the last four bytes and reinterpret them as
 * an unsigned integer. The other bytes are ignored.
 *
 * @note This function will cause loss of information. Use with care.
 *
 * @param id The Uuid to be converted
 * @return The last 4 bytes of the UUID but interpreted as unsigend int
 */
extern unsigned int uuidToUnsignedInt (Uuid id);

/// Only for backward compatibility
extern unsigned int uuidToUnsignedInt (unsigned int id);

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_UUID_H_ */

/* EOF */
