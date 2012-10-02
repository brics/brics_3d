/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
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

#include <math.h>
#include <stdint.h>

#ifndef COLORSPACECONVERTOR_H_
#define COLORSPACECONVERTOR_H_

namespace brics_3d {

class ColorSpaceConvertor {
private:

	/**
	 * Minumum error to compare double-values
	 */
	double epsilon;

	/**
	 * Finds the minimum out of three double values
	 * @param a
	 * @param b
	 * @param c
	 * @return	The minimum value
	 */
	inline double minVal(double a, double b, double c){

		if(a < b){
			if(a < c){
				return a;
			} else {
				return c;
			}
		} else {
			if(b < c){
				return b;
			} else {
				return c;
			}
		}

	}


	/**
	 * Finds the maximum out of three double values
	 * @param a
	 * @param b
	 * @param c
	 * @return	The maximum value
	 */
	inline double maxVal(double a, double b, double c){

		if(a > b){
			if(a > c){
				return a;
			} else {
				return c;
			}
		} else {
			if(b > c){
				return b;
			} else {
				return c;
			}
		}

	}

public:


	ColorSpaceConvertor();


	virtual ~ColorSpaceConvertor();


	/**
	 *
     	R, G and B are scaled to fit 0..1 range

     	V <- max(R,G,B)
     	S <- (V-min(R,G,B))/V   if V≠0, 0 otherwise

             	(G - B)*60/S,  if V=R
     	H <-    180+(B - R)*60/S,  if V=G
             	240+(R - G)*60/S,  if V=B

     	if H<0 then H<-H+360
	 *
	 *	 @param r red-value
	 *	 @param g green-value
	 *	 @param b blue-value
	 *	 @param h corresponding hue
	 *	 @param s corresponding saturation
	 *	 @param v corresponding intensity value
	 */
	void rgbToHsv(int red, int green, int blue, double *hue, double *sat, double *val);


	/**
	 * Calculates R, G, B value for rgb-24 bit encoding
	 * @param rgb24Bit
	 * @param r
	 * @param g
	 * @param b
	 */
	void rgb24bitToRGB(int rgb_val,  uint8_t *r, uint8_t *g, uint8_t *b);



	/**
	 * Calculates rgb-24 bit encoding for R, G, B values
	 * @param rgb24Bit
	 * @param r
	 * @param g
	 * @param b
	 */
	void rgbToRGB24Bit(float *rgb24Bit, unsigned char r, unsigned char g,
			unsigned char b);

};

}

#endif /* COLORSPACECONVERTOR_H_ */
