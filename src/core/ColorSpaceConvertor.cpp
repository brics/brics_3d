/**
 * ColorSpaceConvertor.cpp
 *
 *  Created on: Oct 5, 2011
 *  Author: PinakiSunilBanerjee
 */

#include "ColorSpaceConvertor.h"
#include <stdio.h>
#include <stdint.h>

namespace BRICS_3D {


ColorSpaceConvertor::ColorSpaceConvertor() {
	 this-> epsilon = 0.0001;
}


ColorSpaceConvertor::~ColorSpaceConvertor() {}


void ColorSpaceConvertor::rgbToHsv(int red, int green, int blue, double *hue, double *sat, double *val){

	float max, min;
	float h=0,s=0,v=0;
	float r,g,b;

	r = ((float)red)/255.0;
	g = ((float)green)/255.0;
	b = ((float)blue)/255.0;

	max = maxVal(r, g, b);
	min = minVal(r, g, b);

	v = max;

	if(fabs(v)>= epsilon){
		s= (v-min)/(v);
	} else {
		s=0;

	}
	if(fabs(v-r) <= epsilon){
		h= (g-b)*60.0/(s);
	} else if(fabs((v)-g) <= epsilon) {
		h= 180.0+(b-r)*60.0/(s);
	} else if(fabs((v)-b) <= epsilon) {
		h=240+(r - g)*60.0/(s);
	}

	if((h)<0.0)	(h) = (h)+360.0;

	h /= 2.0;
	s *= 255.0;
	v *= 255.0;

	*hue = h;
	*sat = s;
	*val=v;

//	printf("RGB Values: [%d %d %d %f %f %f %f]\n", red, green, blue, h, s, min, max);

}

void ColorSpaceConvertor::rgb24bitToRGB(int rgb_val,  uint8_t *r, uint8_t *g, uint8_t *b){


		*r = ((rgb_val >> 16) & 0xff);
		*g = ((rgb_val >> 8) & 0xff);
		*b = (rgb_val & 0xff);
}


void ColorSpaceConvertor::rgbToRGB24Bit(float *rgb24Bit, unsigned char r, unsigned char g,
		unsigned char b){

		  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		  *rgb24Bit = *reinterpret_cast<float*>(&rgb);
}

}
