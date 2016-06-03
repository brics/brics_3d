/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2016, KU Leuven
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

#include "CoordinateConversions.h"

#include <math.h>
#include <cstdio>
#include <cstdlib>

namespace brics_3d {
namespace rsg {

const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;

// WGS84 Parameters
const double WGS84_A = 6378137.0;               // major axis
const double WGS84_B = 6356752.31424518;        // minor axis
const double WGS84_F = 0.0033528107;            // ellipsoid flattening
const double WGS84_E = 0.0818191908;            // first eccentricity
const double WGS84_EP = 0.0820944379;           // second eccentricity

// UTM Parameters
const double UTM_K0 = 0.9996;                   // scale factor
const double UTM_FE = 500000.0;                 // false easting
const double UTM_FN_N = 0.0;                    // false northing on north hemisphere
const double UTM_FN_S = 10000000.0;             // false northing on south hemisphere
const double UTM_E2 = (WGS84_E*WGS84_E);        // e^2
const double UTM_E4 = (UTM_E2*UTM_E2);          // e^4
const double UTM_E6 = (UTM_E4*UTM_E2);          // e^6
const double UTM_EP2 = (UTM_E2/(1-UTM_E2));     // e'^2


} /* namespace rsg */
} /* namespace brics_3d */

char brics_3d::rsg::CoordinateConversions::getUTMLetterDesignator(double latitude) {
    char letterDesignator;

    if     ((84 >= latitude) && (latitude >= 72))  letterDesignator = 'X';
    else if ((72 > latitude) && (latitude >= 64))  letterDesignator = 'W';
    else if ((64 > latitude) && (latitude >= 56))  letterDesignator = 'V';
    else if ((56 > latitude) && (latitude >= 48))  letterDesignator = 'U';
    else if ((48 > latitude) && (latitude >= 40))  letterDesignator = 'T';
    else if ((40 > latitude) && (latitude >= 32))  letterDesignator = 'S';
    else if ((32 > latitude) && (latitude >= 24))  letterDesignator = 'R';
    else if ((24 > latitude) && (latitude >= 16))  letterDesignator = 'Q';
    else if ((16 > latitude) && (latitude >= 8))   letterDesignator = 'P';
    else if (( 8 > latitude) && (latitude >= 0))   letterDesignator = 'N';
    else if (( 0 > latitude) && (latitude >= -8))  letterDesignator = 'M';
    else if ((-8 > latitude) && (latitude >= -16)) letterDesignator = 'L';
    else if((-16 > latitude) && (latitude >= -24)) letterDesignator = 'K';
    else if((-24 > latitude) && (latitude >= -32)) letterDesignator = 'J';
    else if((-32 > latitude) && (latitude >= -40)) letterDesignator = 'H';
    else if((-40 > latitude) && (latitude >= -48)) letterDesignator = 'G';
    else if((-48 > latitude) && (latitude >= -56)) letterDesignator = 'F';
    else if((-56 > latitude) && (latitude >= -64)) letterDesignator = 'E';
    else if((-64 > latitude) && (latitude >= -72)) letterDesignator = 'D';
    else if((-72 > latitude) && (latitude >= -80)) letterDesignator = 'C';
    // 'Z' is an error flag, the Latitude is outside the UTM limits
    else letterDesignator = 'Z';

    return letterDesignator;
}

void brics_3d::rsg::CoordinateConversions::convertLatLontoUTM(const double latitude,
		const double longitude, double& UTMNorthing, double& UTMEasting, std::string& UTMZone) {

    char UTMZoneAsChar[] = {0, 0, 0, 0};

	double a = WGS84_A;
	double eccSquared = UTM_E2;
	double k0 = UTM_K0;

	double LongOrigin;
	double eccPrimeSquared;
	double N, T, C, A, M;

	//Make sure the longitude is between -180.00 .. 179.9
	double LongTemp = (longitude+180)-int((longitude+180)/360)*360-180;

	double LatRad = latitude*RADIANS_PER_DEGREE;
	double LongRad = LongTemp*RADIANS_PER_DEGREE;
	double LongOriginRad;
	int    ZoneNumber;

	ZoneNumber = int((LongTemp + 180)/6) + 1;

	if( latitude >= 56.0 && latitude < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
		ZoneNumber = 32;

	// Special zones for Svalbard
	if( latitude >= 72.0 && latitude < 84.0 )
	{
		if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
		else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
		else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
		else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
	}
	// +3 puts origin in middle of zone
	LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
	LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

	//compute the UTM Zone from the latitude and longitude
	snprintf(UTMZoneAsChar, 4, "%d%c", ZoneNumber, getUTMLetterDesignator(latitude));

	eccPrimeSquared = (eccSquared)/(1-eccSquared);

	N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
	T = tan(LatRad)*tan(LatRad);
	C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
	A = cos(LatRad)*(LongRad-LongOriginRad);

	M = a*((1       - eccSquared/4          - 3*eccSquared*eccSquared/64    - 5*eccSquared*eccSquared*eccSquared/256)*LatRad
			- (3*eccSquared/8       + 3*eccSquared*eccSquared/32    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
			+ (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
			- (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

	UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
			+ (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
			+ 500000.0);

	UTMNorthing = (double)(k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
			+ (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
	if(latitude < 0)
		UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere

    UTMZone = UTMZoneAsChar;
}

void brics_3d::rsg::CoordinateConversions::convertUTMtoLatLon(const double UTMNorthing,
		const double UTMEasting, std::string UTMZone, double& latitude,
		double& longitude) {

	double k0 = UTM_K0;
	double a = WGS84_A;
	double eccSquared = UTM_E2;
	double eccPrimeSquared;
	double e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));
	double N1, T1, C1, R1, D, M;
	double LongOrigin;
	double mu, phi1, phi1Rad;
	double x, y;
	int ZoneNumber;
	char* ZoneLetter;
	int NorthernHemisphere; //1 for northern hemispher, 0 for southern

	x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
	y = UTMNorthing;

	ZoneNumber = strtoul(UTMZone.c_str(), &ZoneLetter, 10);
	if((*ZoneLetter - 'N') >= 0)
		NorthernHemisphere = 1;//point is in northern hemisphere
	else
	{
		NorthernHemisphere = 0;//point is in southern hemisphere
		y -= 10000000.0;//remove 10,000,000 meter offset used for southern hemisphere
	}

	LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;  //+3 puts origin in middle of zone

	eccPrimeSquared = (eccSquared)/(1-eccSquared);

	M = y / k0;
	mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64-5*eccSquared*eccSquared*eccSquared/256));

	phi1Rad = mu    + (3*e1/2-27*e1*e1*e1/32)*sin(2*mu)
            		  + (21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*mu)
            		  +(151*e1*e1*e1/96)*sin(6*mu);
	phi1 = phi1Rad*DEGREES_PER_RADIAN;

	N1 = a/sqrt(1-eccSquared*sin(phi1Rad)*sin(phi1Rad));
	T1 = tan(phi1Rad)*tan(phi1Rad);
	C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
	R1 = a*(1-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
	D = x/(N1*k0);

	latitude = phi1Rad - (N1*tan(phi1Rad)/R1)*(D*D/2-(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
			+(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared-3*C1*C1)*D*D*D*D*D*D/720);
	latitude = latitude * DEGREES_PER_RADIAN;

	longitude = (D-(1+2*T1+C1)*D*D*D/6+(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)
			*D*D*D*D*D/120)/cos(phi1Rad);
      longitude = LongOrigin + longitude * DEGREES_PER_RADIAN;

}
/* EOF */
