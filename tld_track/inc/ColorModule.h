// ================================
// ColorModule.h
// ================================

#ifndef AMT_COLOR_MODULE_H
#define AMT_COLOR_MODULE_H

#include <cmath>
#include <cstring>
#include <vector>
#include "matrix.h"

#define MAX3(a,b,c) a > b ? (a > c ? a : c) : (b > c ? b : c);
#define MIN3(a,b,c) a < b ? (a < c ? a : c) : (b < c ? b : c);

#define NUM_BINS 7
#define GRAY_THRESHOLD 0.125

/// ultra discrete color histograms intended to be used as (weak) classifier asset
class AmtColorModule {
public:
	/// get instance of histogram generating singleton
	static AmtColorModule * getInstance();
	/// creates histogram from whole image
	float * getColorDistribution(const unsigned char * const rgb, const int & size) const;
	/// creates histogram from whole image
	float * getColorDistribution(const unsigned char * const rgb, const int & width, const int & height) const;
	/// creates histogram from image section
	float * getColorDistribution(const unsigned char * const rgb, const int & width, const int & height, const ObjectBox & box) const;
	/// creates a debug image with colors which are maped to same histogram value
	unsigned char * debugImage(const int & bin, int & sideLength) const;
	/// compares two histograms by performing a normalized chi-squared test on their average
	static float compareColorDistribution(const float * const hist1, const float * const hist2);

private:
	AmtColorModule();
	~AmtColorModule();

	static void toHS(const float & r, const float & g, const float & b, float & h, float & s);
	static float chiSquareSym(const float * const distr1, const float * const distr2, const int & n);
	static float chiSquare(const float * const correctHistogram, const float * const toCheck, const int & n);

	static AmtColorModule * ivInstance;
	unsigned char * ivLookupRGB;
};


#endif