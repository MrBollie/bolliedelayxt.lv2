/**
    Bollie Filter - (c) 2016 Thomas Ebeling https://ca9.eu

    This file is part of Bollie Filter

    This is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This code is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
* \file bolliefilter.h
* \author Bollie (https://ca9.eu)
* \date 05 Nov 2016
* \brief A generic module for high/low cut filtering.
*/

#ifndef __BOLLIEFILTER_H__
#define __BOLLIEFILTER_H__

#define PI 3.141592

/**
* Filter struct
*/
typedef struct bfilter {
    double  rate;               ///< Current sampling rate
    float   freq;               ///< cut off frequency
    float   Q;                  ///< filter quality
    float   a0;
    float   a1;
    float   a2;
    float   b0;
    float   b1;
    float   b2;
    float   in_buf[3];          ///< buffer for incoming samples
    float   processed_buf[3];   ///< buffer for samples processed by this filter
    unsigned int fill_count;    ///< fill count for the buffers
} BollieFilter;

void bf_init(BollieFilter*);
void bf_reset(BollieFilter*); 
float bf_lcf(const float in, const float freq, const float Q, 
    double rate, BollieFilter* bf); 

float bf_hcf(const float in, const float freq, const float Q, 
    double rate, BollieFilter* bf); 
    

#endif
