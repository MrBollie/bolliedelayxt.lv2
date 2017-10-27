/**
    Bollie Filter - (c) 2016 Thomas Ebeling https://ca9.eu

    This file is part of Bollie Filter.

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
* \file bolliefilter.c
* \author Bollie (https://ca9.eu)
* \date 05 Nov 2016
* \brief A generic module for high/low cut filtering.
*/

#include "bolliefilter.h"
#include <math.h>

/**
* Initializes a BollieFilter object.
* \param bf Pointer to a BollieFilter object.
*/
void bf_init(BollieFilter* bf) {
    for (unsigned int i = 0 ; i < 3 ; ++i) {
        bf->in_buf[i] = 0;
        bf->processed_buf[i] = 0;
    }
    bf->fill_count = 0;
    bf->freq = 0;
    bf->Q = 0;
}


/**
* Resets a BollieFilter object.
*/
void bf_reset(BollieFilter* bf) {
    bf_init(bf);
}


/**
* Processes a frame using a low cut filter.
* \param in     Input sample
* \param freq   Filter cut off frequency
* \param Q      Filter quality
* \param rate   Current sampling rate
* \param bf     Pointer to the BollieFilter object
* \return       Output sample
* \todo         Validating parameters
*/
float bf_lcf(const float in, const float freq, const float Q, 
    double rate, BollieFilter* bf) {

    // Precalculate if needed.
    if (freq != bf->freq || Q != bf->Q || rate != bf->rate) {
        if (Q <= 0) {
            bf->Q = 0.001f;
        }
        else if (Q > 1.4f) {
            bf->Q = 1.4f;
        }
        else {
            bf->Q = Q;
        }
        bf->freq = freq;
        bf->rate = rate;
        float w0 = 2.f * PI * bf->freq / bf->rate;
        float alpha = sin(w0) / (2.f*bf->Q);
        bf->a0 = 1.f+alpha;
        bf->a1 = -2.f * cos(w0);
        bf->a2 = 1.f-alpha;
        bf->b0 = (1.f + cos(w0)) / 2.f;
        bf->b1 = -(1.f + cos(w0));
        bf->b2 = (1.f + cos(w0)) / 2.f; 
    }

    // Filter roll
    bf->in_buf[2] = bf->in_buf[1];
    bf->in_buf[1] = bf->in_buf[0];
    bf->in_buf[0] = in;

    bf->processed_buf[2] = bf->processed_buf[1];
    bf->processed_buf[1] = bf->processed_buf[0];

    // See if we need to fill the buffers first
    if (bf->fill_count < 3) {
        bf->processed_buf[0] = in;
        bf->fill_count++;
        return 0;
    }

    return bf->processed_buf[0] =             
            (bf->b0 / bf->a0 * bf->in_buf[0]) +
            (bf->b1 / bf->a0 * bf->in_buf[1]) +
            (bf->b2 / bf->a0 * bf->in_buf[2]) -
            (bf->a1 / bf->a0 * bf->processed_buf[1]) -
            (bf->a2 / bf->a0 * bf->processed_buf[2]);
}


/**
* Processes a frame using a high cut filter.
* \param in     Input sample
* \param freq   Filter cut off frequency
* \param Q      Filter quality
* \param rate   Current sampling rate
* \param bf     Pointer to the BollieFilter object
* \return       Output sample
*/
float bf_hcf(const float in, const float freq, const float Q, 
    double rate, BollieFilter* bf) {

    // Precalculate if needed.
    if (freq != bf->freq || Q != bf->Q || rate != bf->rate) {
        if (Q <= 0) {
            bf->Q = 0.001f;
        }
        else if (Q > 1.4f) {
            bf->Q = 1.4f;
        }
        else {
            bf->Q = Q;
        }
        bf->freq = freq;
        bf->rate = rate;
        float w0 = 2.f * PI * bf->freq / bf->rate;
        float alpha = sin(w0) / (2.f*bf->Q);
        bf->a0 = 1.f+alpha;
        bf->a1 = -2.f * cos(w0);
        bf->a2 = 1.f-alpha;
        bf->b0 = (1.f - cos(w0)) / 2.f;
        bf->b1 = 1.f - cos(w0);
        bf->b2 = (1.f - cos(w0)) / 2.f; 
    }

    // Filter roll
    bf->in_buf[2] = bf->in_buf[1];
    bf->in_buf[1] = bf->in_buf[0];
    bf->in_buf[0] = in;

    bf->processed_buf[2] = bf->processed_buf[1];
    bf->processed_buf[1] = bf->processed_buf[0];

    // See if we need to fill the buffers first
    if (bf->fill_count < 3) {
        bf->processed_buf[0] = in;
        bf->fill_count++;
        return 0;
    }

    return bf->processed_buf[0] =             
            (bf->b0 / bf->a0 * bf->in_buf[0]) +
            (bf->b1 / bf->a0 * bf->in_buf[1]) +
            (bf->b2 / bf->a0 * bf->in_buf[2]) -
            (bf->a1 / bf->a0 * bf->processed_buf[1]) -
            (bf->a2 / bf->a0 * bf->processed_buf[2]);
}

