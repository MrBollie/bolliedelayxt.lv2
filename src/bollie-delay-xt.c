/**
    Bollie Delay XT - (c) 2016 Thomas Ebeling https://ca9.eu

    This file is part of bolliedelayxt.lv2

    bolliedelay.lv2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    bolliedelay.lv2 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
* \file bollie-delay-xt.c
* \author Bollie
* \date 13 Jul 2017
* \brief An LV2 tempo delay plugin with filters and tapping.
*/

#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "bolliefilter.h"

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"

#define URI "https://ca9.eu/lv2/bolliedelayxt"

// Max buffer size == 192k * 10 seconds (max delay time) + a bit for modulation
#define MAX_BUF_SIZE 2304000


/**
* Make a bool type available. ;)
*/
typedef enum { false, true } bool;


/**
* Enumeration of LV2 ports
*/
typedef enum {
    CP_TEMPO_MODE       = 0,
    CP_TEMPO_HOST       = 1,    
    CP_TEMPO_USER       = 2,    
    CP_TEMPO_DIV_CH1    = 3,
    CP_TEMPO_DIV_CH2    = 4,
    CP_FB_CH1           = 5,
    CP_FB_CH2           = 6,
    CP_CF_CH1           = 7,
    CP_CF_CH2           = 8,
    CP_GAIN_IN_CH1      = 9,
    CP_GAIN_IN_CH2      = 10,
    CP_GAIN_DRY_CH1     = 11,
    CP_GAIN_DRY_CH2     = 12,
    CP_GAIN_WET_CH1     = 13,
    CP_GAIN_WET_CH2     = 14,
    CP_MOD_DEPTH_CH1    = 15,
    CP_MOD_DEPTH_CH2    = 16,    
    CP_MOD_RATE_CH1     = 17,
    CP_MOD_RATE_CH2     = 18,   
    CP_HPF_PRE_ON_CH1   = 19, 
    CP_HPF_PRE_ON_CH2   = 20, 
    CP_HPF_PRE_FREQ_CH1 = 21, 
    CP_HPF_PRE_FREQ_CH2 = 22, 
    CP_HPF_PRE_Q_CH1    = 23, 
    CP_HPF_PRE_Q_CH2    = 24, 
    CP_LPF_PRE_ON_CH1   = 25, 
    CP_LPF_PRE_ON_CH2   = 26, 
    CP_LPF_PRE_FREQ_CH1 = 27, 
    CP_LPF_PRE_FREQ_CH2 = 28, 
    CP_LPF_PRE_Q_CH1    = 29, 
    CP_LPF_PRE_Q_CH2    = 30,   
    CP_HPF_FB_ON_CH1    = 31, 
    CP_HPF_FB_ON_CH2    = 32, 
    CP_HPF_FB_FREQ_CH1  = 33, 
    CP_HPF_FB_FREQ_CH2  = 34, 
    CP_HPF_FB_Q_CH1     = 35, 
    CP_HPF_FB_Q_CH2     = 36, 
    CP_LPF_FB_ON_CH1    = 37, 
    CP_LPF_FB_ON_CH2    = 38, 
    CP_LPF_FB_FREQ_CH1  = 39, 
    CP_LPF_FB_FREQ_CH2  = 40, 
    CP_LPF_FB_Q_CH1     = 41, 
    CP_LPF_FB_Q_CH2     = 42,    
     
} PortIdx;


/**
* Parameter storage
*/
typedef struct {
    float target;   ///< target value
    float current;  ///< current value
} BollieParam;


/**
* Struct for THE BollieDelayXT instance, the host is going to use.
*/
typedef struct {
    double sample_rate;                 ///< Store the current sample reate here

    float buf_delay_ch1[MAX_BUF_SIZE];  ///<< delay buffer for channel 1
    float buf_delay_ch1[MAX_BUF_SIZE];  ///<< delay buffer for channel 2
    
    float cp_t_dly_ch1;                 ///<< delay time for channel 1
    float cp_t_dly_ch2;                 ///<< delay time for channel 2
    

} BollieDelayXT;


/**
* Instantiates the plugin
* Allocates memory for the BollieDelayXT object and returns a pointer as
* LV2Handle.
*/
static LV2_Handle instantiate(const LV2_Descriptor * descriptor, double rate,
    const char* bundle_path, const LV2_Feature* const* features) {
    
    BollieDelayXT *self = (BollieDelayXT*)calloc(1, sizeof(BollieDelayXT));

    // Memorize sample rate for calculation
    self->sample_rate = rate;

    return (LV2_Handle)self;
}


/**
* Used by the host to connect the ports of this plugin.
* \param instance current LV2_Handle (will be cast to BollieDelayXT*)
* \param port LV2 port index, maches the enum above.
* \param data Pointer to the actual port data.
*/
static void connect_port(LV2_Handle instance, uint32_t port, void *data) {
    BollieDelayXT *self = (BollieDelayXT*)instance;

    switch ((PortIdx)port) {
    }
}
    

/**
* This has to reset all the internal states of the plugin
* \param instance pointer to current plugin instance
*/
static void activate(LV2_Handle instance) {
    BollieDelayXT* self = (BollieDelayXT*)instance;

}


/**
* Calculates number of samples used for divided delay times.
* \param self pointer to current plugin instance.
* \param tempo Tempo in BPM
* \param div   Divider
* \return number of samples needed for the delay buffer
* \todo divider enum
*/
static int calc_delay_samples(BollieDelayXT* self, float tempo, int div) {
    // Calculate the samples needed 
    float d = 60 / tempo * self->rate;
    switch(div) {
        case 1:
        d = d * 2/3;
            break;
        case 2:
        d = d / 2;
            break;
        case 3:
        d = d / 4 * 3;
            break;
        case 4:
        d = d / 3;
            break;
        case 5:
            d = d / 4;
            break;
    }
    return floor(d);
}


/**
* Main process function of the plugin.
* \param instance  handle of the current plugin
* \param n_samples number of samples in this current input block.
*/
static void run(LV2_Handle instance, uint32_t n_samples) {
    BollieDelayXT* self = (BollieDelayXT*)instance;


    // Loop over the block of audio we got
    for (unsigned int i = 0 ; i < n_samples ; ++i) {

    }

}


/**
* Called, when the host deactivates the plugin.
*/
static void deactivate(LV2_Handle instance) {
}


/**
* Cleanup, freeing memory and stuff
*/
static void cleanup(LV2_Handle instance) {
    free(instance);
}


/**
* extension stuff for additional interfaces
*/
static const void* extension_data(const char* uri) {
    return NULL;
}


/**
* Descriptor linking our methods.
*/
static const LV2_Descriptor descriptor = {
    URI,
    instantiate,
    connect_port,
    activate,
    run,
    deactivate,
    cleanup,
    extension_data
};


/**
* Symbol export using the descriptor above
*/
LV2_SYMBOL_EXPORT const LV2_Descriptor* lv2_descriptor(uint32_t index) {
    switch (index) {
        case 0:  return &descriptor;
        default: return NULL;
    }
}
