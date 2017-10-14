/**
    Bollie Delay XT - (c) 2017 Thomas Ebeling https://ca9.eu

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
    CP_TRAILS,
    CP_TEMPO_MODE,
    CP_TEMPO_HOST,    
    CP_TEMPO_USER,    
    CP_TEMPO_DIV_CH1,
    CP_TEMPO_DIV_CH2,
    CP_FB,
    CP_CF,
    CP_GAIN_IN,
    CP_GAIN_DRY,
    CP_GAIN_WET,
    CP_MOD_DEPTH,
    CP_MOD_RATE,
    CP_HPF_PRE_ON,
    CP_HPF_PRE_FREQ,
    CP_HPF_PRE_Q,
    CP_LPF_PRE_ON,
    CP_LPF_PRE_FREQ,
    CP_LPF_PRE_Q,
    CP_HPF_FB_ON,
    CP_HPF_FB_FREQ,
    CP_HPF_FB_Q,
    CP_LPF_FB_ON,
    CP_LPF_FB_FREQ,
    CP_LPF_FB_Q,
} PortIdx;


/**
* State enum
*/
typedef enum {
    FADE_IN, FADE_OUT, FADE_OUT_DONE, FILL_BUF, CYCLE
} BollieState;



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
    float buf_delay_ch2[MAX_BUF_SIZE];  ///<< delay buffer for channel 2
    
    float *cp_trails;
    float *cp_tempo_mode;
    float *cp_tempo_host;
    float *cp_tempo_user;
    float *cp_tempo_div_ch1;
    float *cp_tempo_div_ch2;
    float *cp_fb;
    float *cp_cf;
    float *cp_gain_in;
    float *cp_gain_dry;
    float *cp_gain_wet;
    float *cp_mod_depth;
    float *cp_mod_rate;
    float *cp_hpf_pre_on;
    float *cp_hpf_pre_freq;
    float *cp_hpf_pre_q;
    float *cp_lpf_pre_on;
    float *cp_lpf_pre_freq;
    float *cp_lpf_pre_q;
    float *cp_hpf_fb_on;
    float *cp_hpf_fb_freq;
    float *cp_hpf_fb_q;
    float *cp_lpf_fb_on;
    float *cp_lpf_fb_freq;
    float *cp_lpf_fb_q;

    BollieState state;

    float cur_tempo;
    float cur_tempo_div_ch1;
    float cur_tempo_div_ch2;

    float cur_d_t_ch1;
    float cur_d_t_ch2;

    int cur_d_s_ch1;
    int cur_d_s_ch2;

    int pos_w_ch1;
    int pos_w_ch2;
    float pos_r_ch1;
    float pos_r_ch2;
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
        case CP_TRAILS:
            cp_trails = data;
            break;
        case CP_TEMPO_MODE:
            cp_tempo_mode = data;
            break;
        case CP_TEMPO_HOST:
            cp_tempo_host = data;
            break;
        case CP_TEMPO_USER:
            cp_tempo_user = data;
            break;
        case CP_TEMPO_DIV_CH1:
            cp_tempo_div_ch1 = data;
            break;
        case CP_TEMPO_DIV_CH2:
            cp_tempo_div_ch2 = data;
            break;
        case CP_FB:
            cp_fb = data;
            break;
        case CP_CF:
            cp_cf = data;
            break;
        case CP_GAIN_IN:
            cp_gain_in = data;
            break;
        case CP_GAIN_DRY:
            cp_gain_dry = data;
            break;
        case CP_GAIN_WET:
            cp_gain_wet = data;
            break;
        case CP_MOD_DEPTH:
            cp_mod_depth = data;
            break;
        case CP_MOD_RATE:
            cp_mod_rate = data;
            break;
        case CP_HPF_PRE_ON:
            cp_hpf_pre_on = data;
            break;
        case CP_HPF_PRE_FREQ:
            cp_hpf_pre_freq = data;
            break;
        case CP_HPF_PRE_Q:
            cp_hpf_pre_q = data;
            break;
        case CP_LPF_PRE_ON:
            cp_lpf_pre_on = data;
            break;
        case CP_LPF_PRE_FREQ:
            cp_lpf_pre_freq = data;
            break;
        case CP_LPF_PRE_Q:
            cp_lpf_pre_q = data;
            break;
        case CP_HPF_FB_ON:
            cp_hpf_fb_on = data;
            break;
        case CP_HPF_FB_FREQ:
            cp_hpf_fb_freq = data;
            break;
        case CP_HPF_FB_Q:
            cp_hpf_fb_q = data;
            break;
        case CP_LPF_FB_ON:
            cp_lpf_fb_on = data;
            break;
        case CP_LPF_FB_FREQ:
            cp_lpf_fb_freq = data;
            break;
        case CP_LPF_FB_Q:
            cp_lpf_fb_q = data;
            break;
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
static float calc_delay_samples(BollieDelayXT* self, float tempo, int div) {
    // Calculate the samples needed 
    float d = 60 / tempo * self->sample_rate;
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
    return d;
}


/**
* Main process function of the plugin.
* \param instance  handle of the current plugin
* \param n_samples number of samples in this current input block.
*/
static void run(LV2_Handle instance, uint32_t n_samples) {
    BollieDelayXT* self = (BollieDelayXT*)instance;

    // Localize
    int cur_d_s_ch1 = self->cur_d_s_ch1;
    int cur_d_s_ch2 = self->cur_d_s_ch2;
    float cur_d_t_ch1 = self->cur_d_t_ch1;
    float cur_d_t_ch2 = self->cur_d_t_ch2;
    float pos_r_ch1 = self->pos_r_ch_1;
    float pos_r_ch2 = self->pos_r_ch_2;
    int pos_w_ch1 = self->pos_w_ch1;
    int pos_w_ch2 = self->pos_w_ch2;
    BollieState state = self->state;
    float tgt_gain_in = self->tgt_gain_in;
    float tgt_gain_dry = self->tgt_gain_dry;
    float tgt_gain_wet = self->tgt_gain_wet;
    float tgt_cf = self->tgt_cf;
    float tgt_fb = self->tgt_fb;

    // Tempo handling
    // Tempo mode has changed
    float cur_tempo = (*self->cp_tempo_mode == 1 ? *self->cp_tempo_user :
        *self->cp_tempo_host);

    // Tempo has changed
    if (cur_tempo != self->cur_tempo 
        || self->cur_tempo_div_ch1 != *self->cp_tempo_div_ch1
        || self->cur_tempo_div_ch2 != *self->cp_tempo_div_ch2
    ) {
        // Fade out
        if (state == FADE_OUT_DONE) {
            // Memorize current settings
            self->cur_tempo = cur_tempo;
            self->cur_tempo_div_ch1 = *self->cp_tempo_div_ch1;
            self->cur_tempo_div_ch2 = *self->cp_tempo_div_ch2;

            // Calculate sample offset and memorize it
            cur_d_t_ch1 = calc_delay_samples(self, cur_tempo, 
                self->cur_tempo_div_ch1);
            cur_d_t_ch2 = calc_delay_samples(self, cur_tempo, 
                self->cur_tempo_div_ch2);

            // Safety!
            if (cur_d_t_ch1 + 1.f > MAX_BUF_SIZE) 
                cur_d_t_ch1 = MAX_BUF_SIZE-1.f;

            if (cur_d_t_ch2 + 1.f > MAX_BUF_SIZE)
                cur_d_t_ch2 = MAX_BUF_SIZE-1.f;

            // The buffer is integer, so make sure, it is big enough.
            cur_d_s_ch1 = ceil(cur_d_t_ch1) + 1;
            cur_d_s_ch2 = ceil(cur_d_t_ch2) + 1;

            pos_r_ch1 = 0;
            pos_r_ch2 = 0;
            pos_w_ch1 = 0;
            pos_w_ch2 = 0;

            *self->cp_tempo_out = cur_tempo;
            state = FILL_BUF;
        }
        else if (state != FADE_OUT) {
            state = FADE_OUT;
        }
    }

    // Gain handling
    if (*self->cp_gain_in != self->cur_cp_gain_in) {
        if (*self->cp_gain_in > 12.f) {
            tgt_gain_in = 4.f;
        }
        else if (*self->cp_gain_in < 96.f) {
            tgt_gain_in = 0;
        }
        else {
            tgt_gain_in = powf(10, (*self->cp_gain_in/20));
        }
        self->cur_cp_gain_in = *self->cp_gain_in;
    } 
    
    if (*self->cp_gain_dry != self->cur_cp_gain_dry) {
        if (*self->cp_gain_dry > 12.f) {
            tgt_gain_dry = 4.f;
        }
        else if (*self->cp_gain_dry < 96.f) {
            tgt_gain_dry = 0;
        }
        else {
            tgt_gain_dry = powf(10, (*self->cp_gain_dry/20));
        }
        self->cur_cp_gain_dry = *self->cp_gain_dry;
    } 
    
    if (*self->cp_gain_wet != self->cur_cp_gain_wet) {
        if (*self->cp_gain_wet > 12.f) {
            tgt_gain_wet = 4.f;
        }
        else if (*self->cp_gain_wet < 96.f) {
            tgt_gain_wet = 0;
        }
        else {
            tgt_gain_wet = powf(10, (*self->cp_gain_wet/20));
        }
        self->cur_cp_gain_wet = *self->cp_gain_wet;
    } 
    
    // Feedback
    if (*self->cp_fb != self->cur_cp_fb) {
        if (*self->cp_fb > 12.f) {
            tgt_fb = 4.f;
        }
        else if (*self->cp_fb < 96.f) {
            tgt_fb = 0;
        }
        else {
            tgt_fb = powf(10, (*self->cp_fb/20));
        }
        self->cur_cp_fb = *self->cp_fb;
    } 
    
    // Crossfeed
    if (*self->cp_cf != self->cur_cp_cf) {
        if (*self->cp_cf > 12.f) {
            tgt_cf = 4.f;
        }
        else if (*self->cp_cf < 96.f) {
            tgt_cf = 0;
        }
        else {
            tgt_cf = powf(10, (*self->cp_cf/20));
        }
        self->cur_cp_cf = *self->cp_cf;
    } 
    


    // Loop over the block of audio we got
    for (unsigned int i = 0 ; i < n_samples ; ++i) {

        // Current samples
        float cur_s_ch1 = self->input_ch1[i];
        float cur_s_ch2 = self->input_ch1[2];

        // Store old samples here
        float old_s_ch1 = 0;
        float old_s_ch2 = 0;
    }

    self->cur_d_s_ch1 = cur_d_s_ch_1;
    self->cur_d_s_ch2 = cur_d_s_ch_2;
    self->cur_d_t_ch1 = cur_d_t_ch_1;
    self->cur_d_t_ch2 = cur_d_t_ch_2;
    self->pos_r_ch1 = pos_r_ch1;
    self->pos_r_ch2 = pos_r_ch2;
    self->pos_w_ch1 = pos_w_ch1;
    self->pos_w_ch2 = pos_w_ch2;
    self->state = state;
    self->tgt_gain_in = tgt_gain_in;
    self->tgt_gain_dry = tgt_gain_dry;
    self->tgt_gain_wet = tgt_gain_wet;
    self->tgt_cf = tgt_cf;
    self->tgt_fb = tgt_fb;
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
