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
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "bolliefilter.h"

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"

#define URI "https://ca9.eu/lv2/bolliedelayxt"

// Max buffer size == 192k * 10 seconds (max delay time) + a bit for modulation
#define TWO_PI (M_PI*2)
#define MAX_BUF_SIZE 769000
#define FADE_LENGTH_MS 50
#define MOD_OFFSET_MS 5.f
#define LIM_ATTACK 10.f
#define LIM_RELEASE 10.f

/**
* Make a bool type available. ;)
*/
typedef enum { false, true } bool;


/**
* Enumeration of LV2 ports
*/
typedef enum {
    IP_INPUT_CH1,
    IP_INPUT_CH2,
    OP_OUTPUT_CH1,
    OP_OUTPUT_CH2,
    CP_ENABLED,
    CP_TRAILS,
    CP_TEMPO_MODE,
    CP_PING_PONG,
    CP_TEMPO_HOST,    
    CP_TEMPO_USER,    
    CP_TEMPO_DIV_CH1,
    CP_TEMPO_DIV_CH2,
    CP_FB,
    CP_CF,
    CP_GAIN_DRY,
    CP_GAIN_WET,
    CP_MOD_ON,
    CP_MOD_PHASE,
    CP_MOD_DEPTH,
    CP_MOD_RATE,
    CP_HCF_PRE_ON,
    CP_HCF_PRE_FREQ,
    CP_HCF_PRE_Q,
    CP_LCF_PRE_ON,
    CP_LCF_PRE_FREQ,
    CP_LCF_PRE_Q,
    CP_HCF_FB_ON,
    CP_HCF_FB_FREQ,
    CP_HCF_FB_Q,
    CP_LCF_FB_ON,
    CP_LCF_FB_FREQ,
    CP_LCF_FB_Q,
    CP_TEMPO_OUT
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
    double sample_rate;               ///< Current sample rate

    float buffer_ch1[MAX_BUF_SIZE];  ///<< delay buffer for channel 1
    float buffer_ch2[MAX_BUF_SIZE];  ///<< delay buffer for channel 2
    
    const float *cp_enabled;
    const float *cp_trails;
    const float *cp_tempo_mode;
    const float *cp_ping_pong;
    const float *cp_tempo_host;
    const float *cp_tempo_user;
    const float *cp_tempo_div_ch1;
    const float *cp_tempo_div_ch2;
    const float *cp_fb;
    const float *cp_cf;
    const float *cp_gain_dry;
    const float *cp_gain_wet;
    const float *cp_mod_on;
    const float *cp_mod_phase;
    const float *cp_mod_depth;
    const float *cp_mod_rate;
    const float *cp_hcf_pre_on;
    const float *cp_hcf_pre_freq;
    const float *cp_hcf_pre_q;
    const float *cp_lcf_pre_on;
    const float *cp_lcf_pre_freq;
    const float *cp_lcf_pre_q;
    const float *cp_hcf_fb_on;
    const float *cp_hcf_fb_freq;
    const float *cp_hcf_fb_q;
    const float *cp_lcf_fb_on;
    const float *cp_lcf_fb_freq;
    const float *cp_lcf_fb_q;
    float *cp_tempo_out;

    BollieFilter fil_hcf_fb_ch1;
    BollieFilter fil_hcf_fb_ch2;
    BollieFilter fil_lcf_fb_ch1;
    BollieFilter fil_lcf_fb_ch2;
    BollieFilter fil_hcf_pre_ch1;
    BollieFilter fil_hcf_pre_ch2;
    BollieFilter fil_lcf_pre_ch1;
    BollieFilter fil_lcf_pre_ch2;

    const float *input_ch1;
    const float *input_ch2;

    float *output_ch1;
    float *output_ch2;

    BollieState state;

    float cur_cf;
    float cur_fb;

    float cur_cp_gain_dry;
    float cur_cp_gain_wet;

    float cur_cp_cf;
    float cur_cp_fb;

    float cur_gain_dry;
    float cur_gain_wet;
    float cur_gain_buf_in;

    float cur_mod_depth;
    float cur_mod_rate;
    float cur_mod_phase;

    float cur_tempo;
    float cur_tempo_div_ch1;
    float cur_tempo_div_ch2;

    float cur_d_t_ch1;
    float cur_d_t_ch2;

    float lfo_incr;
    float lfo_curphase;
    float lfo_circle;

    int32_t fade_length;
    int32_t fade_pos;

    int32_t pos_w;
    uint32_t mod_offset_samples;

    float tgt_d_t_ch1;
    float tgt_d_t_ch2;

    float tgt_cf;
    float tgt_fb;

    float tgt_gain_dry;
    float tgt_gain_wet;

    float lim_attack;
    float lim_release;
    float lim_envelope_ch1;
    float lim_envelope_ch2;
    
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

    // Prepare fade stuff
    self->fade_length = ceil(rate / 1000 * FADE_LENGTH_MS);
    self->fade_pos = 0;

    self->mod_offset_samples = ceil(MOD_OFFSET_MS / 1000 * rate);

    // LFO
    self->lfo_circle = TWO_PI/(float)rate;

    // limiter
    self->lim_attack  = powf(0.01f, 1.0f / (LIM_ATTACK * rate * 0.001f) ); 
    self->lim_release = powf(0.01f, 1.0f / (LIM_RELEASE * rate * 0.001f) );

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
        case IP_INPUT_CH1:
            self->input_ch1 = data;
            break;
        case IP_INPUT_CH2:
            self->input_ch2 = data;
            break;
        case OP_OUTPUT_CH1:
            self->output_ch1 = data;
            break;
        case OP_OUTPUT_CH2:
            self->output_ch2 = data;
            break;
        case CP_ENABLED:
            self->cp_enabled = data;
            break;
        case CP_TRAILS:
            self->cp_trails = data;
            break;
        case CP_PING_PONG:
            self->cp_ping_pong = data;
            break;
        case CP_TEMPO_MODE:
            self->cp_tempo_mode = data;
            break;
        case CP_TEMPO_HOST:
            self->cp_tempo_host = data;
            break;
        case CP_TEMPO_USER:
            self->cp_tempo_user = data;
            break;
        case CP_TEMPO_DIV_CH1:
            self->cp_tempo_div_ch1 = data;
            break;
        case CP_TEMPO_DIV_CH2:
            self->cp_tempo_div_ch2 = data;
            break;
        case CP_FB:
            self->cp_fb = data;
            break;
        case CP_CF:
            self->cp_cf = data;
            break;
        case CP_GAIN_DRY:
            self->cp_gain_dry = data;
            break;
        case CP_GAIN_WET:
            self->cp_gain_wet = data;
            break;
        case CP_MOD_ON:
            self->cp_mod_on = data;
            break;
        case CP_MOD_PHASE:
            self->cp_mod_phase = data;
            break;
        case CP_MOD_DEPTH:
            self->cp_mod_depth = data;
            break;
        case CP_MOD_RATE:
            self->cp_mod_rate = data;
            break;
        case CP_HCF_PRE_ON:
            self->cp_hcf_pre_on = data;
            break;
        case CP_HCF_PRE_FREQ:
            self->cp_hcf_pre_freq = data;
            break;
        case CP_HCF_PRE_Q:
            self->cp_hcf_pre_q = data;
            break;
        case CP_LCF_PRE_ON:
            self->cp_lcf_pre_on = data;
            break;
        case CP_LCF_PRE_FREQ:
            self->cp_lcf_pre_freq = data;
            break;
        case CP_LCF_PRE_Q:
            self->cp_lcf_pre_q = data;
            break;
        case CP_HCF_FB_ON:
            self->cp_hcf_fb_on = data;
            break;
        case CP_HCF_FB_FREQ:
            self->cp_hcf_fb_freq = data;
            break;
        case CP_HCF_FB_Q:
            self->cp_hcf_fb_q = data;
            break;
        case CP_LCF_FB_ON:
            self->cp_lcf_fb_on = data;
            break;
        case CP_LCF_FB_FREQ:
            self->cp_lcf_fb_freq = data;
            break;
        case CP_LCF_FB_Q:
            self->cp_lcf_fb_q = data;
            break;
        case CP_TEMPO_OUT:
            self->cp_tempo_out = data;
            break;
    }
}
    

/**
* This has to reset all the internal states of the plugin
* \param instance pointer to current plugin instance
*/
static void activate(LV2_Handle instance) {
    BollieDelayXT* self = (BollieDelayXT*)instance;
    self->state = FILL_BUF;
    self->pos_w = 0;
    self->fade_pos = 0;
    self->cur_cp_gain_dry = -97.f;
    self->cur_cp_gain_wet = -97.f;
    self->cur_cp_cf = 0;
    self->cur_cp_fb = 0;
    self->cur_gain_dry = 0;
    self->cur_gain_wet = 0;
    self->cur_mod_depth = 0;
    self->cur_mod_rate = 0;
    self->cur_mod_phase = 0;
    self->cur_cf = 0;
    self->cur_fb = 0;
    self->cur_tempo = 0;
    self->lfo_curphase = 0.0f;
    self->lfo_incr = 0;
    self->cur_gain_buf_in = 0;
    self->tgt_d_t_ch1 = 0.5f * self->sample_rate;
    self->cur_d_t_ch1 = 0;
    self->tgt_d_t_ch2 = 0.5f * self->sample_rate;
    self->cur_d_t_ch2 = 0;

    self->lim_envelope_ch1 = 0;
    self->lim_envelope_ch2 = 0;

    bf_init(&self->fil_hcf_fb_ch1);
    bf_init(&self->fil_hcf_fb_ch2);
    bf_init(&self->fil_lcf_fb_ch1);
    bf_init(&self->fil_lcf_fb_ch2);
    bf_init(&self->fil_hcf_pre_ch1);
    bf_init(&self->fil_hcf_pre_ch2);
    bf_init(&self->fil_lcf_pre_ch1);
    bf_init(&self->fil_lcf_pre_ch2);
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
        case 0: // 1/2
            d = d * 2;
            break;
        case 1: // 1/2T
            d = d * 4/3;
            break;
        case 2: // 1/2.
            d = d * 3;
            break;
        case 4: // 1/4T
            d = d * 2/3;
            break;
        case 5: // 1/4.
            d = d * 1.5f;
            break;
        case 6: // 1/8
            d = d / 2;
            break;
        case 7: // 1/8T
            d = d * 1/3;
            break;
        case 8: // 1/8.
            d = d / 2 * 3;
            break;
        case 9: // 1/16
            d = d / 4;
            break;
        case 10: // 1/16T
            d = d / 6;
            break;
        case 11: // 1/16.
            d = d / 4 * 3;
            break;
    }
    return d;
}

/**
* linear sample interpolation from buffer
* \param buf pointer to the buffer
* \param x sample coordinate. Can be also negative.
* \return interpolated sample
*/
static float interpolate(float *buf, double x) {
    if (x < 0) x += MAX_BUF_SIZE;
    if (x >= MAX_BUF_SIZE) x -= MAX_BUF_SIZE;
    int32_t x0 = (int32_t)x;
    float frac = x - (double)x0;
    int32_t x1 = x0+1;
    return buf[x0]  + frac * (buf[x1 >= MAX_BUF_SIZE ? 0 : x1] - buf[x0]);
}


/**
* Main process function of the plugin.
* \param instance  handle of the current plugin
* \param n_samples number of samples in this current input block.
*/
static void run(LV2_Handle instance, uint32_t n_samples) {
    BollieDelayXT* self = (BollieDelayXT*)instance;

    // Copy variables from heap to stack to speed up looping over the samples
    float cur_cf = self->cur_cf;
    float cur_fb = self->cur_fb;
    float cur_d_t_ch1 = self->cur_d_t_ch1;
    float cur_d_t_ch2 = self->cur_d_t_ch2;
    float cur_gain_buf_in = self->cur_gain_buf_in;
    float cur_gain_dry = self->cur_gain_dry;
    float cur_gain_wet = self->cur_gain_wet;
    float cur_mod_depth = self->cur_mod_depth;
    float cur_mod_phase = self->cur_mod_phase;
    float cur_mod_rate = self->cur_mod_rate;
    float cp_enabled = *self->cp_enabled;
    float cp_ping_pong = *self->cp_ping_pong;
    float cp_hcf_fb_on = *self->cp_hcf_fb_on;
    float cp_hcf_fb_freq = *self->cp_hcf_fb_freq;
    float cp_hcf_fb_q = *self->cp_hcf_fb_q;
    float cp_lcf_fb_on = *self->cp_lcf_fb_on;
    float cp_lcf_fb_freq = *self->cp_lcf_fb_freq;
    float cp_lcf_fb_q = *self->cp_lcf_fb_q;
    float cp_hcf_pre_on = *self->cp_hcf_pre_on;
    float cp_hcf_pre_freq = *self->cp_hcf_pre_freq;
    float cp_hcf_pre_q = *self->cp_hcf_pre_q;
    float cp_lcf_pre_on = *self->cp_lcf_pre_on;
    float cp_lcf_pre_freq = *self->cp_lcf_pre_freq;
    float cp_lcf_pre_q = *self->cp_lcf_pre_q;
    float cp_mod_on = *self->cp_mod_on;
    float cp_mod_phase = *self->cp_mod_phase;
    float cp_mod_depth = *self->cp_mod_depth;
    float cp_mod_rate = *self->cp_mod_rate;
    float cp_trails = *self->cp_trails;
    int32_t fade_pos = self->fade_pos;
    int32_t fade_length = self->fade_length;
    float lfo_curphase = self->lfo_curphase;
    float lfo_incr = self->lfo_incr;
    int32_t pos_w = self->pos_w;
    double rate = self->sample_rate;
    BollieState state = self->state;
    float tgt_d_t_ch1 = self->tgt_d_t_ch1;
    float tgt_d_t_ch2 = self->tgt_d_t_ch2;
    float tgt_gain_dry = self->tgt_gain_dry;
    float tgt_gain_wet = self->tgt_gain_wet;
    float tgt_cf = self->tgt_cf;
    float tgt_fb = self->tgt_fb;

    float lim_attack = self->lim_attack;
    float lim_release = self->lim_release;
    float lim_envelope_ch1 = self->lim_envelope_ch1;
    float lim_envelope_ch2 = self->lim_envelope_ch2;

    // Tempo handling
    // Tempo mode has changed
    float cur_tempo = (*self->cp_tempo_mode == 1 ? *self->cp_tempo_user :
        *self->cp_tempo_host);

    // Tempo has changed
    if (cur_tempo != self->cur_tempo 
        || self->cur_tempo_div_ch1 != *self->cp_tempo_div_ch1
        || self->cur_tempo_div_ch2 != *self->cp_tempo_div_ch2
    ) {
        tgt_d_t_ch1 = calc_delay_samples(self, cur_tempo, 
            *self->cp_tempo_div_ch1);

        tgt_d_t_ch2 = calc_delay_samples(self, cur_tempo, 
            *self->cp_tempo_div_ch2);

        // Safety!
        if (tgt_d_t_ch1 + self->mod_offset_samples >= MAX_BUF_SIZE) 
            tgt_d_t_ch1 = MAX_BUF_SIZE-self->mod_offset_samples-1;

        if (tgt_d_t_ch2 + self->mod_offset_samples >= MAX_BUF_SIZE)
            tgt_d_t_ch2 = MAX_BUF_SIZE-self->mod_offset_samples-1;

        self->cur_tempo = cur_tempo;
        self->cur_tempo_div_ch1 = *self->cp_tempo_div_ch1;
        self->cur_tempo_div_ch2 = *self->cp_tempo_div_ch2;
        *self->cp_tempo_out = cur_tempo;
    }

    // Gain handling
    
    if (*self->cp_gain_dry != self->cur_cp_gain_dry) {
        if (*self->cp_gain_dry > 12.f) {
            tgt_gain_dry = 4.f;
        }
        else if (*self->cp_gain_dry < -96.f) {
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
        else if (*self->cp_gain_wet < -96.f) {
            tgt_gain_wet = 0;
        }
        else {
            tgt_gain_wet = powf(10, (*self->cp_gain_wet/20));
        }
        self->cur_cp_gain_wet = *self->cp_gain_wet;
    } 
    
    // Feedback
    if (!cp_ping_pong) {
        if (*self->cp_fb != self->cur_cp_fb) {
            if (*self->cp_fb > 99.f ) {
                tgt_fb = 1.f;
            }
            else if (*self->cp_fb < 0) {
                tgt_fb = 0;
            }
            else {
                tgt_fb = *self->cp_fb / 100;
            }
            self->cur_cp_fb = *self->cp_fb;
        }
    } 
    else {
        // ping pong mode. We don't want any FB here
        tgt_fb = 0;
    }
    
    // Crossfeed
    if (*self->cp_cf != self->cur_cp_cf) {
        if (*self->cp_cf > 99.f) {
            tgt_cf = 1.f;
        }
        else if (*self->cp_cf < 0) {
            tgt_cf = 0;
        }
        else {
            tgt_cf = *self->cp_cf / 100;
        }
        self->cur_cp_cf = *self->cp_cf;
    } 

    // Modulation
    if (cp_mod_depth < 0.1f || cp_mod_depth > MOD_OFFSET_MS)
        cp_mod_depth = 2.f;

    if (cp_mod_rate < 0.1f || cp_mod_rate > 2.f)
        cp_mod_rate = 0.1f;

    // User disabled the plugin, fade out
    if (!cp_enabled && state != FADE_OUT_DONE && !cp_trails)
        state = FADE_OUT;

    // Loop over the block of audio we got
    for (uint32_t i = 0 ; i < n_samples ; ++i) {

        /* Shortcut here, if the user has disabled the plugin
           This is not relevant for trail mode*/
        if (state == FADE_OUT_DONE) {
            if (!cp_enabled) {
                cur_gain_dry = 0.01f + cur_gain_dry * 0.99f;
                self->output_ch1[i] = self->input_ch1[i] * cur_gain_dry;
                self->output_ch2[i] = self->input_ch2[i] * cur_gain_dry;
                continue;
            }
            else {
                pos_w = 0;
                state = FILL_BUF;
            }
        }

        // Parameter smoothing
        cur_gain_buf_in = (!cp_enabled && cp_trails ? 0 : 0.01f) 
            + cur_gain_buf_in * 0.99f;
        cur_gain_dry = tgt_gain_dry * 0.01f + cur_gain_dry * 0.99f;
        cur_gain_wet = tgt_gain_wet * 0.01f + cur_gain_wet * 0.99f;
        cur_cf = tgt_cf * 0.01f + cur_cf * 0.99f;
        cur_fb = tgt_fb * 0.01f + cur_fb * 0.99f;
        cur_mod_depth = (cp_mod_on ? cp_mod_depth : 0) * 0.01f 
            + cur_mod_depth * 0.99f;

        cur_d_t_ch1 = tgt_d_t_ch1 * 0.001f + cur_d_t_ch1 * 0.999f;
        cur_d_t_ch2 = tgt_d_t_ch2 * 0.001f + cur_d_t_ch2 * 0.999f;

        // Keep the LFO running
        float lfo_offset_ch1 = 0;
        float lfo_offset_ch2 = 0;
        if (cur_mod_depth > 0) {
            float lfo_coeff = sinf(lfo_curphase);
            if (cp_mod_rate != cur_mod_rate) {
                cur_mod_rate = cp_mod_rate;
                lfo_incr = self->lfo_circle * cur_mod_rate;
            }
            lfo_curphase += lfo_incr;

            // A chance to do desired phase switching
            if (lfo_curphase >= TWO_PI) {
                cur_mod_phase = cp_mod_phase;
                lfo_curphase -= TWO_PI;
            }
            else if (lfo_curphase < 0.0f) {
                cur_mod_phase = cp_mod_phase;
                lfo_curphase += TWO_PI;
            }

            // Calculate offset for ch1
            lfo_offset_ch1 = (cur_mod_depth / 1000 * rate) * lfo_coeff;

            // In case the user desires a phase switch, then turn the ch2 by
            // 180 degrees
            lfo_offset_ch2 = 
                cur_mod_phase ? lfo_offset_ch1 * -1 : lfo_offset_ch1;
        }
        else {
            lfo_curphase = 0.0f;
        }

        // Store old samples here
        float old_s_ch1 = 0;
        float old_s_ch2 = 0;

        // Current samples
        float cur_s_ch1 = self->input_ch1[i];
        float cur_s_ch2 = self->input_ch2[i];

        // Gain coefficient used while fading
        float fade_coeff = 0;

        if (state == FADE_OUT) {
            if (fade_pos > 0) { 
               fade_coeff = --fade_pos * (1/(float)fade_length); 
            }
            else {
                state = FADE_OUT_DONE;
            }
        }
        else if (state == FILL_BUF) {
            // Change to state fade in, when the buffer is full enough
            if ((double)pos_w > cur_d_t_ch1 + self->mod_offset_samples
                && (double)pos_w > cur_d_t_ch2 + self->mod_offset_samples) {
                state = FADE_IN;
            }
        }
        else if (state == FADE_IN) {
            if (fade_pos < fade_length) {
                // Keep fading
                fade_coeff = fade_pos++ * (1/(float)fade_length);
            }
            else {
                // fade is done, let's cycle. ;)
                state = CYCLE;
                fade_coeff = 1;
            }
        }
        else {
            // default
            fade_coeff = 1;
        }

        // In this states, we'll retrieve old samples, interpolate if needed
        if (state == FADE_IN || state == FADE_OUT || state == CYCLE) {
            // Channel 1
            double x = (double)pos_w - cur_d_t_ch1 + lfo_offset_ch1; 
            old_s_ch1 = interpolate(self->buffer_ch1, x) * fade_coeff;

            // Channel 2
            x = (double)pos_w - cur_d_t_ch2 + lfo_offset_ch2; 
            old_s_ch2 = interpolate(self->buffer_ch2, x) * fade_coeff;

            /* Limiting happening after retrieval from buffer to safe from
            modulation going bonkers */
            float v = fabs(old_s_ch1);
            lim_envelope_ch1 = (v > lim_envelope_ch1 ? lim_attack : lim_release)
                * (lim_envelope_ch1 - v) + v;
            if (lim_envelope_ch1 > 1.f) old_s_ch1 /= lim_envelope_ch1;

            v = fabs(old_s_ch2);
            lim_envelope_ch2 = (v > lim_envelope_ch2 ? lim_attack : lim_release)
                * (lim_envelope_ch2 - v) + v;
            if (lim_envelope_ch1 > 1.f) old_s_ch2 /= lim_envelope_ch2;

            // High cut filter on feedback
            if (cp_hcf_fb_on) {
                old_s_ch1 = bf_hcf(old_s_ch1, cp_hcf_fb_freq, cp_hcf_fb_q, rate,
                    &self->fil_hcf_fb_ch1);
                old_s_ch2 = bf_hcf(old_s_ch2, cp_hcf_fb_freq, cp_hcf_fb_q, rate,
                    &self->fil_hcf_fb_ch2);
            }

            // Low cut filter on feedback
            if (cp_lcf_fb_on) {
                old_s_ch1 = bf_lcf(old_s_ch1, cp_lcf_fb_freq, cp_lcf_fb_q, rate,
                    &self->fil_lcf_fb_ch1);
                old_s_ch2 = bf_lcf(old_s_ch2, cp_lcf_fb_freq, cp_lcf_fb_q, rate,
                    &self->fil_lcf_fb_ch2);
            }
        }

        /* Filtering before feedback loop */
        float cur_fil_s_ch1 = cur_s_ch1; // current filtered sample
        float cur_fil_s_ch2 = cur_s_ch2;
        if (cp_hcf_pre_on) {
            cur_fil_s_ch1 = bf_hcf(cur_fil_s_ch1, cp_hcf_pre_freq, cp_hcf_pre_q,
                rate, &self->fil_hcf_pre_ch1);
            cur_fil_s_ch2 = bf_hcf(cur_fil_s_ch2, cp_hcf_pre_freq, cp_hcf_pre_q,
                rate, &self->fil_hcf_pre_ch2);
        }

        if (cp_lcf_pre_on) {
            cur_fil_s_ch1 = bf_lcf(cur_fil_s_ch1, cp_lcf_pre_freq, cp_lcf_pre_q,
                rate, &self->fil_lcf_pre_ch1);
            cur_fil_s_ch2 = bf_lcf(cur_fil_s_ch2, cp_lcf_pre_freq, cp_lcf_pre_q,
                rate, &self->fil_lcf_pre_ch2);
        }

        /* Summing for the delay lines */
        if (cp_ping_pong) {
            /* In ping pong mode, we sum both input channels with -6 dBFS
            and send them solely to the buffer for the first channel.
            cur_cf-coeff takes care of the spill-over*/
            self->buffer_ch1[pos_w] = cur_gain_buf_in 
                * (cur_fil_s_ch1 * 0.5f + cur_fil_s_ch2 * 0.5f)
                + old_s_ch2 * cur_cf
            ;
            self->buffer_ch2[pos_w] = old_s_ch1 * cur_cf;
        }
        else {
            // Normal mode
            self->buffer_ch1[pos_w] = cur_gain_buf_in * cur_fil_s_ch1 
                + old_s_ch1 * cur_fb
                + old_s_ch2 * cur_cf
            ;

            self->buffer_ch2[pos_w] = cur_gain_buf_in * cur_fil_s_ch2
                + old_s_ch2 * cur_fb
                + old_s_ch1 * cur_cf
            ;
        }

        // Final summing
        self->output_ch1[i] = cur_s_ch1 * cur_gain_dry 
            + old_s_ch1 * cur_gain_wet;
        self->output_ch2[i] = cur_s_ch2 * cur_gain_dry 
            + old_s_ch2 * cur_gain_wet;

        // Increase write index, wrap around if needed
        pos_w = pos_w + 1 >= MAX_BUF_SIZE ? 0 : pos_w + 1;
    }

    // Copy state variables back to heap for next run
    self->cur_d_t_ch1 = cur_d_t_ch1;
    self->cur_d_t_ch2 = cur_d_t_ch2;
    self->cur_fb = cur_fb;
    self->cur_cf = cur_cf;
    self->cur_gain_buf_in = cur_gain_buf_in;
    self->cur_gain_dry = cur_gain_dry;
    self->cur_gain_wet = cur_gain_wet;
    self->cur_mod_depth = cur_mod_depth;
    self->cur_mod_rate = cur_mod_rate;
    self->cur_mod_phase = cur_mod_phase;
    self->fade_pos = fade_pos;
    self->lfo_curphase = lfo_curphase;
    self->lfo_incr = lfo_incr;
    self->pos_w = pos_w;
    self->state = state;
    self->tgt_d_t_ch1 = tgt_d_t_ch1;
    self->tgt_d_t_ch2 = tgt_d_t_ch2;
    self->tgt_gain_dry = tgt_gain_dry;
    self->tgt_gain_wet = tgt_gain_wet;
    self->tgt_cf = tgt_cf;
    self->tgt_fb = tgt_fb;
    self->lim_envelope_ch1 = lim_envelope_ch1;
    self->lim_envelope_ch2 = lim_envelope_ch2;
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
