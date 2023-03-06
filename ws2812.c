/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "kiss_fftr.h"
#include "ws2812.pio.h"

#define IS_RGBW false
#define NUM_PIXELS 144

#ifdef PICO_DEFAULT_WS2812_PIN
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#else
// default to pin 2 if the board doesn't have a default WS2812 pin defined
#define WS2812_PIN 6
#endif

// set this to determine sample rate
// 0     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 960
#define FSAMP 50000
#define NOISE 550

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 2
#define LED_PIN 25
// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 2048
// globals
dma_channel_config cfg;
uint dma_chan;
float freqs[NSAMP];
float max_volume = 0.1;

void setup();
void sample(uint8_t *capture_buf);

struct RGB
{
    int r;
    int g;
    int b;
};
struct RGB prevRGB;
#define NO_NOTES 56
struct Note
{
    char note[3];
    float frequency;
};

struct Note notes[NO_NOTES] = {{"C0", 16.35},
                               {"D0", 18.35},
                               {"E0", 20.6},
                               {"F0", 21.83},
                               {"G0", 24.50},
                               {"A0", 27.50},
                               {"B0", 30.87},
                               {"C1", 32.70},
                               {"D1", 36.71},
                               {"E1", 41.20},
                               {"F1", 43.65},
                               {"G1", 49},
                               {"A1", 55},
                               {"B1", 61.74},
                               {"C2", 65.41},
                               {"D2", 73.42},
                               {"E2", 82.41},
                               {"F2", 87.31},
                               {"G2", 98},
                               {"A2", 110},
                               {"B2", 123.47},
                               {"C3", 130.81},
                               {"D3", 146.83},
                               {"G3", 196},
                               {"B3", 246.94},
                               {"D4", 293.66},
                               {"F4", 349.23},
                               {"G4", 392},
                               {"A4", 440},
                               {"B4", 493.88},
                               {"C5", 523.25},
                               {"D5", 587.33},
                               {"E5", 659.25},
                               {"F5", 698.46},
                               {"G5", 783.99},
                               {"A5", 880},
                               {"B5", 987.77},
                               {"C6", 1046.5},
                               {"D6", 1174.66},
                               {"E6", 1318.51},
                               {"F6", 1396.91},
                               {"G6", 1567.98},
                               {"A6", 1760},
                               {"B6", 1975.53},
                               {"C7", 2093},
                               {"D7", 2349.32},
                               {"E7", 2637.02},
                               {"F7", 2793.83},
                               {"G7", 3135.96},
                               {"A7", 3520},
                               {"B7", 3951.07},
                               {"C8", 4186.01},
                               {"D8", 4698.63},
                               {"E8", 5274.04},
                               {"F8", 5587.65},
                               {"G8", 6271.93},
                               {"A8", 7040},
                               {"B8", 7902.13}};

struct Note closest_note(int freq)
{
    struct Note closestNote;
    float minDistance = 999999;
    float distance;
    for (int i = 0; i < NO_NOTES; i++)
    {
        distance = abs(freq - notes[i].frequency);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestNote = notes[i];
        }
    }
    return closestNote;
}
struct RGB freq_to_rgb(int freq)
{
    if (freq < 10000)
    {
        freq = freq % 800;
    }
    struct RGB rgb;
    rgb.r = 0;
    rgb.g = 0;
    rgb.b = 0;
    if (freq >= 10000 || freq == 50)
    {
        return rgb;
    }
    if (freq < 40)
    {
        rgb.r = 255;
        rgb.g = 0;
        rgb.b = 0;
    }
    else if (freq >= 40 && freq <= 77)
    {
        int br = (freq - 40) * (255 / 37.0000);
        rgb.r = 255;
        rgb.g = 0;
        rgb.b = br;
    }
    else if (freq > 77 && freq <= 205)
    {
        int r = 255 - ((freq - 78) * 2);
        rgb.r = r;
        rgb.g = 0;
        rgb.b = 255;
    }
    else if (freq >= 206 && freq <= 238)
    {
        int g = (freq - 206) * (255 / 32.0000);
        rgb.r = 0;
        rgb.g = g;
        rgb.b = 255;
    }
    else if (freq <= 239 && freq <= 250)
    {
        int r = (freq - 239) * (255 / 11.0000);
        rgb.r = r;
        rgb.g = 255;
        rgb.b = 255;
    }
    else if (freq >= 251 && freq <= 270)
    {
        rgb.r = 255;
        rgb.g = 255;
        rgb.b = 255;
    }
    else if (freq >= 271 && freq <= 398)
    {
        int rb = 255 - ((freq - 271) * 2);
        rgb.r = rb;
        rgb.g = 255;
        rgb.b = rb;
    }
    else if (freq >= 398 && freq <= 653)
    {
        rgb.r = 0;
        rgb.g = 255 - (freq - 398);
        rgb.b = (freq - 398);
    }
    else
    {
        rgb.r = 255;
        rgb.g = 0;
        rgb.b = 0;
    }

    return rgb;
}

struct RGB note_to_rgb(struct Note my_note)
{
    struct RGB rgb;
    if (my_note.note[0] == 'A')
    { // PURPLE
        rgb.r = 173;
        rgb.g = 99;
        rgb.b = 218;
    }
    else if (my_note.note[0] == 'B')
    { // BLUE
        rgb.r = 0;
        rgb.g = 0;
        rgb.b = 255;
    }
    else if (my_note.note[0] == 'C')
    { // ORANGE
        rgb.r = 234;
        rgb.g = 152;
        rgb.b = 0;
    }
    else if (my_note.note[0] == 'D')
    {
        // GREY
        rgb.r = 192;
        rgb.g = 192;
        rgb.b = 192;
    }
    else if (my_note.note[0] == 'E')
    { // RED
        rgb.r = 255;
        rgb.g = 0;
        rgb.b = 0;
    }
    else if (my_note.note[0] == 'F')
    { // PINK
        rgb.r = 244;
        rgb.g = 181;
        rgb.b = 224;
    }
    else if (my_note.note[0] == 'G')
    { // GREEN
        rgb.r = 0;
        rgb.g = 255;
        rgb.b = 0;
    }
    else
    { // BLACK
        rgb.r = 0;
        rgb.g = 0;
        rgb.b = 0;
    }
    return rgb;
}

static inline void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}
static inline uint32_t urgb_u32_second(struct RGB rgb)
{
    return ((uint32_t)(rgb.r) << 8) |
           ((uint32_t)(rgb.g) << 16) |
           (uint32_t)(rgb.b);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)(r) << 8) |
           ((uint32_t)(g) << 16) |
           (uint32_t)(b);
}

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}
void lightLEDS(struct RGB rgb, float percent_volume)
{
    float noVolumes = 10;
    if (percent_volume != 0)
        for (int i = 1; i <= noVolumes; i++)
        {
            float step = (1 / noVolumes) * i;
            if (percent_volume < step)
            {
                percent_volume = step;
                break;
            }
        }
    rgb.r = floor(rgb.r * percent_volume);
    rgb.g = floor(rgb.g * percent_volume);
    rgb.b = floor(rgb.b * percent_volume);
    struct RGB currRGB = prevRGB;
    int no_steps = 3;
    for (int i = 0; i < no_steps; i++)
    {
        currRGB.r = MAP(i, 0, no_steps - 1, prevRGB.r, rgb.r);
        currRGB.g = MAP(i, 0, no_steps - 1, prevRGB.g, rgb.g);
        currRGB.b = MAP(i, 0, no_steps - 1, prevRGB.b, rgb.b);
        for (int j = 0; j < NUM_PIXELS; j++)
        {
            put_pixel(urgb_u32_second(currRGB));
        }
        sleep_ms(1);
    }

    prevRGB = rgb;
}
void setupLEDS()
{
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
}

int main()
{
    uint8_t cap_buf[NSAMP];
    kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float
    kiss_fft_cpx fft_out[NSAMP];
    kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);
    int noComputings = 0;
    // setup ports and outputs
    setup();
    while (1)
    {
        if (noComputings == 10)
        {
            noComputings = 0;
            max_volume = 0;
        }
        // get NSAMP samples at FSAMP
        sample(cap_buf);
        // fill fourier transform input while subtracting DC component
        uint64_t sum = 0;
        for (int i = 0; i < NSAMP; i++)
        {   
            sum += cap_buf[i];
        }
        float avg = (float)sum / NSAMP;
        for (int i = 0; i < NSAMP; i++)
        {
            fft_in[i] = (float)cap_buf[i] - avg;
        }

        // compute fast fourier transform
        kiss_fftr(cfg, fft_in, fft_out);
        int noBiggest = 3;
        // compute power and calculate max freq component
        float max_power[noBiggest];
        max_power[0] = 0;
        max_power[1] = 0;
        max_power[2] = 0;
        int max_idx[noBiggest];
        max_idx[0] = 0;
        max_idx[1] = 0;
        max_idx[2] = 0;
        // any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
        for (int i = 0; i < NSAMP / 2; i++)
        {
            float power = fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i;
            if (power > max_power[0])
            {

                max_power[2] = max_power[1];
                max_idx[2] = max_idx[1];
                max_power[1] = max_power[0];
                max_idx[1] = max_idx[0];

                max_power[0] = power;
                max_idx[0] = i;
            }
        }

        float max_freq[noBiggest];
        for (int i = 0; i < noBiggest; i++)
        {
            max_freq[i] = freqs[max_idx[i]];
        }
        if (max_freq[0] > 10000 || max_freq[0] <= 200)
        {
            max_power[0] = 0;
        }
        if (max_volume < max_power[0] && max_power[0] > 0)
        {
            max_volume = max_power[0];
        }
        float percent_volume;
        if ((max_power[0] == max_volume && max_power[0] == 0) || max_power[0] == max_power[1])
            percent_volume = 0;
        else if (max_volume == 0)
            percent_volume = 1;
        else
            percent_volume = max_power[0] / max_volume;
        lightLEDS(note_to_rgb(closest_note(max_freq[0])), percent_volume);
        printf("Greatest Frequency Component: %0.1f Hz; Volume: %0.1f; Max_Volume: %0.1f; Percent_Volume: %0.2f;\n", max_freq[0], max_power[0], max_volume, percent_volume);
        printf("Closest notes: %s %s %s \n", closest_note(max_freq[0]).note, closest_note(max_freq[1]).note, closest_note(max_freq[2]).note);
        noComputings++;
    }

    // should never get here
    kiss_fft_free(cfg);
}

void sample(uint8_t *capture_buf)
{
    adc_fifo_drain();
    adc_run(false);

    dma_channel_configure(dma_chan, &cfg,
                          capture_buf,   // dst
                          &adc_hw->fifo, // src
                          NSAMP,         // transfer count
                          true           // start immediately
    );

    gpio_put(LED_PIN, 1);
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_chan);
    gpio_put(LED_PIN, 0);
}

void setup()
{
    stdio_init_all();
    setupLEDS();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_IN);

    adc_gpio_init(26 + CAPTURE_CHANNEL);

    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,  // Write each completed conversion to the sample FIFO
        true,  // Enable DMA data request (DREQ)
        1,     // DREQ (and IRQ) asserted when at least 1 sample present
        false, // We won't see the ERR bit because of 8 bit reads; disable.
        true   // Shift each sample to 8 bits when pushing to FIFO
    );

    // set sample rate
    adc_set_clkdiv(CLOCK_DIV);

    sleep_ms(1000);
    // Set up the DMA to start transferring data as soon as it appears in FIFO
    uint dma_chan = dma_claim_unused_channel(true);
    cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    // calculate frequencies of each bin
    float f_max = FSAMP;
    float f_res = f_max / NSAMP;
    for (int i = 0; i < NSAMP; i++)
    {
        freqs[i] = f_res * i;
    }
}