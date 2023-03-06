// Sample from the ADC continuously at a particular sample rate
// and then compute an FFT over the data
//
// much of this code is from pico-examples/adc/dma_capture/dma_capture.c
// the rest is written by Alex Wulff (www.AlexWulff.com)

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "kiss_fftr.h"
//----
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

#define IS_RGBW false
#define NUM_PIXELS 10

#ifdef PICO_DEFAULT_WS2812_PIN
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#else
// default to pin 2 if the board doesn't have a default WS2812 pin defined
#define WS2812_PIN 6
#endif

static inline void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)(r) << 8) |
           ((uint32_t)(g) << 16) |
           (uint32_t)(b);
}
//-----
// set this to determine sample rate
// 0     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 960
#define FSAMP 50000

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define LED_PIN 25
#define xRedPin 9
#define xGreenPin 8
#define xBluePin 7
// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 1000
#define PWM_PERIOD 65025
// globals
dma_channel_config cfg;
uint dma_chan;
float freqs[NSAMP];

void setup();
void sample(uint8_t *capture_buf);


  // lightLED(7,8,9);
void setupLEDS() {
  
	gpio_init(xRedPin);
	gpio_set_function(xRedPin, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xRedPin, PWM_PERIOD);
	uint slice_num = pwm_gpio_to_slice_num(xRedPin);
	pwm_set_enabled(slice_num, true);

	gpio_init(xGreenPin);
	gpio_set_function(xGreenPin, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGreenPin, PWM_PERIOD);
	slice_num = pwm_gpio_to_slice_num(xGreenPin);
	pwm_set_enabled(slice_num, true);

	gpio_init(xBluePin);
	gpio_set_function(xBluePin, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xBluePin, PWM_PERIOD);
	slice_num = pwm_gpio_to_slice_num(xBluePin);
	pwm_set_enabled(slice_num, true);

}
struct RGB {
  int r;
  int g;
  int b;
};
struct RGB freq_to_rgb(int freq) {
    if(freq < 15000) {
        freq = freq % 800;
    }
    struct RGB rgb;
    rgb.r = 0;
    rgb.g = 0;
    rgb.b = 0; 
    if (freq >= 15000) {
      return rgb;
    }
    if (freq < 40) {
        rgb.r = 255;
        rgb.g = 0;
        rgb.b = 0; 
    } else if (freq >= 40 && freq <= 77) {
        int br = (freq - 40) * (255 / 37.0000);
        rgb.r = 255;
        rgb.g = 0;
        rgb.b = br;
    } else if (freq > 77 && freq <= 205) {
        int r = 255 - ((freq - 78) * 2);
        rgb.r = r;
        rgb.g = 0;
        rgb.b = 255;
    } else if (freq >= 206 && freq <= 238) {
        int g = (freq - 206) * (255 / 32.0000);
        rgb.r = 0;
        rgb.g = g;
        rgb.b = 255;
    } else if (freq <= 239 && freq <= 250) {
        int r = (freq - 239) * (255 / 11.0000);
        rgb.r = r;
        rgb.g = 255;
        rgb.b = 255;
    } else if (freq >= 251 && freq <= 270) {
        rgb.r = 255;
        rgb.g = 255;
        rgb.b = 255;
    } else if (freq >= 271 && freq <= 398) {
        int rb = 255 - ((freq - 271) * 2);
        rgb.r = rb;
        rgb.g = 255;
        rgb.b = rb;
    } else if (freq >= 398 && freq <= 653) {
        rgb.r = 0;
        rgb.g = 255 - (freq - 398);
        rgb.b = (freq - 398);
    } else {
        rgb.r = 255;
        rgb.g = 0;
        rgb.b = 0;
    }

    return rgb;
}

void light_leds(int freq) {
    struct RGB rgb = freq_to_rgb(freq);
    pwm_set_gpio_level(xRedPin, rgb.r * PWM_PERIOD);
    pwm_set_gpio_level(xGreenPin, rgb.g * PWM_PERIOD);
    pwm_set_gpio_level(xBluePin, rgb.b * PWM_PERIOD);
}


int main()
{

    
  while(1) {

  }
  uint8_t cap_buf[NSAMP];
  kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float
  kiss_fft_cpx fft_out[NSAMP];
  kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);

  // setup ports and outputs
  setup();
  setupLEDS();

  while (1)
  {
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

    // compute power and calculate max freq component
    float max_power = 0;
    int max_idx = 0;
    // any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
    for (int i = 0; i < NSAMP / 2; i++)
    {
      float power = fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i;
      if (power > max_power)
      {
        max_power = power;
        max_idx = i;
      }
    }

    float max_freq = freqs[max_idx];
    printf("Greatest Frequency Component: %0.1f Hz\n", max_freq);
    light_leds(max_freq);
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