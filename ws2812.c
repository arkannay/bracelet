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

#include "btstack_run_loop.h"
#include "btstack_event.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/multicore.h"
#include "hal_led.h"
#include "btstack.h"
#include "ble/gatt-service/nordic_spp_service_server.h"
#include "mygatt.h"

#define IS_RGBW false
#define NUM_PIXELS 144

#ifdef PICO_DEFAULT_WS2812_PIN
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#else
// default to pin 2 if the board doesn't have a default WS2812 pin defined
#define WS2812_PIN 22
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
// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 2048
// globals
dma_channel_config cfg;
uint dma_chan;
float freqs[NSAMP];
float max_volume = 0.1;
static bool is_music_mode = false;
static uint32_t rgb = 0x00000000;
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static btstack_packet_callback_registration_t hci_event_callback_registration;
const uint8_t adv_data[] = {
    // Flags general discoverable, BR/EDR not supported
    2,
    BLUETOOTH_DATA_TYPE_FLAGS,
    0x06,
    // Name
    6,
    BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'A',
    'I',
    'L',
    'E',
    'D',
    // UUID ...
    17,
    BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x9e,
    0xca,
    0xdc,
    0x24,
    0xe,
    0xe5,
    0xa9,
    0xe0,
    0x93,
    0xf3,
    0xa3,
    0xb5,
    0x1,
    0x0,
    0x40,
    0x6e,
};
const uint8_t adv_data_len = sizeof(adv_data);

void setup();
void sample(uint8_t *capture_buf);

struct RGB
{
    int r;
    int g;
    int b;
};

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

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    switch (hci_event_packet_get_type(packet))
    {
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        con_handle = HCI_CON_HANDLE_INVALID;
        break;
    default:
        break;
    }
}

static void nordic_spp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{   
    UNUSED(channel);
    struct RGB currRGB;
    switch (packet_type)
    {
    case HCI_EVENT_PACKET:
        if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META)
            break;
        switch (hci_event_gattservice_meta_get_subevent_code(packet))
        {
        case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
            con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
            printf("Connected with handle 0x%04x\n", con_handle);
            break;
        case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
            con_handle = HCI_CON_HANDLE_INVALID;
            break;
        default:
            break;
        }
        break;
    case RFCOMM_DATA_PACKET:
        switch (packet[0])
        {
        case 0x72:
            rgb = 0x800000;
            currRGB.r = 255;
            currRGB.g = 0;
            currRGB.b = 0;
            is_music_mode = false;
            break;
        case 0x62:
            rgb = 0x80000000;
            currRGB.r = 0;
            currRGB.g = 255;
            currRGB.b = 0;
            is_music_mode = false;
            break;
        case 0x67:
            rgb = 0x008000;
            currRGB.r = 0;
            currRGB.g = 0;
            currRGB.b = 255;
            is_music_mode = false;
            break;
        case 0x01:
            is_music_mode = !is_music_mode;
            printf("music mode changed: %d\n", is_music_mode);
            if (!is_music_mode) {            
                currRGB.r = 0;
                currRGB.g = 0;
                currRGB.b = 0;
                rgb = 0x000000;
            }
            break;
        }
            multicore_fifo_push_blocking((uint32_t) is_music_mode);
            sleep_ms(100);
            printf("Got new rgb: %#010x\n", rgb);
            for (int i = 1; i <= NUM_PIXELS; i++)
                put_pixel(urgb_u32_second(currRGB));
        break;
        //   pio_sm_put_blocking(pio0,0,rgb);

    default:
        break;
    }
}

int setupBluetooth() {
    printf("Setting up bluetooth");
    if (cyw43_arch_init())
    {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();
    sm_init();
    att_server_init(profile_data, NULL, NULL);
    nordic_spp_service_server_init(&nordic_spp_packet_handler);

    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t *)adv_data);
    gap_advertisements_enable(1);

    hci_power_control(HCI_POWER_ON);
    return 0;
}

void core1_entry() {
    uint8_t cap_buf[NSAMP];
    kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float
    kiss_fft_cpx fft_out[NSAMP];
    kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);
    int noComputings = 0;
    uint32_t local_music_mode = (uint32_t) true;
    uint32_t status;
    while (1)
    {   
        status = multicore_fifo_get_status();
        if (multicore_fifo_get_status() == 11) {
            multicore_fifo_pop_timeout_us(100, &status);
            printf("new value from fifo: %d \n", status);
            local_music_mode = status;
        }
        if (!local_music_mode) {
            multicore_fifo_drain();
            sleep_ms(1000);
            continue;
        }
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
        lightLEDS(freq_to_rgb(max_freq[0]), 0.05);
        printf("Greatest Frequency Component: %0.1f Hz;\n", max_freq[0]);
        noComputings++;
    }

    // should never get here
    kiss_fft_free(cfg);
}

int main()
{
    // setup ports and outputs
    setup();
    multicore_launch_core1(core1_entry);
    btstack_run_loop_execute();
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

    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_chan);
}

void setup()
{
    stdio_init_all();
    setupLEDS();
    setupBluetooth();

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