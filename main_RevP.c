#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SAMPLES_IN_BUFFER 10

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;
uint32_t              resultat;
float                 tempC;
float                 windspeed = 1.23;
bool calcul= false;
// RESULT = [V(P) � V(N) ] * GAIN/REFERENCE * 2(RESOLUTION - m)
// (m=0) if CONFIG.MODE=SE, or (m=1) if CONFIG.MODE=Diff.

// V(N) = 0;
// GAIN = 1/6;
// REFERENCE Voltage = internal (0.6V);
// RESOLUTION : 12 bit;
// m = 0;

// 12bit
// V(P) = RESULT x REFERENCE / ( GAIN x RESOLUTION) = RESULT x (600 / (1/6 x 2^(12)) =  ADC_RESULT x 0.87890625;
//#define ADC_RESULT_IN_MILLI_VOLTS(ADC_RESULT) ((ADC_RESULT * 0.87890625))

// 14bit
// V(P) = RESULT x REFERENCE / ( GAIN x RESOLUTION) = RESULT x (600 / (1/6 x 2^(14)) =  ADC_RESULT x 0.2197265625;
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_RESULT) ((ADC_RESULT * 0.004833984375 ))


void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 100ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 1);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {     
        uint32_t buffer_samples = 0;
        uint16_t adc_result = 0;
        float adc_voltage_mv = 0;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        m_adc_evt_counter++;

        NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

        for (uint8_t i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            buffer_samples += p_event->data.done.p_buffer[i];
        }

        adc_result = buffer_samples/SAMPLES_IN_BUFFER;
        adc_voltage_mv = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
        resultat  = adc_result;
        NRF_LOG_INFO("adc result : %d.", adc_result);
        NRF_LOG_INFO("voltage : %d V.", adc_voltage_mv);
        NRF_LOG_INFO("Voltage en V : " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(adc_voltage_mv));
        calcul = true;
        NRF_LOG_INFO("---------------------------");
        
    }


}


void saadc_init(void)
{
    ret_code_t err_code;
    // Using the channel default single ended (SE) config
    // pin NRF_SAADC_INPUT_AIN1 : P0.03
    nrf_saadc_channel_config_t channel_config =
    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    channel_config.gain = NRF_SAADC_GAIN1_6;
    channel_config.reference = NRF_SAADC_REFERENCE_VDD4 ;
    // changing the saadc resolution to 10bit
    nrf_drv_saadc_config_t saadc_config ;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT;

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);

    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
    NRF_LOG_INFO("SAADC HAL simple example started.");

    while (1)
    {
        nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
        if (calcul)
        {
          calcul = false;
          float debut = resultat - 264.0;
          float debut1 = (debut/85.6814);
          if (debut1 > 0)
            windspeed = pow (debut1,3.36814);
          else
            windspeed = pow (-debut1,3.36814);
          NRF_LOG_INFO("Vent : " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(windspeed));
          printf("Wind speed : %.2f mph\t",windspeed);
          printf("\t %.2f m/s\n",windspeed*0.44704);
        }
    }
}


/** @} */