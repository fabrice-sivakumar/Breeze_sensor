

/*

const int ledPin = 13; 
const int hallPin = 31;

int sensorValue; 


void setup(){
  nrf_gpio_pin_dir_set(hallPin, NRF_GPIO_PIN_DIR_INPUT);
  nrf_gpio_pin_dir_set(ledPin, NRF_GPIO_PIN_DIR_OUTPUT);

}


int main()
  {
    setup();
    while (1)
     {
      // lecture du capteur a Effet Hall
      sensorValue = nrf_gpio_pin_read(hallPin);
  
      // senseurValue = HIGH sans aimant
      // senseurValue = LOW  quand POLE SUD aimant
      sensorValue = !( sensorValue );
  
      // Allumer eteindre la LED
      nrf_gpio_pin_write(ledPin, sensorValue);
    }
}
*/


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_delay.h"
#include "nrf_log.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf51_bitfields.h"
#include "app_timer.h"
#include <math.h>

#define pinTrig 25
#define pinEcho 29 
#define pinTrig2 25 //Need to activate both trig otherwise an echo won't be activated
#define pinEcho2 11

 
static volatile uint32_t tCount = 0;
static volatile float countToUs = 1;

static volatile float distance = 0;
static volatile float distance2 = 0;

static volatile float duration = 0;
static volatile float duration2 = 0;
static volatile float duration_abs = 0;
static volatile float wind_speed = 0;
static volatile float wind_speed2 = 0;



void start_timer(void)
{
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->TASKS_CLEAR = 1;

    uint8_t prescaler = 0;
    NRF_TIMER1->PRESCALER = prescaler;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;

    uint16_t comp1 = 25;
    NRF_TIMER1->CC[1] = comp1;

    countToUs = 0.0625 * comp1 * (1 << prescaler);

    printf("timer tick = %f us\n", countToUs);

    NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
    NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);

    NVIC_EnableIRQ(TIMER1_IRQn);

    NRF_TIMER1->TASKS_START = 1;
}

void TIMER1_IRQHandler(void)
{
    if (NRF_TIMER1->EVENTS_COMPARE[1] && NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE1_Msk) {
        NRF_TIMER1->EVENTS_COMPARE[1] = 0;
        tCount++;
    }
}



bool measure_distance()
{
    nrf_gpio_pin_clear(pinTrig);
    nrf_delay_us(20);
    nrf_gpio_pin_set(pinTrig);
    nrf_delay_us(12);
    nrf_gpio_pin_clear(pinTrig);
    nrf_delay_us(20);
    while (!nrf_gpio_pin_read(pinEcho));
    tCount = 0;
    while (nrf_gpio_pin_read(pinEcho));
    duration = countToUs * tCount;
    distance = duration * 0.017;
    if (distance > 400.0)
      return false;
    return true;
}


bool measure_distance2()
{
    nrf_gpio_pin_clear(pinTrig2);
    nrf_delay_us(20);
    nrf_gpio_pin_set(pinTrig2);
    nrf_delay_us(12);
    nrf_gpio_pin_clear(pinTrig2);
    nrf_delay_us(20);
    while (!nrf_gpio_pin_read(pinEcho2));
    tCount = 0;
    while (nrf_gpio_pin_read(pinEcho2));
    duration2 = countToUs * tCount;
    distance2 = duration2 * 0.017;
    if (distance2 > 400.0)
      return false;
    return true;
}




void measure_duration()
{
    nrf_gpio_pin_clear(pinTrig);
    nrf_delay_us(20);
    nrf_gpio_pin_set(pinTrig);
    nrf_delay_us(12);
    nrf_gpio_pin_clear(pinTrig);
    nrf_delay_us(20);
    while (!nrf_gpio_pin_read(pinEcho2));
    tCount = 0;
    while (nrf_gpio_pin_read(pinEcho2));
    duration = countToUs * tCount;
    distance = duration * 0.034;

}

void measure_duration2()
{
    nrf_gpio_pin_clear(pinTrig);
    nrf_delay_us(20);
    nrf_gpio_pin_set(pinTrig);
    nrf_delay_us(12);
    nrf_gpio_pin_clear(pinTrig);
    nrf_delay_us(20);
    while (!nrf_gpio_pin_read(pinEcho));
    tCount = 0;
    while (nrf_gpio_pin_read(pinEcho));
    duration2 = countToUs * tCount;
    distance2 = duration2 * 0.034;
}


int main()
  {
    app_timer_init();
    start_timer();
    printf("Debut des mesures:\n");
    nrf_gpio_pin_dir_set(pinTrig, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(pinEcho, NRF_GPIO_PIN_DIR_INPUT);
    nrf_gpio_pin_dir_set(pinEcho2, NRF_GPIO_PIN_DIR_INPUT);

    while (1)
     {
        
        
        measure_duration();
        printf("Distance 1 : %.2f cm\n",distance);
        printf("Duration 1 : %.2f us\n",duration);
        nrf_delay_ms(250);
        measure_duration2();
        printf("Distance 2 : %.2f cm\n",distance2);
        printf("Duration 2 : %.2f us\n",duration2);
        
      

        if (measure_distance())
          printf("Distance 1 : %.2f cm\n",distance);
          printf("Duration 1 : %.2f us\n",duration);
        if (measure_distance2())
          printf("Distance 2 : %.2f cm\n",distance2);
          printf("Duration 2 : %.2f us\n",duration2);
        
        wind_speed = (0.12/2) * ((1000000/duration)- (1000000/duration2));  //Distance of 12cm between the 2 sensors
        float sonic_speed = (0.12/2) * ((1000000/duration) + (1000000/duration2));
        printf("Wind speed  : %f m/s \n",wind_speed);
        printf("Sonic speed  : %f m/s \n",sonic_speed);
        
        nrf_delay_ms(250);
    }
}










