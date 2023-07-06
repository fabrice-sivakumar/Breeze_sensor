/**
 * @file 00std_rrdv_robot.c
 * @author Trifun Savic <trifun.savic@inria.fr>
 *
 * @brief This is an example application for Ultrasound ranging with HC-SR04 sensor and nRF52840.
 * This app receives a radio packet from the gateway, parse the packet and sets up the Timer offset for triggering the US ranging.
 * The US readings are then transmitted back to the gateway by radio using Long Range BLE.
 *
 * @copyright Inria, 2022
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "nrf.h"

//============================ defines ==========================================

// defines for the robot ID
#define DEVICE_ID_MASK   0xffffffffffff
#define MOTE_ID          ((((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) + (uint64_t)(NRF_FICR->DEVICEADDR[0])) & DEVICE_ID_MASK)
#define nRF1             0x041d8f7b3eb9
#define nRF2             0x0e519397b037
#define nRF3             0xf744aa09cc95
#define nRF4             0x71f564980c7d
#define nRF5             0x11837fde1015

// gateway
#define GATEWAY_ID 0x77

// radio
#define RADIO_BASE_ADDRESS_0       0x12345678UL 
#define RADIO_BASE_ADDRESS_1       0xFEDCBA98UL
#define NUMBER_OF_BYTES_IN_PACKET  5UL
#define NUMBER_OF_BYTES_RECEIVED   5UL
#define NUMBER_OF_BYTES_TO_SEND    5UL

// timer 
#define fromMsToTics     16000 // prescaler 0 -> 16Mhz timer
#define defaultCmpValue  0xDEADBEEF  // big value to initialize when we don't want timer to elapse
#define cmdDur_ms        0.75 
#define maxCmdDur_ms     1 
#define triggerOffset_ms 0.01 
#define triggerDur_ms    0.01 
#define echoDuration_ms  37.1  
#define tdmaTimeSlot_ms  1.1  
#define robotTxOffset_ms 1 
#define wdTx_ms          0.01 
#define notifDur_ms      0.65 
#define ifsDur_ms        0.3 
#define maxNotifDur_ms   1    
#define tdmaDelayTx(id) (tdmaTimeSlot_ms*id)
#define DURATION_rt1    maxCmdDur_ms * fromMsToTics 
//#define DURATION_rt2    triggerOffset_ms * fromMsToTics
#define DURATION_rt2    (cmdDur_ms + triggerOffset_ms) * fromMsToTics
#define DURATION_rt3    triggerDur_ms * fromMsToTics
#define DURATION_rt4    echoDuration_ms * fromMsToTics
#define DURATION_rt6    (robotTxOffset_ms + wdTx_ms) * fromMsToTics
#define DURATION_rt7    maxNotifDur_ms * fromMsToTics
#define DURATION_rt8    ifsDur_ms * fromMsToTics

// ppi
#define channel_ri0_or_ri8              0UL
#define channel_ri1                     1UL
#define channel_ri2                     2UL
#define channel_ri3                     3UL
#define channel_ri4                     4UL
#define channel_ri5                     5UL
#define channel_ri6                     6UL
#define channel_ri7_or_10               7UL
#define channel_ri9                     9UL
#define channel_rie0_or_rie1_or_rie2    11UL
#define ppiChannelClrEnable             1UL
#define ppiChannelSetEnable             1UL

// US sensor
// GPIOTE for turning ON US sensor and READING the range measurement 
#define US_ON_PORT              1UL        // output port number
#define US_ON_PIN               10UL       // output pin number
#define US_READ_PORT            1UL        
#define US_READ_PIN             6UL
#define US_ON_CH                0UL
#define US_READ_CH_LoToHi       1UL
#define US_READ_CH_HiToLo       2UL
#define US_ECHO_ERROR_VALUE     0x01f270   // 127600 us -> equivalent to 22 m distance 

// interrupts
#define TIMER0_INT_PRIORITY     0UL
#define RADIO_INT_PRIORITY      1UL
#define GPIOTE_INT_PRIORITY     2UL
 
// debug
#define FSM_DEBUG_PIN           31UL 
#define ISR_DEBUG_PIN           30UL

//=========================== typedefs ========================================

// FSM
typedef enum {
    S_RADIORXLISTEN   = 1UL,
    S_RADIORX         = 2UL,
    S_USPREPARE       = 3UL,
    S_TRIGGER         = 4UL,
    S_WAITECHOHIGH    = 5UL,
    S_WAITECHOLOW     = 6UL,
    S_ECHODONE        = 7UL,
    S_RADIOTXDELAY    = 8UL,
    S_RADIOTXPREPARE  = 9UL,
    S_RADIOTXREADY    = 10UL,
    S_RADIOTX         = 11UL,
    S_IFS             = 12UL,
    S_ECHOERROR       = 13UL
} robot_state_t;

typedef struct {
    //robot
    uint8_t       robot_id;
    uint32_t      robot_bitmask; // up to 16 robots
    double        DURATION_rt5;
    robot_state_t state;
    //radio
    uint8_t       dataToSendRadio[NUMBER_OF_BYTES_TO_SEND];
    uint8_t       dataReceivedRadio[NUMBER_OF_BYTES_RECEIVED];
    uint8_t       radioPacket[NUMBER_OF_BYTES_IN_PACKET];
    //us sensor
    uint32_t      timeEchoLowHigh;
    uint32_t      timeEchoHighLow;
    uint32_t      usEchoDuration;
    bool          usEchoDone;
} robot_vars_t;  

                           
//=========================== prototypes ========================================

// ISR
void activity_ri0(void);
void activity_ri1(void);
void activity_rie0(void);
void activity_ri2(void);
void activity_ri3(void);
void activity_ri4(uint32_t capturedTime);
void activity_ri5(uint32_t capturedTime);
void activity_ri6(void);
void activity_ri7(void);
void activity_rie1(void);
void activity_ri8(void);
void activity_rie2(void);
void activity_ri9(void);
void activity_ri10(void);
void activity_rie3(void);
void activity_rie4(void);

// Init
void gpiote_init(void); 
void timer_init(void);
void radio_init(void);
void radio_set_frequency(uint8_t freq);
void radio_rx_enable(void);
void hfclk_init(void);
void ppi_setup(void);

// Defaults
void robot_init_setup(void);
void changeState(robot_state_t newstate);
uint8_t get_mote_id(void);

// Debug
void set_debug_pins(void);
void toggle_fsm_debug_pin(void);
void toggle_isr_debug_pin(void);

//=========================== variables =========================================

robot_vars_t robot_vars;

//============================= main ============================================
int main(void) {
    
    // initialize variables
    memset(&robot_vars, 0, sizeof(robot_vars_t));
   
    gpiote_init();
    timer_init();
    radio_init();
    radio_set_frequency(100);
    ppi_setup();
    hfclk_init();

    set_debug_pins();
    robot_init_setup();  

    while (1) {        
        //__WFE();      
        //__SEV();
        //__WFE();
    }
    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//============================= ISRs ============================================

/**
@brief Indicates the FSM timer has fired.

This function executes in ISR mode, when the FSM timer fires.
*/

void TIMER0_IRQHandler(void){
    if(NRF_TIMER0->EVENTS_COMPARE[0] != 0)
    {   
        NRF_TIMER0->EVENTS_COMPARE[0] = 0; //Clear compare register 0 event	 
        
        switch (robot_vars.state) {
        case S_RADIORX:
            activity_rie0();
            break;       
        case S_USPREPARE:
            activity_ri2();
            break;  
        case S_TRIGGER:
            activity_ri3();
            break;      
        case S_ECHODONE:
            activity_ri6();
            break;      
        case S_WAITECHOLOW:
            activity_rie4();
            break; 
        case S_RADIOTXDELAY:
            activity_ri7();
            break; 
        case S_RADIOTXREADY:
            activity_rie1();
            break;    
        case S_RADIOTX:
            activity_rie2();
            break; 
        case S_IFS:
            activity_ri10();
            break;                                  
        default:
            printf("ERROR FSM STATE IN THE TIMER ISR\r\n");
            break;
        }   
    }
}

/**
 * @brief ISR for reading ranging value from the sensor. It captures the value of the Timer1 (pulse width) and calls the callback for US reading
 * 
 */
void GPIOTE_IRQHandler(void){   
    if(NRF_GPIOTE->EVENTS_IN[US_READ_CH_LoToHi] != 0) {
        NRF_GPIOTE->EVENTS_IN[US_READ_CH_LoToHi] = 0UL;

        switch (robot_vars.state) {
        case S_WAITECHOHIGH:
            // capture time when Echo pin goes from LOW to HIGH
            robot_vars.timeEchoLowHigh = NRF_TIMER0->CC[1];
            activity_ri4(robot_vars.timeEchoLowHigh);
            break;                                                  
        default:
            printf("ERROR FSM STATE IN GPIOTE LowToHigh ISR\r\n");
            break;
        }
    }
    if(NRF_GPIOTE->EVENTS_IN[US_READ_CH_HiToLo] != 0) {
        NRF_GPIOTE->EVENTS_IN[US_READ_CH_HiToLo] = 0UL;

        switch (robot_vars.state) {
        case S_WAITECHOLOW:
            // capture time when Echo pin goes from HIGH to LOW
            robot_vars.timeEchoHighLow = NRF_TIMER0->CC[2];
            activity_ri5(robot_vars.timeEchoHighLow);
            break;              
        case S_ECHOERROR:
            activity_rie3();
            break;                                      
        default:
            printf("ERROR FSM STATE IN GPIOTE HighToLow ISR\r\n");
            break;
        }
    }
}

/**
 * @brief Interruption handler for the Radio.
 *
 * This function will be called each time a radio packet is received.
 *
 */
void RADIO_IRQHandler(void) { 
    // Check if the interrupt was caused by the end of the frame
    if (NRF_RADIO->EVENTS_ADDRESS) {
        // Clear the Interrupt flag
        NRF_RADIO->EVENTS_ADDRESS = 0;
            
        switch (robot_vars.state) {
        case S_RADIORXLISTEN:
            activity_ri0();
            break;              
        case S_RADIOTXREADY:
            activity_ri8();
            break;                                      
        default:
            printf("ERROR FSM STATE IN RADIO ISR START OF FRAME\r\n");
            break;
        }
    }
    // Check if the interrupt was caused by the end of the frame
    if (NRF_RADIO->EVENTS_END) {
        // Clear the Interrupt flag
        NRF_RADIO->EVENTS_END = 0;

        switch (robot_vars.state) {
        case S_RADIORX:
            activity_ri1();
            break;              
        case S_RADIOTX:
            activity_ri9();
            break;                                      
        default:
            printf("ERROR FSM STATE IN RADIO ISR END OF FRAME\r\n");
            break;
        }
    }
}
//=========================== functions =========================================

void activity_ri0(void){
    toggle_isr_debug_pin();
    // start of the frame, this is where we synchronize robots
    changeState(S_RADIORX);

    // disable ppi channel for start of frame
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri7_or_10;  
  
    //set CC[0] to elapse when US trigger needs to be activated
    NRF_TIMER0->CC[0] = DURATION_rt1;

    // enable ppi channel for waiting the end of frame and clear the timer
    //NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri0_or_ri8;  

    // enable ppi channel for error state, we reach it if the timer elapses for the maxCmdDur_ms
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_rie0_or_rie1_or_rie2;
    toggle_isr_debug_pin();
}

void activity_ri1(void) {
    toggle_isr_debug_pin();
    uint32_t gateway_bitmask;

    // disable previous ppi channels
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri0_or_ri8; 
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_rie0_or_rie1_or_rie2; 

    // Copy packet into the Rx buffer
    memcpy(robot_vars.dataReceivedRadio, robot_vars.radioPacket, NUMBER_OF_BYTES_IN_PACKET);

    // packet received from the gateway, PPI already enabled when we end up here
    if(robot_vars.dataReceivedRadio[0] == GATEWAY_ID) {
        
        // parse the packet and see if the packet indicates to trigger US range
        gateway_bitmask       = 0;
        gateway_bitmask  = robot_vars.dataReceivedRadio[1] ;
        gateway_bitmask |= robot_vars.dataReceivedRadio[2] << 8;
        gateway_bitmask |= robot_vars.dataReceivedRadio[3] << 16;
        gateway_bitmask |= robot_vars.dataReceivedRadio[4] << 24;

        if ((gateway_bitmask & robot_vars.robot_bitmask) != 0) {
            // if we end up here the US ranging needs to be triggered, we let PPI do the work
            changeState(S_USPREPARE);

            // set CC[0] to a triggerOffset
            NRF_TIMER0->CC[0] = DURATION_rt2;

            // enable ppi channel for US sensor trigger pulse high, it happens after triggerOffset
            NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri1; 
        }
        else {
            // if we end up here, we received a packet from the gateway but we do not trigger the US range            
            changeState(S_RADIORXLISTEN);

            // set CC[0] to a default big value which should never be reached
            NRF_TIMER0->CC[0] = defaultCmpValue;
            // enable ppi channel for start of frame
            NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri7_or_10;  

            // Put Radio in Rx mode
            radio_rx_enable();

        }      
        // Clear the rx_buffer
        memset(robot_vars.dataReceivedRadio, 0, NUMBER_OF_BYTES_IN_PACKET);  
    }
    // packet not received from the gateway, drop the packet and go back to S_RADIORXLISTEN
    else {           
        changeState(S_RADIORXLISTEN);

        // set CC[0] to a default big value which should never be reached
        NRF_TIMER0->CC[0] = defaultCmpValue;
        // enable ppi channel for start of frame
        NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri7_or_10;  

        // Put Radio in Rx mode
        radio_rx_enable();
                
        // Clear the rx_buffer
        memset(robot_vars.dataReceivedRadio, 0, NUMBER_OF_BYTES_IN_PACKET);
    }
    toggle_isr_debug_pin();
}

void activity_rie0(void) {
    toggle_isr_debug_pin();
    printf("ERROR rie0 packet received too long\r\n");
    printf("current value of the timer: %d", NRF_TIMER0->CC[1]);    
    toggle_isr_debug_pin();
}

void activity_ri2(void) {
    toggle_isr_debug_pin();
    changeState(S_TRIGGER);

    // dissable ppi channel for setting High US sensor pin when timer elapses
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri1;

    //set CC[0] to elapse after defined US trigger duration
    NRF_TIMER0->CC[0] = DURATION_rt3;

    // enable ppi channel for setting Low US sensor pin when timer elapses
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri2;
    toggle_isr_debug_pin();
}

void activity_ri3(void) {
    toggle_isr_debug_pin();
    changeState(S_WAITECHOHIGH);

    // dissable ppi channel for setting Low US sensor pin when timer elapses
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri2;

    //set CC[0] to elapse after defined US trigger duration
    NRF_TIMER0->CC[0] = DURATION_rt4;

    // enable the ppi channel for capturing the time when the Echo pin goes High
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri3;
    
    // enable the ppi channel for watching where the Timer elapses after echoDuration
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri5;
    toggle_isr_debug_pin();
}

void activity_ri4(uint32_t capturedTime) {
    toggle_isr_debug_pin(); 
    changeState(S_WAITECHOLOW);

    // dissable the ppi channel for capturing the time when the Echo pin goes High
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri3;

    // save the time captured when Echo pin went from Low to High
    robot_vars.timeEchoLowHigh = capturedTime;

    // enable the ppi channel for capturing the time when the Echo pin goes Low
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri4;
    toggle_isr_debug_pin();
}

void activity_ri5(uint32_t capturedTime) {
    toggle_isr_debug_pin();
    changeState(S_ECHODONE);
    // set Echo Done flag
    robot_vars.usEchoDone   = true;
    // dissable the ppi channel for capturing the time when the Echo pin goes Low
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri4;

    // save the time captured when Echo pin went from High to Low
    robot_vars.timeEchoHighLow = capturedTime;
    // calculate the pulse duration
    robot_vars.usEchoDuration = (robot_vars.timeEchoHighLow - robot_vars.timeEchoLowHigh)/16;

    //Prepare the packet -> first byte is ROBOT ID
    robot_vars.dataToSendRadio[0] = robot_vars.robot_id;
    // send Ultrasound ranging data in us
    robot_vars.dataToSendRadio[1] = robot_vars.usEchoDuration;
    robot_vars.dataToSendRadio[2] = robot_vars.usEchoDuration >> 8;
    robot_vars.dataToSendRadio[3] = robot_vars.usEchoDuration >> 16;
    robot_vars.dataToSendRadio[4] = robot_vars.usEchoDuration >> 24;
    
    // Load the tx_buffer into memory.
    memcpy(robot_vars.radioPacket, robot_vars.dataToSendRadio, NUMBER_OF_BYTES_TO_SEND);

    // Clear the send buffer
    memset(robot_vars.dataToSendRadio, 0, NUMBER_OF_BYTES_IN_PACKET);
    //clear the variables
    robot_vars.timeEchoHighLow = 0;
    robot_vars.timeEchoLowHigh = 0;
    robot_vars.usEchoDuration  = 0;

    toggle_isr_debug_pin();
}

void activity_ri6(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIOTXDELAY);

    // dissable the ppi channel set to elapse when Echo measurements are collected
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri5;
    
    //set CC[0] to elapse after defined TDMA delay duration
    NRF_TIMER0->CC[0] = robot_vars.DURATION_rt5;
    
    // enable the ppi channel set when timer elapses after the TDMA delay, Radio Tx is also triggered after the timer elapses
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri6;
    toggle_isr_debug_pin();
}

void activity_ri7(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIOTXPREPARE);    
    
    // dissable the ppi channel set to start when the timer elapses after TDMA Tx delay
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri6;
    
    //set CC[0] to elapse after watchdog
    NRF_TIMER0->CC[0] = DURATION_rt6;
    
    // enable the ppi channel for waiting the start of the frame event
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri7_or_10;
    // enable ppi channel for error state, we reach it if the timer elapses after (robotTxOffset_ms + wdTx_ms)
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_rie0_or_rie1_or_rie2;

    changeState(S_RADIOTXREADY); 
    toggle_isr_debug_pin();
}

void activity_rie1(void) {
    toggle_isr_debug_pin();
    printf("ERROR rie1 packet not send at the right time\r\n");
    printf("current value of the timer: %d", NRF_TIMER0->CC[1]);
    toggle_isr_debug_pin();
}

void activity_ri8(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIOTX); 

    // dissable the ppi channel enabled in ri7
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri7_or_10;
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_rie0_or_rie1_or_rie2;
    
    //set CC[0] to elapse after defined watchdog set for message sending
    NRF_TIMER0->CC[0] = DURATION_rt7;
    
    // enable ppi channel for waiting the end of frame and clear the timer
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri0_or_ri8;
    // enable ppi channel for error state, we reach it if the timer elapses for the maxCmdDur_ms
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_rie0_or_rie1_or_rie2;
    toggle_isr_debug_pin();
}

void activity_rie2(void) {
    toggle_isr_debug_pin();    
    printf("ERROR rie2 packet too long\r\n");
    printf("current value of the timer: %d", NRF_TIMER0->CC[1]);
    toggle_isr_debug_pin();
}

void activity_ri9(void) {
    toggle_isr_debug_pin();
    changeState(S_IFS); 

    // dissable the ppi channels from ri8
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri0_or_ri8;
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_rie0_or_rie1_or_rie2;
    
    //set CC[0] to elapse after defined IFS duration
    NRF_TIMER0->CC[0] = DURATION_rt8;

    // enable ppi channel for waiting the timer for IFS to elapse
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri9;
    toggle_isr_debug_pin();
}

void activity_ri10(void) {
    toggle_isr_debug_pin();
    // if we end up here it means that whole US range cycle is completed and data sent back to the gateway
    // previously enabled ppi channel stays enabled because it enables the Radio Rx
    changeState(S_RADIORXLISTEN);     

    // disable the channels from ri9
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri9;

    // set CC[0] to a default big value which should never be reached
    NRF_TIMER0->CC[0] = defaultCmpValue;
        
    // enable the channel to clear the timer when the new frame is received
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri7_or_10;
    toggle_isr_debug_pin();
}

void activity_rie3(void) {
    toggle_isr_debug_pin();
    // if we end up here it means that the Echo pin was high too long and this function is called when it goes low to restart the FSM
    changeState(S_RADIORXLISTEN); 

    // set CC[0] to a default big value which should never be reached
    NRF_TIMER0->CC[0] = defaultCmpValue;
        
    // enable the channel to clear the timer when the new frame is received
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri7_or_10;

    // enable radio packet reception
    radio_rx_enable();

    toggle_isr_debug_pin();
}

void activity_rie4(void) {
    toggle_isr_debug_pin();
    // change state to Error - the Echo Pulse is too long i.e. the distance is >12 m
    changeState(S_ECHOERROR);    
        
    // dissable the ppi channel set to elapse when Echo measurements are collected
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_ri5;

    // set CC[0] to a default big value which should never be reached
    NRF_TIMER0->CC[0] = defaultCmpValue;

    //clear the variables for capturing echo pulse duration
    robot_vars.timeEchoHighLow = 0;
    robot_vars.timeEchoLowHigh = 0;
    robot_vars.usEchoDuration  = 0;
    printf("ERROR ECHO PULSE TOO LONG\r\n");
    toggle_isr_debug_pin();
}
/**
 * @brief Function for initializing GPIOTE for US trigger and US echo signals.
 */
void gpiote_init(void) {
    //configure US_ON as an output PIN which toggles 
    NRF_GPIOTE->CONFIG[US_ON_CH]          = (GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos)       |
                                            (US_ON_PIN                     << GPIOTE_CONFIG_PSEL_Pos)       |
                                            (US_ON_PORT                    << GPIOTE_CONFIG_PORT_Pos)       |
                                            (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)   |
                                            (GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos);
   
    // configure the US_READ as input PIN detecting low to high edge of the signal
    NRF_GPIOTE->CONFIG[US_READ_CH_LoToHi] = (GPIOTE_CONFIG_MODE_Event        << GPIOTE_CONFIG_MODE_Pos)     |
                                            (US_READ_PIN                     << GPIOTE_CONFIG_PSEL_Pos)     |
                                            (US_READ_PORT                    << GPIOTE_CONFIG_PORT_Pos)     |
                                            (GPIOTE_CONFIG_POLARITY_LoToHi   << GPIOTE_CONFIG_POLARITY_Pos);

    // configure the US_READ as input PIN detecting high to low edge of the signal
    NRF_GPIOTE->CONFIG[US_READ_CH_HiToLo] = (GPIOTE_CONFIG_MODE_Event        << GPIOTE_CONFIG_MODE_Pos)     |
                                            (US_READ_PIN                     << GPIOTE_CONFIG_PSEL_Pos)     |
                                            (US_READ_PORT                    << GPIOTE_CONFIG_PORT_Pos)     |
                                            (GPIOTE_CONFIG_POLARITY_HiToLo   << GPIOTE_CONFIG_POLARITY_Pos);
     
    // Configure the Interruptions
    NVIC_DisableIRQ(GPIOTE_IRQn); 

    // Enable interrupts for LoToGi and HiToLo edge of the input
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN1_Set << GPIOTE_INTENSET_IN1_Pos)|
                           (GPIOTE_INTENSET_IN2_Set << GPIOTE_INTENSET_IN2_Pos); 

    NVIC_SetPriority(GPIOTE_IRQn, GPIOTE_INT_PRIORITY);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);    // Clear the flag for any pending radio interrupt
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

/**
 * @brief This function sets up Timer0 used as FSM timer 
 * we use only CC[0] compare register
 */
void timer_init(void) {
    // configure the Timer for US on
    NRF_TIMER0->BITMODE   = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER0->PRESCALER = 0UL;

    // enable interrupts on compare 0 register
    NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);

    // Configure the Interruptions for the Timer0 used as the ON US sensor timer
    NVIC_DisableIRQ(TIMER0_IRQn); 
    NVIC_SetPriority(TIMER0_IRQn, TIMER0_INT_PRIORITY);
    NVIC_ClearPendingIRQ(TIMER0_IRQn);    // Clear the flag for any pending radio interrupt
    // enable interupts
    NVIC_EnableIRQ(TIMER0_IRQn);
       
}

/**
 * @brief Initializes the Long Range RADIO peripheral (125 kbps). 
 *
 * After this function you must explicitly set the frequency of the radio
 * with the db_radio_set_frequency function.
 *
 */
void radio_init(void) {

    // General configuration of the radio.
    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos); // 8dBm Power output

    NRF_RADIO->MODE  = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);       // Use Long Range 125 kbps modulation
    
    // Coded PHY (Long range)
    NRF_RADIO->PCNF0 =  (0                              << RADIO_PCNF0_S1LEN_Pos)    |
                        (1                              << RADIO_PCNF0_S0LEN_Pos)    |
                        (8                              << RADIO_PCNF0_LFLEN_Pos)    |
                        (3                              << RADIO_PCNF0_TERMLEN_Pos)  |
                        (2                              << RADIO_PCNF0_CILEN_Pos)    |
                        (RADIO_PCNF0_PLEN_LongRange     << RADIO_PCNF0_PLEN_Pos);

    NRF_RADIO->PCNF1 =  (RADIO_PCNF1_WHITEEN_Disabled   << RADIO_PCNF1_WHITEEN_Pos)  |
                        (RADIO_PCNF1_ENDIAN_Little      << RADIO_PCNF1_ENDIAN_Pos)   |
                        (3                              << RADIO_PCNF1_BALEN_Pos)    |
                        (0                              << RADIO_PCNF1_STATLEN_Pos)  |
                        (NUMBER_OF_BYTES_IN_PACKET      << RADIO_PCNF1_MAXLEN_Pos);

    // Configuring the on-air radio address
    NRF_RADIO->BASE0    = RADIO_BASE_ADDRESS_0; // base address for prefix 0
    NRF_RADIO->BASE1    = RADIO_BASE_ADDRESS_1; // base address for prefix 1-7

    NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos;
    NRF_RADIO->TXADDRESS   = (0 << RADIO_TXADDRESS_TXADDRESS_Pos) & RADIO_TXADDRESS_TXADDRESS_Msk;

    // CRC Config
    NRF_RADIO->CRCCNF  =  (RADIO_CRCCNF_LEN_Two         << RADIO_CRCCNF_LEN_Pos);        // Checksum uses 2 bytes, and is enabled.
    NRF_RADIO->CRCINIT =  0xFFFFUL;                                                      // initial value
    NRF_RADIO->CRCPOLY =  0x11021UL;                                                     // CRC poly: x^16 + x^12^x^5 + 1

    // pointer to packet payload
    NRF_RADIO->PACKETPTR = (uint32_t)robot_vars.radioPacket;

        // Configure the Short to expedite the packet transmission
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |  // yet the packet as soon as the radio is ready
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);   // disable the radio as soon as the packet is sent/received
   
    // Configure the Interruptions
    NVIC_DisableIRQ(RADIO_IRQn);                                                // Disable interruptions while configuring

    // Enable interruption for when at address and end events to call ISR at the start and end of the frame
    NRF_RADIO->INTENSET = (RADIO_INTENSET_ADDRESS_Enabled << RADIO_INTENSET_ADDRESS_Pos) | 
                          (RADIO_INTENSET_END_Enabled     << RADIO_INTENSET_END_Pos);

    NVIC_SetPriority(RADIO_IRQn, RADIO_INT_PRIORITY);                     // Set priority for Radio interrupts to 1
    NVIC_ClearPendingIRQ(RADIO_IRQn);                                           // Clear the flag for any pending radio interrupt
    
    // Enable Radio interruptions
    NVIC_EnableIRQ(RADIO_IRQn);
}

/**
 * @brief Set the tx-rx frequency of the radio, by the following formula
 *
 * Radio frequency 2400 + freq (MHz) [0, 100]
 *
 * @param[in] freq Frequency of the radio [0, 100]
 */
void radio_set_frequency(uint8_t freq) {

    NRF_RADIO->FREQUENCY = freq << RADIO_FREQUENCY_FREQUENCY_Pos;
}

/**
 * @brief This function puts radio in the Rx mode
 */
void radio_rx_enable(void){
    // Radio initially in Rx mode
    NRF_RADIO->EVENTS_RXREADY = 0;                                    // Clear the flag before enabling the Radio.
    NRF_RADIO->TASKS_RXEN     = RADIO_TASKS_RXEN_TASKS_RXEN_Trigger;  // Enable radio reception.
    while (NRF_RADIO->EVENTS_RXREADY == 0) {}                         // Wait for the radio to actually start receiving.
}

/**
 * @brief This function is public and it sets the PPI channels to allow the US trigger and US echo. Ranging starts by calling this function
 */
void ppi_setup(void) {  
    // GPIOTE
    uint32_t us_power_task_addr             = (uint32_t)&NRF_GPIOTE->TASKS_OUT[US_ON_CH];
    uint32_t us_read_event_low_high_addr    = (uint32_t)&NRF_GPIOTE->EVENTS_IN[US_READ_CH_LoToHi];
    uint32_t us_read_event_high_low_addr    = (uint32_t)&NRF_GPIOTE->EVENTS_IN[US_READ_CH_HiToLo];
    // TIMER
    uint32_t timer0_task_capture1_addr      = (uint32_t)&NRF_TIMER0->TASKS_CAPTURE[1];
    uint32_t timer0_task_capture2_addr      = (uint32_t)&NRF_TIMER0->TASKS_CAPTURE[2];
    uint32_t timer0_task_clear_addr         = (uint32_t)&NRF_TIMER0->TASKS_CLEAR;
    uint32_t timer0_task_stop_addr          = (uint32_t)&NRF_TIMER0->TASKS_STOP;
    uint32_t timer0_events_compare_0_addr   = (uint32_t)&NRF_TIMER0->EVENTS_COMPARE[0];   
    // RADIO
    uint32_t radio_events_address_addr      = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
    uint32_t radio_events_end_addr          = (uint32_t)&NRF_RADIO->EVENTS_END;
    uint32_t radio_tasks_txen_addr          = (uint32_t)&NRF_RADIO->TASKS_TXEN;
    uint32_t radio_tasks_rxen_addr          = (uint32_t)&NRF_RADIO->TASKS_RXEN;

    // set channels
    // enable in ri0 and in ri8 || dissable in ri1 and ri9 
    NRF_PPI->CH[channel_ri0_or_ri8].EEP         = radio_events_end_addr;
    NRF_PPI->CH[channel_ri0_or_ri8].TEP         = timer0_task_clear_addr;

    // enable in ri0 and ri7 and ri8 || dissable in ri1 and ri8 and ri9 
    NRF_PPI->CH[channel_rie0_or_rie1_or_rie2].EEP        = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_rie0_or_rie1_or_rie2].TEP        = timer0_task_stop_addr;
    NRF_PPI->FORK[channel_rie0_or_rie1_or_rie2].TEP      = timer0_task_capture1_addr;
    
    // enable in ri1 || dissable in ri2
    NRF_PPI->CH[channel_ri1].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_ri1].TEP         = us_power_task_addr;
    NRF_PPI->FORK[channel_ri1].TEP       = timer0_task_clear_addr;
    
    // enable in ri2 || dissable in ri3
    NRF_PPI->CH[channel_ri2].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_ri2].TEP         = us_power_task_addr;
    NRF_PPI->FORK[channel_ri2].TEP       = timer0_task_clear_addr;
    
    // enable in ri3 || dissable in ri4
    NRF_PPI->CH[channel_ri3].EEP         = us_read_event_low_high_addr;
    NRF_PPI->CH[channel_ri3].TEP         = timer0_task_capture1_addr;
    
    // enable in ri4 || dissable in ri5
    NRF_PPI->CH[channel_ri4].EEP         = us_read_event_high_low_addr;
    NRF_PPI->CH[channel_ri4].TEP         = timer0_task_capture2_addr;
    
    // enable in ri5 || dissable in ri6
    NRF_PPI->CH[channel_ri5].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_ri5].TEP         = timer0_task_clear_addr;
    
    // enable in ri6 || dissable in ri7
    NRF_PPI->CH[channel_ri6].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_ri6].TEP         = timer0_task_clear_addr;
    NRF_PPI->FORK[channel_ri6].TEP       = radio_tasks_txen_addr; 

    //enable in ri7 in ri10 || dissable in ri8 and ri0
    NRF_PPI->CH[channel_ri7_or_10].EEP   = radio_events_address_addr;
    NRF_PPI->CH[channel_ri7_or_10].TEP   = timer0_task_clear_addr;

    // enable in ri9 in ri10 or ri1(if the packet received is not for me)  || dissable in ri10 ri0
    NRF_PPI->CH[channel_ri9].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_ri9].TEP         = radio_tasks_rxen_addr;
    NRF_PPI->FORK[channel_ri9].TEP       = timer0_task_clear_addr;
     
}

/**
 * @brief This function enables HFCLK
 */
void hfclk_init(void) {   
    // Configure the external High-frequency Clock. (Needed for correct operation)
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0x00;    // Clear the flag
    NRF_CLOCK->TASKS_HFCLKSTART    = 0x01;    // Start the clock
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {;} // Wait for the clock to actually start.
}

/**
\brief Changes the state of the RRDV Robot FSM.
Besides simply updating the state global variable, this function toggles the FSM debug pin.
\param[in] newstate The state the RRDV Robot FSM is now in.
*/
void changeState(robot_state_t newstate) {
    // update the state
    robot_vars.state = newstate;

    // wiggle the FSM debug pin
    switch (robot_vars.state) {
        case S_RADIORXLISTEN:
        case S_RADIORX:
        case S_USPREPARE:
        case S_TRIGGER:
        case S_WAITECHOHIGH:
        case S_WAITECHOLOW:
        case S_ECHODONE:
        case S_RADIOTXDELAY:
        case S_RADIOTXPREPARE:
        case S_RADIOTXREADY:
        case S_RADIOTX:
        case S_IFS:
        case S_ECHOERROR:
            // toggle the pins
            toggle_fsm_debug_pin();
            break;
    }
}

/**
 * @brief This function returns the id of the NRF. We first need to read the MAC address from the device and hard code it as a Macro
 */
uint8_t get_mote_id(void) {
    uint64_t id;
    
    id = MOTE_ID & DEVICE_ID_MASK;

    if      (id == nRF1) {return 0x01;}
    else if (id == nRF2) {return 0x02;}
    else if (id == nRF3) {return 0x03;}
    else if (id == nRF4) {return 0x04;}
    else if (id == nRF5) {return 0x05;}
    else {printf("ERROR MOTE ID\r\n"); return 0;}
}

/**
 * @brief This function initializes the variables with default values and enables the reception of the packet 
 */ 
void robot_init_setup(void) {
    // DEFAULTS
    // get the robot ID 
    robot_vars.robot_id       = get_mote_id();
    // calculating the tdma offset for the given robot, depending on the ID assigned to it
    robot_vars.DURATION_rt5   = tdmaDelayTx(robot_vars.robot_id) * 16000;
    // create robot bitmask depending on the robot ID, used to check if the Gateway wants the robot to range
    robot_vars.robot_bitmask |= 1 << (robot_vars.robot_id - 1);
    printf("%d",robot_vars.robot_id);

    // default initial state for the RRDV ROBOT FSM
    robot_vars.state        = S_RADIORXLISTEN;
    // set CC[0] to a default big value which should never be reached
    NRF_TIMER0->CC[0]       = defaultCmpValue;
    // start the timer
    NRF_TIMER0->TASKS_START = (TIMER_TASKS_START_TASKS_START_Trigger << TIMER_TASKS_START_TASKS_START_Pos);  
    // us Echo Done flag
    robot_vars.usEchoDone   = false;

    radio_rx_enable();

    // default initial PPI channel enabled
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_ri7_or_10;  
}

/**
 * @brief This function sets the debug pins for ISR and FSM
 */ 
void set_debug_pins(void) {

    // For debug FSM Set the test pin (P0.31) as an output
    NRF_P0->PIN_CNF[FSM_DEBUG_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |  // Set Pin as output
                                     (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos); // Activate high current gpio mode.
    // HIGH by default
    NRF_P0->OUTSET = 1 << FSM_DEBUG_PIN;

    // For debug ISR Set the test pin (P0.30) as an output
    NRF_P0->PIN_CNF[ISR_DEBUG_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |  // Set Pin as output
                                     (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos); // Activate high current gpio mode.

}

/**
 * @brief This function toggles the FSM pin
 */ 
void toggle_fsm_debug_pin(void) {
    NRF_P0->OUT ^= 1 << FSM_DEBUG_PIN;
}

/**
 * @brief This function toggles the ISR pin
 */  
void toggle_isr_debug_pin(void) {
    // Toggle
    NRF_P0->OUT ^= 1 << ISR_DEBUG_PIN;
}

//========================== End of File ========================================
