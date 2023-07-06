/**
 * @file 00std_rrdv_robot.c
 * @author Trifun Savic <trifun.savic@inria.fr>
 *
 * @brief This is an example application for Ultrasound ranging Gateway 
 *
 * @copyright Inria, 2022
 *
 */
//TODO header

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "nrf.h"
#include "openhdlc.h"

//============================ defines ==========================================

// gateway
#define GATEWAY_ID       0x77
#define NUMBER_OF_ROBOTS 5UL

// radio
#define RADIO_BASE_ADDRESS_0             0x12345678UL 
#define RADIO_BASE_ADDRESS_1             0xFEDCBA98UL
#define NUMBER_OF_RADIO_BYTES_IN_PACKET  5UL
#define NUMBER_OF_RADIO_BYTES_RECEIVED   5UL
#define NUMBER_OF_RADIO_BYTES_TO_SEND    5UL

// timer
#define fromMsToTics     16000 // prescaler 0 -> 16Mhz timer 
#define defaultCmpValue  0xDEADBEEF  // big value to initialize when we don't want timer to elapse
#define uartOffset_ms      0.01
#define gatewayTxOffset_ms 1
#define cmdDur_ms          0.65 
#define maxCmdDur_ms       1 
#define wdTx_ms            1
#define triggerOffset_ms   0.01 
#define triggerDur_ms      0.01 
#define echoDuration_ms    37.14
#define rxPrepare_ms       1    
#define notifDur_ms        0.65 
#define tdmaTimeSlot_ms    1.1 
#define ifsDur_ms          0.3
#define radioRxOffset_ms   0.1
#define notifDurAll_ms     (tdmaTimeSlot_ms + radioRxOffset_ms + ifsDur_ms) * NUMBER_OF_ROBOTS
#define procDelay_ms       0.1//TODO
#define txUartPrepare_ms   0.1//TODO
#define maxUartTxDur_ms    10//TODO
#define DURATION_gt0       uartOffset_ms * fromMsToTics
#define DURATION_gt1       (gatewayTxOffset_ms + wdTx_ms) * fromMsToTics 
#define DURATION_gt2       maxCmdDur_ms * fromMsToTics
#define DURATION_gt3       (triggerOffset_ms + triggerDur_ms + echoDuration_ms)  * fromMsToTics
#define DURATION_gt4       rxPrepare_ms * fromMsToTics
#define DURATION_gt5       notifDurAll_ms * fromMsToTics
#define DURATION_gt6       (procDelay_ms + txUartPrepare_ms) * fromMsToTics
#define DURATION_gt7       maxUartTxDur_ms * fromMsToTics

// UART
#define UART_RX_PIN                 8UL // p0.08
#define UART_TX_PIN                 6UL // p0.06
#define UART_BAUDRATE_115200        0x01D7E000  // Baud 115200
#define UART_BAUDRATE_1M            0x10000000  // Baud 1M
#define UART_INTEN_RXDRDY_POS       2
#define UART_INTEN_TXDRDY_POS       7
#define UART_CONFIG_PARITY          0 // excluded
#define UART_CONFIG_PARITY_POS      1
#define UART_CONFIG_HWFC            0
#define UART_CONFIG_HWFC_POS        0
#define NUMBER_OF_UART_BYTES_TX     32UL
#define NUMBER_OF_UART_BYTES_RX     32UL

// HDLC
#define OUTPUT_BUFFER_MASK          0x1F

// ppi
#define channel_gi0                     0UL
#define channel_gi1                     1UL
#define channel_gi2                     2UL
#define channel_gi3                     3UL
#define channel_gi4                     4UL
#define channel_gi5                     5UL
#define channel_gi6                     6UL
#define channel_gi8                     8UL
#define channel_gi9                     9UL
#define channel_gie0_or_gie1_or_gie2    11UL
#define ppiChannelClrEnable             1UL
#define ppiChannelSetEnable             1UL

// interrupts
#define TIMER0_INT_PRIORITY     0UL
#define RADIO_INT_PRIORITY      1UL
#define UART_INT_PRIORITY       2UL
 
// debug
#define FSM_DEBUG_PIN           31UL 
#define ISR_DEBUG_PIN           30UL

//=========================== typedefs ========================================

// FSM
typedef enum {
    S_UARTRXENABLE      = 1UL,
    S_UARTRXDATA        = 2UL,
    S_RADIOTXPREPARE    = 3UL,
    S_RADIOTXREADY      = 4UL,
    S_RADIOTX           = 5UL,
    S_SLEEP             = 6UL,
    S_RADIORXREADY      = 7UL,
    S_RADIORXLISTEN     = 8UL,
    S_RADIORX           = 9UL,
    S_RADIORXPREPARE    = 10UL,
    S_UARTTXPREPARE     = 11UL,
    S_UARTTXREADY       = 12UL,
    S_UARTTXDATA        = 13UL,
    S_UARTRXDONE        = 14UL,
    S_UARTTXDONE        = 15UL
} gateway_state_t;

typedef struct {
    //gateway
    gateway_state_t state;
    uint8_t         robotUsTriggerMask;
    //radio
    uint8_t         dataToSendRadio[NUMBER_OF_RADIO_BYTES_TO_SEND];
    uint8_t         dataReceivedRadio[NUMBER_OF_RADIO_BYTES_RECEIVED];
    uint8_t         radioPacket[NUMBER_OF_RADIO_BYTES_IN_PACKET];

    //UART
    uint8_t         dataReceivedUart[NUMBER_OF_UART_BYTES_RX];
    uint8_t         dataToSendUart[NUMBER_OF_UART_BYTES_TX];
    // input
    uint8_t         inputBuf[NUMBER_OF_UART_BYTES_TX];
    uint8_t         inputBufFillLevel;
    uint8_t         hdlcLastRxByte;
    bool            hdlcBusyReceiving;
    uint16_t        hdlcInputCrc;
    bool            hdlcInputEscaping;
    // output
    uint8_t         outputBuf[NUMBER_OF_UART_BYTES_TX];
    uint16_t        outputBufIdxW;
    uint16_t        outputBufIdxR;
    bool            fBusyFlushing;
    uint16_t        hdlcOutputCrc;
    //temp
    uint8_t         uartLastRxByteIndex;
    uint8_t         uartLastTxByteIndex;
    uint8_t         uartLastTxByteSent;
} gateway_vars_t;  

                           
//=========================== prototypes ========================================

// ISR
void activity_gi0(void);
void activity_gi1(void);
void activity_gi2(void);
void activity_gie0(void);
void activity_gi3(void);
void activity_gie1(void);
void activity_gi4(void);
void activity_gi5(void);
void activity_gi6(void);
void activity_gi7(void);
void activity_gi8(void);
void activity_gi9(void);
void activity_gi10(void);
void activity_gie2(void);

// Uart
void uart_init(void);
uint8_t uart_readByte(void);
void uart_writeByte(uint8_t byteToWrite);
void handle_uart_rx_frame(void);
// HDLC output
void outputHdlcOpen(void);
void outputHdlcWrite(uint8_t b);
void outputHdlcClose(void);
// HDLC input
void inputHdlcOpen(void);
void inputHdlcWrite(uint8_t b);
void inputHdlcClose(void);

// Timer
void timer_init(void);
// Radio
void radio_init(void);
void radio_set_frequency(uint8_t freq);
void radio_rx_enable(void);
// PPI
void hfclk_init(void);
void ppi_setup(void);

// Defaults
void gateway_init_setup(void);
void changeState(gateway_state_t newstate);

// Debug
void set_debug_pins(void);
void toggle_fsm_debug_pin(void);
void toggle_isr_debug_pin(void);

//=========================== variables =========================================

gateway_vars_t gateway_vars;

//============================= main ============================================
int main(void) {
    
    // initialize variables
    memset(&gateway_vars, 0, sizeof(gateway_vars_t));
   
    uart_init();
    timer_init();
    radio_init();
    radio_set_frequency(100);
    ppi_setup();
    hfclk_init();

    set_debug_pins();
    gateway_init_setup();  

    while (1) {        
        __WFE();      
        __SEV();
        __WFE();
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
        NRF_TIMER0->EVENTS_COMPARE[0] = 0UL; //Clear compare register 0 event	 
        
        switch (gateway_vars.state) {
        case S_UARTRXDONE:
            activity_gi1();
            break;
        case S_RADIOTXREADY: 
            activity_gie0();
            break;      
        case S_RADIOTX: 
            activity_gie1();
            break;     
        case S_SLEEP: 
            activity_gi4();
            break;   
        case S_RADIORXREADY: 
            activity_gi5();
            break;  
        case S_RADIORXLISTEN: 
            activity_gi8();
            break;       
        case S_UARTTXREADY: 
            activity_gi9();
            break;
        case S_UARTTXDATA: 
            activity_gie2();
            break;                                                                                          
        default:
            printf("ERROR FSM STATE IN RADIO ISR TIMER\r\n");
            printf("Current state: %d\r\n",gateway_vars.state);
            break;
        }   
    }
}

/**
 * @brief ISR for handling UART Tx and Rx
 * 
 */
void UARTE0_UART0_IRQHandler(void){   
    if (NRF_UART0->EVENTS_RXDRDY != 0) {
        NRF_UART0->EVENTS_RXDRDY = 0UL;
       
        switch (gateway_vars.state) {
        case S_UARTRXENABLE: 
            activity_gi0();
            break;    
        case S_UARTRXDATA: 
            activity_gi0();
            break;                                                                    
        default:
            printf("ERROR FSM STATE IN UART ISR RECEIVE\r\n");
            printf("Current state: %d\r\n",gateway_vars.state);
            break;
        }
    }

    if (NRF_UART0->EVENTS_TXDRDY != 0) {       
        NRF_UART0->EVENTS_TXDRDY = 0UL;
        
        switch (gateway_vars.state) {
        case S_UARTTXREADY: 
            activity_gi9();
            break;    
        case S_UARTTXDATA: 
            activity_gi9();
            break;         
        case S_UARTTXDONE: 
            activity_gi10();
            break;                                                   
        default:
            printf("ERROR FSM STATE IN UART ISR SEND\r\n");
            printf("Current state: %d\r\n",gateway_vars.state);
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
        NRF_RADIO->EVENTS_ADDRESS = 0UL;
            
        switch (gateway_vars.state) {
        case S_RADIOTXREADY: 
            activity_gi2();
            break;    
        case S_RADIORXLISTEN: 
            activity_gi6();
            break;                                                           
        default:
            printf("ERROR FSM STATE IN RADIO ISR START OF FRAME\r\n");
            printf("Current state: %d\r\n",gateway_vars.state);
            break;
        }
    }
    // Check if the interrupt was caused by the end of the frame
    if (NRF_RADIO->EVENTS_END) {
        // Clear the Interrupt flag
        NRF_RADIO->EVENTS_END = 0;

        switch (gateway_vars.state) { 
        case S_RADIOTX: 
            activity_gi3();
            break;              
        case S_RADIORX: 
            activity_gi7();
            break;                                                 
        default:
            printf("ERROR FSM STATE IN RADIO ISR END OF FRAME\r\n");
            printf("Current state: %d\r\n",gateway_vars.state);
            break;
        }
    }
}

//=========================== functions =========================================

void activity_gi0(void){
    uint8_t rxbyte;
    toggle_isr_debug_pin();

    rxbyte = uart_readByte();
    
    if (
            gateway_vars.hdlcBusyReceiving == false &&
            gateway_vars.hdlcLastRxByte == HDLC_FLAG &&
            rxbyte != HDLC_FLAG
            ) {
        // start of frame
        // switch state to S_UARTRXDATA at the start of the frame
        changeState(S_UARTRXDATA); 
        // I'm now receiving
        gateway_vars.hdlcBusyReceiving = true;

        // create the HDLC frame
        inputHdlcOpen();

        // add the byte just received
        inputHdlcWrite(rxbyte);
    } 
    else if (
            gateway_vars.hdlcBusyReceiving == true &&
            rxbyte != HDLC_FLAG
            ) {
        // middle of frame

        // add the byte just received
        inputHdlcWrite(rxbyte);
        if (gateway_vars.inputBufFillLevel + 1 > NUMBER_OF_UART_BYTES_RX) {
            // push task
            printf("HDLC: INPUT BUFFER OVERFLOW");
            gateway_vars.inputBufFillLevel = 0;
            gateway_vars.hdlcBusyReceiving = false;
        }
    } 
    else if (
            gateway_vars.hdlcBusyReceiving == true &&
            rxbyte == HDLC_FLAG
            ) {
        // end of frame

        // finalize the HDLC frame
        inputHdlcClose();
        gateway_vars.hdlcBusyReceiving = false;

        if (gateway_vars.inputBufFillLevel == 0) {
            // push task
            printf("HDLC: WRONG CRC INPUT");
        } 
        else { 
        // HDLC frame with good CRC received            
            // change state
            changeState(S_UARTRXDONE);        

            // clear the timer
            NRF_TIMER0->TASKS_CLEAR = (TIMER_TASKS_CLEAR_TASKS_CLEAR_Trigger << TIMER_TASKS_CLEAR_TASKS_CLEAR_Pos);

            //set CC[0] to elapse after duration gt0
            NRF_TIMER0->CC[0] = DURATION_gt0;

            // enable ppi channels for gi1
            NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gi0;          
        }
    }

    gateway_vars.hdlcLastRxByte = rxbyte;

    toggle_isr_debug_pin();
}

void activity_gi1(void) {
    toggle_isr_debug_pin();
    
    changeState(S_RADIOTXPREPARE);

    // disable ppi channels for gi0
    NRF_PPI->CHENSET = ppiChannelClrEnable << channel_gi0;
     
    //set CC[0] to elapse after duration gt1
    NRF_TIMER0->CC[0] = DURATION_gt1;

    // enable ppi channels for gi1
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gi1;
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gie0_or_gie1_or_gie2;

    // stop Uart Rx
    NRF_UART0->TASKS_STOPRX = 1UL;

    handle_uart_rx_frame();

    // enable Radio Tx
    NRF_RADIO->TASKS_TXEN = 1UL;

    changeState(S_RADIOTXREADY);
    toggle_isr_debug_pin();
}

void activity_gi2(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIOTX);
    // dissable the ppi channel enabled in gi1
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gi1;
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gie0_or_gie1_or_gie2;
    
    //set CC[0] to elapse after defined duration gt2
    NRF_TIMER0->CC[0] = DURATION_gt2;

    // enable ppi channels for gi2
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gi2;
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gie0_or_gie1_or_gie2;

    toggle_isr_debug_pin();
}

void activity_gie0(void) {
    toggle_isr_debug_pin();
    printf("ERROR gie0 command not sent at the right time\r\n");
    printf("current value of the timer: %d", NRF_TIMER0->CC[1]);    
    toggle_isr_debug_pin();
}

void activity_gi3(void) {
    toggle_isr_debug_pin();
    changeState(S_SLEEP);

    // dissable the ppi channel enabled in gi2
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gi2;
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gie0_or_gie1_or_gie2;

    //set CC[0] to elapse after defined duration gt3
    NRF_TIMER0->CC[0] = DURATION_gt3;

    // enable ppi channels for gi3
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gi3;

    toggle_isr_debug_pin();
}

void activity_gie1(void) {
    toggle_isr_debug_pin();
    printf("ERROR gie1 command too long\r\n");
    printf("current value of the timer: %d", NRF_TIMER0->CC[1]);
    toggle_isr_debug_pin();
}

void activity_gi4(void) {
    toggle_isr_debug_pin(); 
    changeState(S_RADIORXREADY);
    
    // dissable the ppi channel enabled in gi3
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gi3;

    //set CC[0] to elapse after defined duration gt3
    NRF_TIMER0->CC[0] = DURATION_gt4;
    
    // Disable the Short to expedite the packet transmission
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |  
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

    // Enable the Short to expedite the packet reception
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |  // yet the packet as soon as the radio is ready
                        (RADIO_SHORTS_END_START_Enabled   << RADIO_SHORTS_END_START_Pos);
       
    // enable ppi channels for gi4
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gi4;

    toggle_isr_debug_pin();
}

void activity_gi5(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIORXLISTEN);

    // dissable the ppi channel enabled in gi4
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gi4;

    //set CC[0] to elapse after defined duration gt3
    NRF_TIMER0->CC[0] = DURATION_gt5;

    // enable ppi channels for gi5
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gi5;    
 
    // Write a opening flag to HDLC frame to transmit
    outputHdlcOpen();

    toggle_isr_debug_pin();
}

void activity_gi6(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIORX);

    //TODO

    toggle_isr_debug_pin();
}

void activity_gi7(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIORXPREPARE);  

    // Copy packet into the Rx buffer.
    memcpy(gateway_vars.dataReceivedRadio, gateway_vars.radioPacket, NUMBER_OF_RADIO_BYTES_IN_PACKET);

    // start filling the HDLC frame 
    outputHdlcWrite(gateway_vars.dataReceivedRadio[0]);
    outputHdlcWrite(gateway_vars.dataReceivedRadio[1]);
    outputHdlcWrite(gateway_vars.dataReceivedRadio[2]);
    outputHdlcWrite(gateway_vars.dataReceivedRadio[3]);
    outputHdlcWrite(gateway_vars.dataReceivedRadio[4]);

    // Clear the Radio Rx buffer
    memset(gateway_vars.dataReceivedRadio, 0, NUMBER_OF_RADIO_BYTES_RECEIVED);
    
    changeState(S_RADIORXLISTEN);
    toggle_isr_debug_pin();
}

void activity_gi8(void) {
    toggle_isr_debug_pin();
    changeState(S_UARTTXPREPARE); 

    // dissable the ppi channel enabled in gi5
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gi5;

    //set CC[0] to elapse after defined duration gt6
    NRF_TIMER0->CC[0] = DURATION_gt6;

    NRF_RADIO->TASKS_DISABLE = 1;

    // Disable the Short to expedite the packet reception 
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Disabled << RADIO_SHORTS_READY_START_Pos) |  // yet the packet as soon as the radio is ready
                        (RADIO_SHORTS_END_START_Disabled   << RADIO_SHORTS_END_START_Pos);
    // Enable the Short to expedite the packet transmission
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |  
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

    // enable ppi channels for gi8
    NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gi8;   

    // write final CRC to HDLC frame
    outputHdlcClose();
    
    changeState(S_UARTTXREADY);     
    toggle_isr_debug_pin();
}

void activity_gi9(void) {
    toggle_isr_debug_pin();  
    // I have some bytes to transmit
    if (gateway_vars.outputBufIdxW != gateway_vars.outputBufIdxR) {     
         // check if this is the first byte to send
        if (gateway_vars.outputBufIdxR == 0) {
            // when sending the first byte
            changeState(S_UARTTXDATA); // switch state to S_UARTTXDATA at the start of the frame

            // dissable the ppi channel enabled in g8
            NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gi8;
     
            //set CC[0] to elapse after defined duration gt7
            NRF_TIMER0->CC[0] = DURATION_gt7;

            NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gie0_or_gie1_or_gie2;
        }  
        // I have a last byte to send
        if(gateway_vars.outputBufIdxW == (gateway_vars.outputBufIdxR + 1)) {
            // before sending last byte      
            changeState(S_UARTTXDONE);
        
            // enable ppi channels for gi9 before sending the last byte.
            NRF_PPI->CHENSET = ppiChannelSetEnable << channel_gi9;   
        }
        // send byte
        uart_writeByte(gateway_vars.outputBuf[OUTPUT_BUFFER_MASK & (gateway_vars.outputBufIdxR++)]);
        //gateway_vars.fBusyFlushing = true;
    }

    toggle_isr_debug_pin();
}

void activity_gi10(void) {
    toggle_isr_debug_pin();    
    changeState(S_UARTRXENABLE);
    
    // dissable the ppi channel enabled in g9
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gi9;
    NRF_PPI->CHENCLR = ppiChannelClrEnable << channel_gie0_or_gie1_or_gie2;

    // set CC[0] to a default big value which should never be reached
    NRF_TIMER0->CC[0] = defaultCmpValue;

    gateway_vars.outputBufIdxR = 0;
    gateway_vars.outputBufIdxW = 0;

    // start listening to new HDLC frame
    NRF_UART0->TASKS_STARTRX = 1UL;

    toggle_isr_debug_pin();
}

void activity_gie2(void) {
    toggle_isr_debug_pin();    
    printf("ERROR gie2 UART packet too long\r\n");
    printf("current value of the timer: %d", NRF_TIMER0->CC[1]);
    toggle_isr_debug_pin();
}

/**
 * @brief Function for initializing UART
 */
void uart_init(void) {
    // configure txd and rxd pin
    NRF_P0->OUTSET =  1 << UART_TX_PIN;

    // tx pin configured as output
    NRF_P0->PIN_CNF[UART_TX_PIN]  =  ((uint32_t)GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)         |
                                     ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) | 
                                     ((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)     |
                                     ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)       |
                                     ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
 
    // rx pin configured as input
    NRF_P0->PIN_CNF[UART_RX_PIN] =  ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)       | 
                                    ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                    ((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)  |
                                    ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)    |
                                    ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    // configure uart
    NRF_UART0->BAUDRATE = (uint32_t)(UART_BAUDRATE_1M);
    NRF_UART0->CONFIG   = (uint32_t)(UART_CONFIG_PARITY << UART_CONFIG_PARITY_POS) |
                          (uint32_t)(UART_CONFIG_HWFC   << UART_CONFIG_HWFC_POS);

    NRF_UART0->PSEL.RXD = (uint32_t)UART_RX_PIN;
    NRF_UART0->PSEL.TXD = (uint32_t)UART_TX_PIN;

    // enable UART rx done ready and tx done ready interrupts

    NRF_UART0->INTENSET = (uint32_t)(1<<UART_INTEN_RXDRDY_POS) |
                          (uint32_t)(1<<UART_INTEN_TXDRDY_POS);

    // enable uart
    NRF_UART0->ENABLE = (uint32_t)UART_ENABLE_ENABLE_Enabled;

    // set priority and enable interrupt in NVIC
    NVIC_SetPriority(UARTE0_UART0_IRQn, UART_INT_PRIORITY);
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);
}

/**
 * @brief This function writes the byte to send
 */
void uart_writeByte(uint8_t byteToWrite){
     NRF_UART0->TXD = byteToWrite;
}

/**
 * @brief This function returns the byte received
 */
uint8_t uart_readByte(void) {
    return NRF_UART0->RXD;
}

//===== hdlc (output)

/**
\brief Start an HDLC frame in the output buffer.
*/
void outputHdlcOpen(void) {
    // initialize the value of the CRC

    gateway_vars.hdlcOutputCrc = HDLC_CRCINIT;

    // write the opening HDLC flag
    gateway_vars.outputBuf[OUTPUT_BUFFER_MASK & (gateway_vars.outputBufIdxW++)] = HDLC_FLAG;

}

/**
\brief Add a byte to the outgoing HDLC frame being built.
*/
void outputHdlcWrite(uint8_t b) {

    // iterate through CRC calculator
    gateway_vars.hdlcOutputCrc = crcIteration(gateway_vars.hdlcOutputCrc, b);

    // add byte to buffer
    if (b == HDLC_FLAG || b == HDLC_ESCAPE) {
        gateway_vars.outputBuf[OUTPUT_BUFFER_MASK & (gateway_vars.outputBufIdxW++)] = HDLC_ESCAPE;
        b = b ^ HDLC_ESCAPE_MASK;
    }
    gateway_vars.outputBuf[OUTPUT_BUFFER_MASK & (gateway_vars.outputBufIdxW++)] = b;

}

/**
\brief Finalize the outgoing HDLC frame.
*/
void outputHdlcClose(void) {
    uint16_t finalCrc;

    // finalize the calculation of the CRC
    finalCrc = ~gateway_vars.hdlcOutputCrc;

    // write the CRC value
    outputHdlcWrite((finalCrc >> 0) & 0xff);
    outputHdlcWrite((finalCrc >> 8) & 0xff);

    // write the closing HDLC flag
    gateway_vars.outputBuf[OUTPUT_BUFFER_MASK & (gateway_vars.outputBufIdxW++)] = HDLC_FLAG;
}

//===== hdlc (input)

/**
\brief Start an HDLC frame in the input buffer.
*/
void inputHdlcOpen(void) {
    // reset the input buffer index
    gateway_vars.inputBufFillLevel = 0;

    // initialize the value of the CRC
    gateway_vars.hdlcInputCrc = HDLC_CRCINIT;
}

/**
\brief Add a byte to the incoming HDLC frame.
*/
void inputHdlcWrite(uint8_t b) {
    if (b == HDLC_ESCAPE) {
        gateway_vars.hdlcInputEscaping = true;
    } else {
        if (gateway_vars.hdlcInputEscaping == true) {
            b = b ^ HDLC_ESCAPE_MASK;
            gateway_vars.hdlcInputEscaping = false;
        }

        // add byte to input buffer
        gateway_vars.inputBuf[gateway_vars.inputBufFillLevel] = b;
        gateway_vars.inputBufFillLevel++;

        // iterate through CRC calculator
        gateway_vars.hdlcInputCrc = crcIteration(gateway_vars.hdlcInputCrc, b);
    }
}

/**
\brief Finalize the incoming HDLC frame.
*/
void inputHdlcClose(void) {

    // verify the validity of the frame
    if (gateway_vars.hdlcInputCrc == HDLC_CRCGOOD) {
        // the CRC is correct

        // remove the CRC from the input buffer
        gateway_vars.inputBufFillLevel -= 2;
    } else {
        // the CRC is incorrect

        // drop the incoming frame
        gateway_vars.inputBufFillLevel = 0;
    }
}

uint16_t crcIteration(uint16_t crc, uint8_t byte) {
   return (crc >> 8) ^ fcstab[(crc ^ byte) & 0xff];
}

void handle_uart_rx_frame(void) {
    // prepare the Radio packet
    gateway_vars.dataToSendRadio[0] = GATEWAY_ID;

    gateway_vars.dataToSendRadio[1] = gateway_vars.inputBuf[0];
    gateway_vars.dataToSendRadio[2] = gateway_vars.inputBuf[1];
    gateway_vars.dataToSendRadio[3] = gateway_vars.inputBuf[2];
    gateway_vars.dataToSendRadio[4] = gateway_vars.inputBuf[3];

    gateway_vars.inputBufFillLevel = 0;

    // Load the tx_buffer into memory.
    memcpy(gateway_vars.radioPacket, gateway_vars.dataToSendRadio, NUMBER_OF_RADIO_BYTES_TO_SEND);

    // Clear the radio Tx buffer
    memset(gateway_vars.dataToSendRadio, 0, NUMBER_OF_RADIO_BYTES_TO_SEND);
    // Clear the UART Rx buffer
    memset(gateway_vars.inputBuf, 0, NUMBER_OF_UART_BYTES_RX);
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
    NVIC_ClearPendingIRQ(TIMER0_IRQn);    // Clear the flag for any pending timer interrupt
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
    NRF_RADIO->PCNF0 =  (0                               << RADIO_PCNF0_S1LEN_Pos)    |
                        (1                               << RADIO_PCNF0_S0LEN_Pos)    |
                        (8                               << RADIO_PCNF0_LFLEN_Pos)    |
                        (3                               << RADIO_PCNF0_TERMLEN_Pos)  |
                        (2                               << RADIO_PCNF0_CILEN_Pos)    |
                        (RADIO_PCNF0_PLEN_LongRange      << RADIO_PCNF0_PLEN_Pos);

    NRF_RADIO->PCNF1 =  (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos)  |
                        (RADIO_PCNF1_ENDIAN_Little       << RADIO_PCNF1_ENDIAN_Pos)   |
                        (3                               << RADIO_PCNF1_BALEN_Pos)    |
                        (0                               << RADIO_PCNF1_STATLEN_Pos)  |
                        (NUMBER_OF_RADIO_BYTES_IN_PACKET << RADIO_PCNF1_MAXLEN_Pos);

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
    NRF_RADIO->PACKETPTR = (uint32_t)gateway_vars.radioPacket;

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
    // UART
    uint32_t uart0_events_rxdrdy_addr       = (uint32_t)&NRF_UART0->EVENTS_RXDRDY;
    uint32_t uart0_events_txdrdy_addr       = (uint32_t)&NRF_UART0->EVENTS_TXDRDY;
    uint32_t uart0_task_starttx_addr        = (uint32_t)&NRF_UART0->TASKS_STARTTX;
    uint32_t uart0_task_startrx_addr        = (uint32_t)&NRF_UART0->TASKS_STARTRX;
    uint32_t uart0_task_stoptx_addr         = (uint32_t)&NRF_UART0->TASKS_STOPTX;
    uint32_t uart0_task_stoprx_addr         = (uint32_t)&NRF_UART0->TASKS_STOPRX;

    // TIMER
    uint32_t timer0_task_capture1_addr      = (uint32_t)&NRF_TIMER0->TASKS_CAPTURE[1];
    uint32_t timer0_task_clear_addr         = (uint32_t)&NRF_TIMER0->TASKS_CLEAR;
    uint32_t timer0_task_stop_addr          = (uint32_t)&NRF_TIMER0->TASKS_STOP;
    uint32_t timer0_events_compare_0_addr   = (uint32_t)&NRF_TIMER0->EVENTS_COMPARE[0];   
    // RADIO
    uint32_t radio_events_address_addr      = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
    uint32_t radio_events_end_addr          = (uint32_t)&NRF_RADIO->EVENTS_END;
    uint32_t radio_tasks_txen_addr          = (uint32_t)&NRF_RADIO->TASKS_TXEN;
    uint32_t radio_tasks_rxen_addr          = (uint32_t)&NRF_RADIO->TASKS_RXEN;
    uint32_t radio_tasks_start_addr         = (uint32_t)&NRF_RADIO->TASKS_START;

    // set channels
    // enable in gi1 in gi2 and gi9(at the start of UART Tx) || dissable in gi2 in gi3 and gi10
    NRF_PPI->CH[channel_gie0_or_gie1_or_gie2].EEP        = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_gie0_or_gie1_or_gie2].TEP        = timer0_task_stop_addr;
    NRF_PPI->FORK[channel_gie0_or_gie1_or_gie2].TEP      = timer0_task_capture1_addr; 

    // enable in gi0 || dissable in gi1
    NRF_PPI->CH[channel_gi0].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_gi0].TEP         = timer0_task_clear_addr;

    //enable in gi1 || dissable in gi2 
    NRF_PPI->CH[channel_gi1].EEP         = radio_events_address_addr;
    NRF_PPI->CH[channel_gi1].TEP         = timer0_task_clear_addr;

    //enable in gi2 || dissable in gi3
    NRF_PPI->CH[channel_gi2].EEP         = radio_events_end_addr;
    NRF_PPI->CH[channel_gi2].TEP         = timer0_task_clear_addr;

    // enable in gi3 || dissable in gi4
    NRF_PPI->CH[channel_gi3].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_gi3].TEP         = timer0_task_clear_addr;

    // enable in gi4 || dissable in gi5
    NRF_PPI->CH[channel_gi4].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_gi4].TEP         = timer0_task_clear_addr;
    NRF_PPI->FORK[channel_gi4].TEP       = radio_tasks_rxen_addr;

    // enable in gi5 || dissable in gi8
    NRF_PPI->CH[channel_gi5].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_gi5].TEP         = timer0_task_clear_addr;

    //enable in gi6 || dissable in gi7 
    NRF_PPI->CH[channel_gi6].EEP         = radio_events_end_addr;
    NRF_PPI->CH[channel_gi6].TEP         = radio_tasks_start_addr;

    // enable in gi8 || dissable in gi9 
    NRF_PPI->CH[channel_gi8].EEP         = timer0_events_compare_0_addr;
    NRF_PPI->CH[channel_gi8].TEP         = uart0_task_starttx_addr;
    NRF_PPI->FORK[channel_gi8].TEP       = timer0_task_clear_addr;

    //enable in gi9 at EOF before sending last byte || dissable in gi10 
    NRF_PPI->CH[channel_gi9].EEP         = uart0_events_txdrdy_addr;
    NRF_PPI->CH[channel_gi9].TEP         = uart0_task_stoptx_addr;
    NRF_PPI->FORK[channel_gi9].TEP       = timer0_task_clear_addr;
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
void changeState(gateway_state_t newstate) {
    // update the state
    gateway_vars.state = newstate;

    // wiggle the FSM debug pin
    switch (gateway_vars.state) {
        case S_UARTRXENABLE:
        case S_UARTRXDATA:
        case S_UARTRXDONE:
        case S_RADIOTXPREPARE:
        case S_RADIOTXREADY:
        case S_RADIOTX:
        case S_SLEEP:
        case S_RADIORXREADY:
        case S_RADIORXLISTEN:
        case S_RADIORX:
        case S_RADIORXPREPARE:
        case S_UARTTXPREPARE:
        case S_UARTTXREADY:
        case S_UARTTXDATA:
        case S_UARTTXDONE:
            // toggle the pins
            toggle_fsm_debug_pin();
            break;
    }
}

/**
 * @brief This function initializes the variables with default values for initializing gateway
 */ 
void gateway_init_setup(void) {
    // DEFAULTS

    // input
    gateway_vars.hdlcBusyReceiving = false;
    gateway_vars.hdlcInputEscaping = false;
    gateway_vars.inputBufFillLevel = 0;

    // ouput
    gateway_vars.outputBufIdxR = 0;
    gateway_vars.outputBufIdxW = 0;
    //gateway_vars.fBusyFlushing = false;

    // default initial state for the RRDV ROBOT FSM
    gateway_vars.state = S_UARTRXENABLE;
    // set CC[0] to a default big value which should never be reached
    NRF_TIMER0->CC[0] = defaultCmpValue;
    // start the timer
    NRF_TIMER0->TASKS_START = (TIMER_TASKS_START_TASKS_START_Trigger << TIMER_TASKS_START_TASKS_START_Pos);
    // start listening to new HDLC frame
    NRF_UART0->TASKS_STARTRX = 1UL;
}

/**
 * @brief This function sets the debug pins for ISR and FSM
 */ 
void set_debug_pins(void) {
    // For debug FSM Set the test pin (P0.31) as an output
    NRF_P0->PIN_CNF[FSM_DEBUG_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |  // Set Pin as output
                                     (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos); // Activate high current gpio mode.

    // For debug ISR Set the test pin (P0.30) as an output
    NRF_P0->PIN_CNF[ISR_DEBUG_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |  // Set Pin as output
                                     (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos); // Activate high current gpio mode.
}

/**
 * @brief This function toggles the FSM pin
 */ 
void toggle_fsm_debug_pin(void) {
    // Toggle
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
