// Broadcaster

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "radio_config.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "nrf_drv_gpiote.h" 
#include "nrf_nvic.h" 
#include "app_uart.h" 
#include "nrf_delay.h"
#include "app_button.h"
#include "nrf_drv_timer.h"
#include "nrf.h"



const  nrf_drv_timer_t PACKET_TIMER = NRF_DRV_TIMER_INSTANCE(1);          /* Timer 1 instance */
const  nrf_drv_timer_t TIMESTAMP_TIMER = NRF_DRV_TIMER_INSTANCE(2);       /* Timer 2 instance */

static uint8_t volatile le_min_packet_tx[PDU_LENGTH_LE_MIN];             /*  packet buffer for minimum packet data length */
static uint8_t volatile le_adv_packet_tx[PDU_LENGTH_LE_ADV];             /*  packet buffer for le advetrising packet */
static uint8_t volatile le_data_packet_tx[PDU_LENGTH_LE_DATA];           /*  packet buffer for le data packet */
static uint8_t volatile trigger_packet_rx[PDU_LENGTH_LE_ADV];            /* Received trigger packet buffer */

uint8_t B1_addr[LE_ADDR_LENGTH]={0x11,0x11,0x11,0x11,0x11,0x11};  /* Broadcaster 1 device address */
uint8_t B2_addr[LE_ADDR_LENGTH]={0x22,0x22,0x22,0x22,0x22,0x22};  /* Broadcaster 1 device address */

//General structure of BLE packet on NRF52_SERIES, Size of 40 Bytes--> 37 PACKET_PAYLOAD_MAXSIZE + 3 bytes header

	
// Local variables//
uint32_t p_counter=0;
uint8_t real_time_seconds;
uint8_t real_time_minutes;
uint8_t real_time_hours;
uint32_t current_time;
uint32_t current_tick;
bool time_set = false;
bool tp_received = false;		 
uint8_t rssi_result;
int8_t batch_counter;
uint8_t PACKET_CONFIG;
uint8_t BLE_MODE;
uint8_t TX_POW;
uint8_t CHANNEL;
uint8_t scenario;
bool p_timer_set=false;	 

uint32_t ADV_INTERVAL = 2;          /* adv interval in ms */
uint32_t PACKET_COUNT = 250; 
	 

/* Function for handling UART errors */
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/* Powering up radio  */
void radio_power_up()
{
/* Power up and reset all states in the radio peripheral */
	NRF_RADIO->POWER = 1;
	NRF_RADIO->EVENTS_DISABLED = 0; 
 	
}

/* Configuring CRC  */
void radio_CRC_init()
{
 /* CRC config */
  NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) | 
                       (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); /* Skip Address when computing crc */    
  NRF_RADIO->CRCINIT = 0x555555;    // Initial value of CRC
  NRF_RADIO->CRCPOLY = 0x00065B;    // CRC polynomial function
	 	
}


/* Configuring Access Address  */
void radio_addr_init()
{	
  /* Configure Access Address to be the BLE standard */
  NRF_RADIO->BASE0 		 = 0x89BED600;
  NRF_RADIO->PREFIX0	 = 0x0000008E;	
  NRF_RADIO->TXADDRESS = 0x00UL;					/* Use logical address 0 (PREFIX0 + BASE0) = 0x8E89BED6 when transmitting */
	NRF_RADIO->RXADDRESSES = 0x01UL;				/* Enable reception on logical address 0 (PREFIX0 + BASE0) = = 0x8E89BED6 when receiving */
	
}

/* Setting interframe spacing */
void radio_tifs_set(void)
{	
	NRF_RADIO->TIFS = 150; 	
}

/* Setting packet PDU header configurations */
void radio_set_packet_header_configuration(uint8_t p_con)
{ 
	switch (p_con)
   {
					
		case PACKET_CONFIG_MIN: 
		
         le_min_packet_tx[0]= PDU_HEADER_S0_VALUE;
         le_min_packet_tx[1]= PACKET_PAYLOAD_MAXSIZE_LE_MIN;
				 break;
						
		case PACKET_CONFIG_ADV:
         		
			   le_adv_packet_tx[0]= PDU_HEADER_S0_VALUE;
         le_adv_packet_tx[1]= PACKET_PAYLOAD_MAXSIZE_LE_ADV;  					
         break;								 
                         				
		case PACKET_CONFIG_DATA:
         	
			   le_data_packet_tx[0]= PDU_HEADER_S0_VALUE;
         le_data_packet_tx[1]= PACKET_PAYLOAD_MAXSIZE_LE_DATA;
         break;					                 
        		    	     
  } 

}

/* setting radio packet PCNF1 register configurations */
void radio_set_pcnf1_configuration(void)
{ 

 NRF_RADIO->PCNF1 = (  (PACKET_PAYLOAD_MAXSIZE_LE_DATA  << RADIO_PCNF1_MAXLEN_Pos)     /* maximum length of payload for le adv packet (37 bytes). */
                     | (PACKET_STATIC_LENGTH            << RADIO_PCNF1_STATLEN_Pos) 	/* Static length of the packet  */
                     | (PACKET_BASE_ADDRESS_LENGTH      << RADIO_PCNF1_BALEN_Pos)            /* base address length in number of bytes. */
                     | (RADIO_PCNF1_ENDIAN_Little       << RADIO_PCNF1_ENDIAN_Pos)          /* endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
                     | (RADIO_PCNF1_WHITEEN_Enabled     << RADIO_PCNF1_WHITEEN_Pos) 	      /* enable packet whitening. */
                    );					        				                       		    	  

}

/* Setting radio packet PCNF0 register configurations */
void radio_set_pcnf0_configuration(uint8_t ble_mode)
{ 
	NRF_RADIO->PCNF0   = (PACKET_S0_FIELD_LENGTH     << RADIO_PCNF0_S0LEN_Pos) |  // length of S0 field in bytes. 
	                     (PACKET_S1_FIELD_LENGTH     << RADIO_PCNF0_S1LEN_Pos) |  // length of S1 field in bits.                      
                       (PACKET_LENGTH_FIELD_LENGTH << RADIO_PCNF0_LFLEN_Pos);   // length of length field in bits. 

 switch (ble_mode)
   {
		 case BLE_1MBIT: 
			    NRF_RADIO->PCNF0 |= (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos); // length of preamble field in bits.
				  break;
		 
		 case BLE_2MBIT: 
			    NRF_RADIO->PCNF0 |= (RADIO_PCNF0_PLEN_16bit << RADIO_PCNF0_PLEN_Pos); // length of preamble field in bits.
				  break;
		 
		 case BLE_LR500KBIT:
		 case BLE_LR125KBIT:
          NRF_RADIO->PCNF0 |= (PACKET_CI_FIELD_LENGTH    << RADIO_PCNF0_CILEN_Pos)   |  // length of CI field in bits.
	                            (PACKET_TERM_FIELD_LENGTH  << RADIO_PCNF0_TERMLEN_Pos) |  // length of TERM field in bits.
	                            (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos);		   // length of preamble field in bits.	 		                     
				  break;
		 
	 }		 				        				                       		    	 
}

/* Setting scan timer interval */
void radio_ble_set_mode(uint8_t mode)
{
	radio_set_pcnf0_configuration(mode);
	
	switch (mode)
   {					
		case BLE_1MBIT: 
         NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);			
				 break;
						
		case BLE_2MBIT: 
         NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos);			
         break;								 
                         				
		case BLE_LR500KBIT: 
         NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_LR500Kbit << RADIO_MODE_MODE_Pos); 			
         break;

    case BLE_LR125KBIT: 
			   NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);   
         break;		
  }	
}


/* Function for setting the radio transmitter power */
void radio_set_tx_power(uint8_t tx_power)
{ 

 switch (tx_power)
   {
					
		case 1: NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
				    break;
						
		case 2: NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos); 			
            break;								 
                         				
		case 3: NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Neg8dBm << RADIO_TXPOWER_TXPOWER_Pos);    
            break;

    case 4: NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Neg20dBm << RADIO_TXPOWER_TXPOWER_Pos);    
            break;		
	 } 
}


/* Function for setting the radio channels  */
 void radio_ble_ch_set(uint8_t ch)
{
	 switch (ch)
   {
		
		 case 37: NRF_RADIO->FREQUENCY = 2U;
		          NRF_RADIO->DATAWHITEIV = ch;   // 37
				      break;
		 
		 case 38: NRF_RADIO->FREQUENCY = 26U;
		          NRF_RADIO->DATAWHITEIV = ch;   // 38
				      break;		 
		 
     case 39: NRF_RADIO->FREQUENCY = 80U;
		          NRF_RADIO->DATAWHITEIV = ch;   // 39
				      break;
		 
		 case 4:  NRF_RADIO->FREQUENCY = 12U;
		          NRF_RADIO->DATAWHITEIV = ch;   // 4
				      break;
						
		 case 15: NRF_RADIO->FREQUENCY = 36U;
              NRF_RADIO->DATAWHITEIV = ch;	  // 15	 
              break;								 
                         				
		 case 28: NRF_RADIO->FREQUENCY = 62U;
              NRF_RADIO->DATAWHITEIV = ch;	  // 28	 
              break;
	
	 }              	  
}


/* Function for reading the rssi of the packet */
 void radio_rssi_read()
{
	NRF_RADIO->TASKS_RSSISTART = 1;
  while(!NRF_RADIO->EVENTS_RSSIEND);
  NRF_RADIO->EVENTS_RSSIEND = 0;
  rssi_result = NRF_RADIO->RSSISAMPLE;
  NRF_RADIO->TASKS_RSSISTOP = 1;
}

/* Function to set radio transmitted packet  */
void radio_ble_set_packetptr(uint8_t p_ptr)
{	
	switch (p_ptr)
   {		
    case 0: NRF_RADIO->PACKETPTR = (uint32_t)trigger_packet_rx;   
            break;
		 
		case 1: NRF_RADIO->PACKETPTR = (uint32_t)le_adv_packet_tx;                
				    break;
		 
		case 2: NRF_RADIO->PACKETPTR = (uint32_t)le_min_packet_tx;    			      
            break;								 
                         				
		case 3: NRF_RADIO->PACKETPTR = (uint32_t)le_data_packet_tx;      
            break;
	
  }	
}


/* Function for setting the broadcaster address	*/
void set_device_address(uint8_t idx)
{ 	
	 if (idx == 1)
		 {							 
			  for (int i=0; i< LE_ADDR_LENGTH; i++)
			   {
				   le_min_packet_tx[i+PDU_HEADER_LENGTH]= B1_addr[i];
					 le_adv_packet_tx[i+PDU_HEADER_LENGTH]= B1_addr[i];
					 le_data_packet_tx[i+PDU_HEADER_LENGTH]= B1_addr[i];
			   }
		 }
		 
		 if (idx == 2)
		 {
			 for (int i=0; i< LE_ADDR_LENGTH; i++)
			   {
				   le_min_packet_tx[i+PDU_HEADER_LENGTH]= B2_addr[i];
					 le_adv_packet_tx[i+PDU_HEADER_LENGTH]= B2_addr[i];
					 le_data_packet_tx[i+PDU_HEADER_LENGTH]= B2_addr[i];
			   }			
		 }	
}


/* Function for setting the advertising data	*/
void set_adv_data(void)
{	 
	  // set advertising data starting from index 9 since index 8 is reserved for the packet counter
	  for (int i=0; i< PACKET_PAYLOAD_MAXSIZE_LE_ADV-LE_ADDR_LENGTH-1; i++)   
		 {
			 le_adv_packet_tx[i+PDU_HEADER_LENGTH+LE_ADDR_LENGTH+1]= i;   
			 nrf_delay_us(100);
		 }
	
    for (int i=0; i< PACKET_PAYLOAD_MAXSIZE_LE_DATA-LE_ADDR_LENGTH-1; i++)
		 {
			 le_data_packet_tx[i+PDU_HEADER_LENGTH+LE_ADDR_LENGTH+1]= i;
			 nrf_delay_us(100);
	   }

}


/* Function for sending the packets	*/
void send_packet()
{
    // initiate the radio as transmitter	
    NRF_RADIO->EVENTS_READY = 0U;
    NRF_RADIO->TASKS_TXEN   = 1;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END  = 0U;
    NRF_RADIO->TASKS_START = 1U;

    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
		
}

/* Function for disabling the radio	*/
void radio_disable()
{
    NRF_RADIO->TASKS_DISABLE = 1U;
    while (NRF_RADIO->EVENTS_DISABLED == 0U) {/* wait*/}
}

/* Initializing default mode for radio	*/
void radio_default_init()
{

	radio_ble_set_mode(BLE_1MBIT);
	radio_set_pcnf1_configuration();
  radio_set_tx_power(TX_PWR_0dBm);	
	radio_set_packet_header_configuration(PACKET_CONFIG_ADV);
  radio_ble_ch_set(37);
  set_device_address(B1_ADDR);
  set_adv_data();
  radio_ble_set_packetptr(PACKET_CONFIG_TRIGGER);
  	
}


/* Function for initializing the clock	*/
void clock_initialization()
{
    /* Start the 32 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}

/* Function for button presses	*/
void button_handler(nrf_drv_gpiote_pin_t pin_no, nrf_gpiote_polarity_t button_action)
{    
   switch (pin_no)
    {
      case BSP_BUTTON_0:      // transmit 250 packets on channel 37
							   
					 radio_disable();
			     tp_received = true;
			     PACKET_CONFIG=PACKET_CONFIG_ADV;
			     radio_ble_ch_set(37);
			     radio_ble_set_packetptr(PACKET_CONFIG_ADV);
           nrf_delay_ms(500);				 
		       nrf_drv_timer_enable(&PACKET_TIMER);						
					 nrf_gpio_pin_clear(LED_1_PIN);			
					 break;
						
		  case BSP_BUTTON_1:      // transmit 250 packets on channel 38 
								
           radio_disable();
			     tp_received = true;
			     radio_ble_ch_set(38);
			     PACKET_CONFIG=PACKET_CONFIG_ADV;
			     radio_ble_set_packetptr(PACKET_CONFIG_ADV);
           nrf_delay_ms(500);				 
		       nrf_drv_timer_enable(&PACKET_TIMER);						
					 nrf_gpio_pin_clear(LED_2_PIN);			     					
           break;								 
                         				
		  case BSP_BUTTON_2:      // transmit 250 packets on channel 39 
							   						     
					 radio_disable();
			     tp_received = true;
			     radio_ble_ch_set(39);
			     PACKET_CONFIG=PACKET_CONFIG_ADV;
			     radio_ble_set_packetptr(PACKET_CONFIG_ADV);
           nrf_delay_ms(500);				 
		       nrf_drv_timer_enable(&PACKET_TIMER);						
					 nrf_gpio_pin_clear(LED_3_PIN);
           break;					                 
        
		  case BSP_BUTTON_3:     // disable transmitting

           nrf_drv_timer_disable(&PACKET_TIMER);
           nrf_gpio_pin_set(LED_4_PIN);					 																    						    
           break;       
   }  
}

/* Function for handling packet timer interrupts	*/
void Packet_TIMER_event_handler(nrf_timer_event_t event_type, void* p_context)
{    
  switch (event_type)
   {
     case NRF_TIMER_EVENT_COMPARE0:             // this event is triggered when the timer fires
					
          if( PACKET_CONFIG==1){ le_adv_packet_tx[8] = p_counter;}
					if( PACKET_CONFIG==2){ le_min_packet_tx[8] = p_counter;}
					if( PACKET_CONFIG==3){ le_data_packet_tx[8]= p_counter;}
				  nrf_gpio_pin_toggle(LED_4_PIN);        // toggle the LED
				  nrf_gpio_pin_clear(LED_1_PIN);
          send_packet();				    				       // send the packet
				  p_counter++;                             // increment packet counter
				  if ( p_counter == PACKET_COUNT+1 )       // if we reached 250 packets, stop
						{
							nrf_drv_timer_disable(&PACKET_TIMER);	
              nrf_gpio_pin_set(LED_1_PIN);							
							p_counter=0;
							radio_disable();
						  radio_default_init();
							tp_received = false;					
						}				
         break;
					 													
        default:
            
            break;
    }
}

/* Function for handling timestamp timer interrupts	*/
void TIMESTAMP_TIMER_event_handler(nrf_timer_event_t event_type, void* p_context)
{    
  switch (event_type)
   {        				 
		 case NRF_TIMER_EVENT_COMPARE0:
						 					 
          nrf_drv_timer_enable(&PACKET_TIMER);
				  nrf_gpio_pin_clear(LED_1_PIN);
					 
          real_time_seconds++; 				       
					if( real_time_seconds == 60)
					 {
						real_time_minutes++;
						 real_time_seconds=0;
					 }

          if( real_time_minutes == 60)
					 {
						 real_time_hours++;
						 real_time_minutes=0;
					 }
					nrf_gpio_pin_toggle(LED_3_PIN);									
          break;
					 
    default:            
      break;
  }
}

/* Setting packet (transmission) timer interval */
void packet_timer_set_interval(uint32_t interval)
{
	uint32_t PACKET_TIMER_ms = interval; //Time (in miliseconds) 
  uint32_t PACKET_TIMER_ticks;
	
	PACKET_TIMER_ticks  = nrf_drv_timer_ms_to_ticks(&PACKET_TIMER, PACKET_TIMER_ms);
	nrf_drv_timer_extended_compare(&PACKET_TIMER, NRF_TIMER_CC_CHANNEL0, PACKET_TIMER_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	
}


/* Function for configuring GPIO	*/
static void gpio_config(void) 	
{
    ret_code_t err_code;

    // Initialze driver.
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // Configure 4 output pins for LED's.
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrf_drv_gpiote_out_init(LED_1_PIN, &out_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(LED_2_PIN, &out_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(LED_3_PIN, &out_config);
    APP_ERROR_CHECK(err_code);
	  err_code = nrf_drv_gpiote_out_init(LED_4_PIN, &out_config);
    APP_ERROR_CHECK(err_code);
		  	
    // Set output pins (this will turn off the LED's).
    nrf_drv_gpiote_out_set(LED_1_PIN);
    nrf_drv_gpiote_out_set(LED_2_PIN);
    nrf_drv_gpiote_out_set(LED_3_PIN);
		nrf_drv_gpiote_out_set(LED_4_PIN); 
		

    // Make a configuration for input pins. 
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    // Configure input pins for buttons, with separate event handlers for each button.
    err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_0, &in_config, button_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_1, &in_config, button_handler);
    APP_ERROR_CHECK(err_code);
		err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_2, &in_config, button_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_3, &in_config, button_handler);
    APP_ERROR_CHECK(err_code); 
																					 														 
   //  Enable input pins for buttons.
    nrf_drv_gpiote_in_event_enable(BSP_BUTTON_0, true);
    nrf_drv_gpiote_in_event_enable(BSP_BUTTON_1, true);
		nrf_drv_gpiote_in_event_enable(BSP_BUTTON_2, true);
    nrf_drv_gpiote_in_event_enable(BSP_BUTTON_3, true);
																																							
} 

/* Function for reading the received packets	*/
void read_packet()
{
  //  uint32_t result = 0;
	
    NRF_RADIO->EVENTS_READY = 0U;
    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;

    // Wait for end of packet 
    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    if (NRF_RADIO->CRCSTATUS == 1U)
    {  
        if ( trigger_packet_rx[2] == 0x33 && trigger_packet_rx[3] == 0x33 )     // if packet received from observer board, start the time stamp timer
				{
   
					BLE_MODE = trigger_packet_rx[8];
					CHANNEL = trigger_packet_rx[9];
					PACKET_CONFIG = trigger_packet_rx[10];          
          TX_POW = trigger_packet_rx[11];
          batch_counter = trigger_packet_rx[12];					
          scenario = trigger_packet_rx[13];
         	tp_received=true;					
				}			  
    }
		if (NRF_RADIO->CRCSTATUS == 0U)
     {          
		  
     }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
     {
        // wait
     }  
	
		if (tp_received)
		 {					  				 
			 radio_ble_set_mode(BLE_MODE);
       radio_set_tx_power(TX_POW);
       radio_ble_ch_set(CHANNEL);				 
	     radio_set_packet_header_configuration(PACKET_CONFIG);
       radio_ble_set_packetptr(PACKET_CONFIG);
				 
			 if( BLE_MODE==1 && scenario==1 && PACKET_CONFIG !=3 && batch_counter==1) {packet_timer_set_interval(2);}     // adv 1Mbit
			 if( BLE_MODE==1 && scenario==1 && PACKET_CONFIG ==3 && batch_counter==1) {packet_timer_set_interval(4);}
					
			 if( BLE_MODE==1 && scenario==2 && PACKET_CONFIG !=3 && batch_counter==1) {packet_timer_set_interval(2);}    // data 1Mbit
			 if( BLE_MODE==1 && scenario==2 && PACKET_CONFIG ==3 && batch_counter==1) {packet_timer_set_interval(4);}
					
			 if( BLE_MODE==2 && scenario==2 && PACKET_CONFIG !=3 && batch_counter==1) {packet_timer_set_interval(2);}    // data 2Mbit
			 if( BLE_MODE==2 && scenario==2 && PACKET_CONFIG ==3 && batch_counter==1) {packet_timer_set_interval(4);}
					
			 if( BLE_MODE==3 && scenario==2 && PACKET_CONFIG == 1 && batch_counter==1) {packet_timer_set_interval(3);}    // data 500Kbit
			 if( BLE_MODE==3 && scenario==2 && PACKET_CONFIG == 2 && batch_counter==1) {packet_timer_set_interval(2);}
			 if( BLE_MODE==3 && scenario==2 && PACKET_CONFIG == 3 && batch_counter==1) {packet_timer_set_interval(6);}
					
			 if( BLE_MODE==4 && scenario==2 && PACKET_CONFIG == 1 && batch_counter==1) {packet_timer_set_interval(5);}    // data 500Kbit
			 if( BLE_MODE==4 && scenario==2 && PACKET_CONFIG == 2 && batch_counter==1) {packet_timer_set_interval(3);}
			 if( BLE_MODE==4 && scenario==2 && PACKET_CONFIG == 3 && batch_counter==1) {packet_timer_set_interval(19);}
          										
       nrf_delay_ms(5);        
		   nrf_drv_timer_enable(&PACKET_TIMER);
       trigger_packet_rx[2]=0;
       trigger_packet_rx[3]=0;				 
     }
}

/* Function to initialize the UART interface */
 void UART_initialize(void)
{
     uint32_t err_code;
	
		// Application UART interface parameters
	const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };
  // Initialize the UART interface
    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
	  if (err_code != NRF_SUCCESS){nrf_gpio_pin_clear(LED_1_PIN);}
					 
}


/* main */
int main(void)
{
	  // Timers parameters
	  uint32_t PACKET_TIMER_ms = ADV_INTERVAL; //Time (in miliseconds) 
    uint32_t PACKET_TIMER_ticks;	
	  uint32_t TIMESTAMP_TIMER_ms = 1000;
	  uint32_t TIMESTAMP_TIMER_ticks;
    uint32_t err_code = NRF_SUCCESS;

	  // ble initialization
    clock_initialization();
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	  
	  radio_power_up(); 
	  radio_addr_init();	
	  radio_CRC_init();
		radio_tifs_set();
    radio_default_init();	
    gpio_config();
    UART_initialize();		
		
		// timers initialization
		nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
		uint32_t TIMER_err_code = NRF_SUCCESS;
    TIMER_err_code = nrf_drv_timer_init(&PACKET_TIMER, &timer_cfg, Packet_TIMER_event_handler);
		TIMER_err_code = nrf_drv_timer_init(&TIMESTAMP_TIMER, &timer_cfg, TIMESTAMP_TIMER_event_handler);
    APP_ERROR_CHECK(TIMER_err_code); 
		
    PACKET_TIMER_ticks  = nrf_drv_timer_ms_to_ticks(&PACKET_TIMER, PACKET_TIMER_ms);
    TIMESTAMP_TIMER_ticks    = nrf_drv_timer_ms_to_ticks(&PACKET_TIMER, TIMESTAMP_TIMER_ms);
		
		nrf_drv_timer_extended_compare(&PACKET_TIMER, NRF_TIMER_CC_CHANNEL0, PACKET_TIMER_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
		nrf_drv_timer_extended_compare(&TIMESTAMP_TIMER, NRF_TIMER_CC_CHANNEL0, TIMESTAMP_TIMER_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
			
    while (true)
    {
       if (!tp_received) {read_packet();}       // keep scanning until trigger packet is received  
    }
}

