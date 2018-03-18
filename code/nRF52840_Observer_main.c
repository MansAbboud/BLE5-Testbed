// Observer


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "radio_config.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "app_error.h"
#include "nrf_drv_gpiote.h" 
#include "nrf_nvic.h" 
#include "app_uart.h" 
#include "nrf_delay.h"
#include "app_button.h"
#include "nrf_drv_timer.h"
#include "nrf.h"




const  nrf_drv_timer_t M_STOP_TIMER = NRF_DRV_TIMER_INSTANCE(0);  /* Timer 1 instance */
const  nrf_drv_timer_t MEASUREMENT_TIMER = NRF_DRV_TIMER_INSTANCE(1);  /* Timer 1 instance */
const  nrf_drv_timer_t TIMESTAMP_TIMER = NRF_DRV_TIMER_INSTANCE(2);    // Timer 2 instance  */

static uint8_t volatile le_min_packet_rx[PDU_LENGTH_LE_MIN];          /* Received packet buffer for minimum packet data length */
static uint8_t volatile le_adv_packet_rx[PDU_LENGTH_LE_ADV];           /* Received packet buffer for le advetrising packet */
static uint8_t volatile le_data_packet_rx[PDU_LENGTH_LE_DATA];         /* Received packet buffer for le data packet */
static uint8_t volatile trigger_packet_tx[PDU_LENGTH_LE_ADV];          /* Received trigger packet buffer */

uint8_t OB_addr[LE_ADDR_LENGTH]={0x33,0x33,0x33,0x33,0x33,0x33};       /* Observer device address  */

// Local variables//

uint32_t PACKET_COUNT = 250;
uint32_t SCAN_INTERVAL = 600;
uint32_t printing_delay = 400;
uint32_t waiting_delay = 300;


uint8_t ble_addr[6];	
uint16_t B1_packet_counter;
uint16_t B2_packet_counter;
uint8_t real_time_seconds;
uint8_t real_time_minutes;
uint8_t real_time_hours;
uint32_t current_time;
uint32_t current_tick;
bool time_set = false;
bool tp_sent = false;
bool measuring = false; 
bool B1_rec_ind = false; 
bool broadcaster_missed = true;
uint8_t B1_rec_packets_rssi[504];
uint8_t controlArray[6];
uint8_t uart_rx_buffer[502];
uint16_t uart_rx_buffer_idx;
uint32_t array_counter=0;
uint8_t rssi_counter=1;
uint8_t uart_idx=1;
uint8_t recValue;
uint8_t batch_counter=1;
uint8_t idx_counter=1;
uint8_t data_idx_counter=1;
uint8_t rssi_result;
uint8_t PER;
uint8_t PACKET_CONFIG;
uint8_t BLE_MODE;
uint8_t TX_POW;
uint8_t CHANNEL;
uint8_t scenario;



char *radio__mode[]={"","1Mbit", "2Mbit", "500Kbit S=2", "125Kbit S=8"};
char *packet__config[]={"","Avg (Adv), 31 Bytes", "Min, 1 Byte", "Max, 255 Bytes" };
char *tx__power[]={"", "+8dBm", "0dBm", "-8dBm", "-20dBm"};    


/* aFunction for handling UART errors */
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
void radio_power_up(void)
{
/* Power up and reset all states in the radio peripheral */
	NRF_RADIO->POWER = 1;
	NRF_RADIO->EVENTS_DISABLED = 0; 
 	
}

/* Configuring CRC  */
void radio_CRC_init(void)
{
 /* CRC config */
  NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) | 
                       (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); /* Skip Address when computing crc */    
  NRF_RADIO->CRCINIT = 0x555555;    // Initial value of CRC
  NRF_RADIO->CRCPOLY = 0x00065B;    // CRC polynomial function
 	
}

/* Setting interframe spacing */
void radio_tifs_set(void)
{	
	NRF_RADIO->TIFS = 150; 	
}

/* Configuring Access Address  */
void radio_addr_init(void)
{
	
	/* Configure Access Address to be the BLE standard */
	NRF_RADIO->BASE0 		 = 0x89BED600;
  NRF_RADIO->PREFIX0	 = 0x0000008E; 
  NRF_RADIO->TXADDRESS = 0x00;					/* Use logical address 0 (PREFIX0 + BASE0) = 0x8E89BED6 when transmitting */
	NRF_RADIO->RXADDRESSES = 0x01;				/* Enable reception on logical address 0 (PREFIX0 + BASE0) = = 0x8E89BED6 when receiving */
	
}

/* Setting radio packet PCNF0 register configurations and radio mode */
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

/* Setting radio packet PCNF1 register configurations */
void radio_set_pcnf1_configuration(void)
{ 

 NRF_RADIO->PCNF1 = (  (PACKET_PAYLOAD_MAXSIZE_LE_DATA  << RADIO_PCNF1_MAXLEN_Pos)     /* maximum length of payload for le adv packet (37 bytes). */
                     | (PACKET_STATIC_LENGTH            << RADIO_PCNF1_STATLEN_Pos) 	/* Static length of the packet  */
                     | (PACKET_BASE_ADDRESS_LENGTH      << RADIO_PCNF1_BALEN_Pos)            /* base address length in number of bytes. */
                     | (RADIO_PCNF1_ENDIAN_Little       << RADIO_PCNF1_ENDIAN_Pos)          /* endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
                     | (RADIO_PCNF1_WHITEEN_Enabled     << RADIO_PCNF1_WHITEEN_Pos) 	      /* enable packet whitening. */
                    );					        				                       		    	  

}

/* Function for setting radio transmitter power */
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

/* Function for setting the radio channels ( LE adv channels only ) */
 void radio_ble_ch_set(uint32_t channel)
{
	 switch (channel)
   {		
		 case 37: NRF_RADIO->FREQUENCY = 2U;
		          NRF_RADIO->DATAWHITEIV = channel;   // 37
				      break;
		 
		 case 38: NRF_RADIO->FREQUENCY = 26U;
		          NRF_RADIO->DATAWHITEIV = channel;   // 38
				      break;		 
		 
     case 39: NRF_RADIO->FREQUENCY = 80U;
		          NRF_RADIO->DATAWHITEIV = channel;   // 39
				      break;
		 
		 case 4:  NRF_RADIO->FREQUENCY = 12U;
		          NRF_RADIO->DATAWHITEIV = channel;   // 4
				      break;
						
		 case 15: NRF_RADIO->FREQUENCY = 36U;
              NRF_RADIO->DATAWHITEIV = channel;	  // 15	 
              break;								 
                         				
		 case 28: NRF_RADIO->FREQUENCY = 62U;
              NRF_RADIO->DATAWHITEIV = channel;	  // 28	 
              break;
	
	 }              	  
}

/* Setting scan timer interval */
void measurement_timer_set_interval(uint32_t interval)
{
	uint32_t MEASUREMENT_TIMER_ms = interval; //Time (in miliseconds) 
  uint32_t MEASUREMENT_TIMER_ticks;
	
	MEASUREMENT_TIMER_ticks  = nrf_drv_timer_ms_to_ticks(&MEASUREMENT_TIMER, MEASUREMENT_TIMER_ms);
	nrf_drv_timer_extended_compare(&MEASUREMENT_TIMER, NRF_TIMER_CC_CHANNEL0, MEASUREMENT_TIMER_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	
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

/* Function for setting the observer address	*/
void set_device_address()
{ 		 
		for (int i=0; i< LE_ADDR_LENGTH; i++)
		 {
			trigger_packet_tx[i+PDU_HEADER_LENGTH]= OB_addr[i];
		 }		   						
}

/* Setting radio mode */
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

/* Function to set radio transmitted packet  */
void radio_ble_set_packetptr(uint8_t p_ptr)
{
	
	switch (p_ptr)
   {
		
    case PACKET_CONFIG_TRIGGER: 
			   NRF_RADIO->PACKETPTR = (uint32_t)trigger_packet_tx;   
         break;
		 
		case PACKET_CONFIG_ADV: 
			   NRF_RADIO->PACKETPTR = (uint32_t)le_adv_packet_rx;               
				 break;
		 
		case PACKET_CONFIG_MIN: 
			   NRF_RADIO->PACKETPTR = (uint32_t)le_min_packet_rx;    			      
         break;								 
                         				
		case PACKET_CONFIG_DATA: 
			   NRF_RADIO->PACKETPTR = (uint32_t)le_data_packet_rx; 	      
         break;
   				
  }	
}


/* Function for setting the advertising data	*/
void set_adv_data()
{
	  trigger_packet_tx[8] = BLE_MODE;          // 1: BLE 1Mbit, 2: BLE 2Mbit, 3: BLE LR 500Kbit, 4: BLE LR 125Kbit
    trigger_packet_tx[9] = CHANNEL;           // 1: MIN, 2: ADV, 3: DATA(max)
    trigger_packet_tx[10]= PACKET_CONFIG;     // 1: 0dBm, 2: -8dBm, 3: -20dBm  
    trigger_packet_tx[11]= TX_POW;	          // 1: 37, 2: 38, 3: 39
    trigger_packet_tx[12]= batch_counter;
	  trigger_packet_tx[13]= scenario;
}


/* Function for preparing the trigger packet	*/
void prepare_trigger_packet()
{ 	
	  trigger_packet_tx[0]= PDU_HEADER_S0_VALUE;
    trigger_packet_tx[1]= PACKET_PAYLOAD_MAXSIZE_LE_ADV;
 
}


// Function to send control data through UART
static void UART_send_control_data(void)     
{			
   controlArray[0]=254;
	 controlArray[1]=BLE_MODE;
	 controlArray[2]=CHANNEL;
	 controlArray[3]=PACKET_CONFIG;
	 controlArray[4]=TX_POW;
	 controlArray[5]=batch_counter;
	
	 for (int i = 0; i < 6; i++)                                  // loop on all elements of the array, 201 bytes      
	    {        								
        while(app_uart_put(controlArray[i]) != NRF_SUCCESS);				 // send data through UART				
			}										 					 
}


/* Function for sending timestamp trigger packet	*/
void send_trigger_packet()
{
    // initiate the radio as transmitter
    NRF_RADIO->EVENTS_READY = 0U;
    NRF_RADIO->TASKS_TXEN   = 1;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
		// send the packet
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
		NRF_RADIO->EVENTS_DISABLED = 0;
}

/* Initialize default state for radio	*/
void radio_default_init()
{
		
  BLE_MODE = BLE_1MBIT;
  CHANNEL = 37;
  PACKET_CONFIG = PACKET_CONFIG_ADV;    
  TX_POW = TX_PWR_Pos8dBm;                             
	radio_ble_set_mode(BLE_1MBIT);	
	radio_set_pcnf1_configuration();
  radio_set_tx_power(TX_PWR_Pos8dBm);	
  radio_ble_ch_set(37);
  prepare_trigger_packet();
  set_device_address();	
  	
}

/* Initialize radio for sending trigger packet	*/
void radio_trigger_state_init(void)
{
 // radio always send trigger packet on channel 37 with 1Mbit mode
 radio_ble_set_mode(BLE_1MBIT);
 radio_ble_set_packetptr(PACKET_CONFIG_TRIGGER); // Set payload pointer to trigger packet array
 radio_ble_ch_set(37);
 set_adv_data();  	
}

/* Initialize radio 1Mbit mode	*/
void radio_ble_1Mbit_scenario_start()
{
 radio_disable();
 tp_sent=false;
 nrf_delay_ms(1);
 BLE_MODE = BLE_1MBIT;
 radio_trigger_state_init();
 nrf_delay_ms(1);	
 send_trigger_packet();
 UART_send_control_data();
 radio_ble_set_packetptr(PACKET_CONFIG_DATA);   	
 radio_ble_ch_set(CHANNEL);	
 tp_sent=true;
 nrf_drv_timer_enable(&M_STOP_TIMER);
 	
}

/* Initialize radio 2Mbit mode	*/
void radio_ble_2Mbit_scenario_start()
{
 radio_disable();
 tp_sent=false;
 nrf_delay_ms(1);
 BLE_MODE = BLE_2MBIT;
 radio_trigger_state_init();
 nrf_delay_ms(1);	
 send_trigger_packet();
 radio_ble_set_mode(BLE_2MBIT);
 UART_send_control_data();	
 radio_ble_set_packetptr(PACKET_CONFIG_DATA);     // Set payload pointer to trigger packet array	
 radio_ble_ch_set(CHANNEL);	
 tp_sent=true;
 nrf_drv_timer_enable(&M_STOP_TIMER);
 	
}


/* Initialize radio FEC 500Kbit mode	*/
void radio_ble_500Kbit_scenario_start()
{
	
 radio_disable();
 tp_sent=false;
 nrf_delay_ms(1);
 BLE_MODE = BLE_LR500KBIT;
 radio_trigger_state_init();
 nrf_delay_ms(1);	
 send_trigger_packet();
 radio_ble_set_mode(BLE_LR500KBIT);
 UART_send_control_data();	
 radio_ble_set_packetptr(PACKET_CONFIG_DATA);     // Set payload pointer to trigger packet array	
 radio_ble_ch_set(CHANNEL);	
 tp_sent=true;
 nrf_drv_timer_enable(&M_STOP_TIMER);
	
}

/* Initialize radio FEC 125Kbit mode	*/
void radio_ble_125Kbit_scenario_start()
{
 radio_disable();
 tp_sent=false;
 nrf_delay_ms(1);
 BLE_MODE = BLE_LR125KBIT;
 radio_trigger_state_init();
 nrf_delay_ms(1);	
 send_trigger_packet();
 radio_ble_set_mode(BLE_LR125KBIT);
 UART_send_control_data();	
 radio_ble_set_packetptr(PACKET_CONFIG_DATA);     	
 radio_ble_ch_set(CHANNEL);	
 tp_sent=true;
 nrf_drv_timer_enable(&M_STOP_TIMER); 	
}


/* Function for reading the RSSI of the received packet	*/
int do_env_rssi_sampling(uint32_t channel)
{
   NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos;
 
   NRF_RADIO->FREQUENCY = channel;
   NRF_RADIO->TASKS_RXEN = 1;
   while (!NRF_RADIO->EVENTS_READY);
   NRF_RADIO->EVENTS_READY = 0;

   NRF_RADIO->TASKS_RSSISTART = 1;
   while(!NRF_RADIO->EVENTS_RSSIEND);
   NRF_RADIO->EVENTS_RSSIEND = 0;

   int env_rssi_result = NRF_RADIO->RSSISAMPLE;

   NRF_RADIO->TASKS_RSSISTOP = 1;

   NRF_RADIO->TASKS_DISABLE = 1;
   while (!NRF_RADIO->EVENTS_DISABLED);
   NRF_RADIO->EVENTS_DISABLED = 0;
   return env_rssi_result;
}


/* Function for initializing the clock	*/
void clock_initialization()
{
    /* Start 32 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}


/* Function for handling timestamp timer interrupts	*/
void M_STOP_TIMER_event_handler(nrf_timer_event_t event_type, void* p_context)
{    
  switch (event_type)
   {       				 
		case NRF_TIMER_EVENT_COMPARE0:						 					           							 
					      
		     if(broadcaster_missed)     
	         {
		         nrf_drv_timer_disable(&MEASUREMENT_TIMER);               
             nrf_delay_ms(1);						 
				     radio_ble_1Mbit_scenario_start();                  		          	  	
		       } 
								
	       else {nrf_drv_timer_disable(&M_STOP_TIMER);}				  
                
        default:
            //Do nothing.
            break;
    }
}

/* Function for handling timestamp timer interrupts	*/
void TIMESTAMP_TIMER_event_handler(nrf_timer_event_t event_type, void* p_context)
{    
    switch (event_type)
    {        				 
			case NRF_TIMER_EVENT_COMPARE0:
						 					           
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
                
      default:
            //Do nothing.
       break;
    }
}

/* Function for processing advertising channels scenario	*/
void process_adv_scenario()
{
	batch_counter++;
	idx_counter++;
  PER=0;
  measuring=false;
	broadcaster_missed=true;
                        
  if( batch_counter ==5)    // if we reached 4 batches
	 {
		batch_counter=1;
		TX_POW++;								
	 }
							
	if( TX_POW == 5)         // if we reached 4 tx power    should be 5 and 4 without -20
		{
		 batch_counter=1;
		 TX_POW = 1;
 //	 PACKET_CONFIG++;
	  }
							 
	 if( idx_counter <17)    // while 16 batches not reached, 4 batches packets X 4 tx power  13 without -20
		{
     nrf_delay_ms(waiting_delay);
		 radio_ble_1Mbit_scenario_start();
		}
							 
	 if( idx_counter == 17)   // if we reached 16 batch 17, and  13 without -20
		{  																						 
		 CHANNEL++;
									 
			if( CHANNEL == 40){CHANNEL = 37; PACKET_CONFIG++;   }    // break;
		  if( PACKET_CONFIG < 3)
       {  									 
				idx_counter=1;
				nrf_delay_ms(200);
				radio_ble_1Mbit_scenario_start();
			 }
			else{PACKET_CONFIG=1;}
		}			
 	
}

/* Function for processing data channels scenario	*/
void process_data_scenario()
{	
	 batch_counter++;
	 idx_counter++;
	 data_idx_counter++;
   PER=0;
   measuring=false;
                                      
		if( PACKET_CONFIG < 4)
       { 		

         if( batch_counter ==5)    // if we reached 4 batches   5
					{
					 batch_counter=1;
					 TX_POW++;								
					}
							
				 if( TX_POW == 5)         // if we reached 4 tx power
					{
						batch_counter=1;
						TX_POW = 1;
					}
				 
				 if( idx_counter <17)    // if 16 batches reached, 4 batches packets X 4 tx power     
				  {
            nrf_delay_ms(waiting_delay);
						if(BLE_MODE==1){	radio_ble_1Mbit_scenario_start();}
						if(BLE_MODE==2){	radio_ble_2Mbit_scenario_start();}
						if(BLE_MODE==3){	radio_ble_500Kbit_scenario_start();}
						if(BLE_MODE==4){	radio_ble_125Kbit_scenario_start();}							
				  }
							 
					if( idx_counter == 17)   // if we reached 16 batch      
					 {  							
										
						if( data_idx_counter == 17){CHANNEL = 15;}          
						if( data_idx_counter == 33){CHANNEL = 28;}	                       							 
            if( data_idx_counter == 49){CHANNEL= 4; PACKET_CONFIG++; data_idx_counter=1;}  
                   
									 
								//	 if( PACKET_CONFIG == 2)       // this is only for fec
                //    { 
									 //   measurement_timer_set_interval(1100);
										//	printing_delay = 900;
									//		if(BLE_MODE==3){ measurement_timer_set_interval(600); printing_delay =500;	} 					
					        //    if(BLE_MODE==4){ measurement_timer_set_interval(850); printing_delay = 500;	}
																			 																																			
								//		}
									 
									 
						if( PACKET_CONFIG == 3)
             { 											
					//	if( PACKET_CONFIG == 3 && CHANNEL == 4 ) {nrf_delay_ms(5000); }
							measurement_timer_set_interval(1100);
						  printing_delay = 900;
							if(BLE_MODE==3){ measurement_timer_set_interval(1600); printing_delay =1200;	} 					
					    if(BLE_MODE==4){ measurement_timer_set_interval(4850); printing_delay = 2000;	}																																																						
						 }
                   
            if( PACKET_CONFIG != 4)
             {
               idx_counter=1;
							 nrf_delay_ms(200);
							 if(BLE_MODE==1){	radio_ble_1Mbit_scenario_start();}
							 if(BLE_MODE==2){	radio_ble_2Mbit_scenario_start();}
               if(BLE_MODE==3){	radio_ble_500Kbit_scenario_start();}
							 if(BLE_MODE==4){	radio_ble_125Kbit_scenario_start();}																																											
						 } 																			  									 									  
					 }	
			   }
			 
			else
			 {
				 PACKET_CONFIG=1;
				 data_idx_counter=1;					
				 measurement_timer_set_interval(600); 
				 printing_delay = 400;
         if(BLE_MODE==3){ measurement_timer_set_interval(1100); printing_delay =900;	} 					
				 if(BLE_MODE==4){ measurement_timer_set_interval(850); printing_delay = 600;	}					
				 	
				}			
 	
}

/* Function for processing scan measurement  */
void process_measurement()
{
				
	  if (le_data_packet_rx[2] == 0x11 && le_data_packet_rx[3] == 0x11 )
			{

				if ( measuring == false) 
         {
          nrf_drv_timer_enable(&MEASUREMENT_TIMER);
          measuring = true;
					nrf_gpio_pin_clear(LED_3_PIN);
         }
				nrf_gpio_pin_toggle(LED_4_PIN);
				B1_rec_packets_rssi[B1_packet_counter+1] = rssi_result;
				B1_rec_packets_rssi[B1_packet_counter+252]=le_data_packet_rx[8];
        
				B1_packet_counter++;
				B1_rec_ind = true;
				broadcaster_missed = false;
				rssi_result=0;
				le_data_packet_rx[2]=0;
				le_data_packet_rx[3]=0;
			} 	
}


/* Function for handling measurement timer interrupts	*/
void MEASUREMENT_TIMER_event_handler(nrf_timer_event_t event_type, void* p_context)
{    
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:                  // this event is triggered when the timer fires
				{
				   nrf_drv_timer_disable(&MEASUREMENT_TIMER);
					 nrf_gpio_pin_set(LED_3_PIN);
					 B1_packet_counter=0;
					 nrf_delay_ms(printing_delay);
					 
					 B1_rec_packets_rssi[0]=253;
															
					 for (uint16_t i=2; i < 252; i++)                       
	            { 
								if(B1_rec_packets_rssi[i]==0){PER++;}     
                nrf_delay_us(20);								
			        }
							B1_rec_packets_rssi[503]=PER;
							
							
							for (uint16_t z=0; z < 504; z++)              
	            { 
								while(app_uart_put(B1_rec_packets_rssi[z]) != NRF_SUCCESS);						
			        }
							
					  for (uint16_t x = 0; x < 504; x++)           
	            {        								
                B1_rec_packets_rssi[x]=0;				           // empty the buffer								
			        }
							
							if(scenario==1){process_adv_scenario();}
							if(scenario==2){process_data_scenario();}
							
								
							}
				            			
            break;
					 													
        default:
            //Do nothing.
            break;
    }
}



/* Function for button presses	*/
void button_handler(nrf_drv_gpiote_pin_t pin_no, nrf_gpiote_polarity_t button_action)
{    
        switch (pin_no)
        {
            case BSP_BUTTON_0:      
							 
						     scenario=1;            //adv
						     nrf_delay_ms(700);
				         nrf_gpio_pin_clear(LED_1_PIN);
						     radio_ble_1Mbit_scenario_start();  
						     	
						     break;
						
						case BSP_BUTTON_1:

                scenario=2;         //data
						    CHANNEL=4;
						    measurement_timer_set_interval(1100);
								printing_delay = 600;
						    nrf_delay_ms(700);						    
				        nrf_gpio_pin_clear(LED_1_PIN);
            //    radio_ble_1Mbit_scenario_start();
                radio_ble_125Kbit_scenario_start();						
               						
                 break;								 
                         				
				    case BSP_BUTTON_2:
							   
						     
						     scenario=2;         //data
						     CHANNEL=4;
						     measurement_timer_set_interval(850);
								 printing_delay = 600;
						     nrf_delay_ms(700);						    
				         nrf_gpio_pin_clear(LED_1_PIN);
					   //    radio_ble_2Mbit_scenario_start();						     
					        
					       radio_ble_500Kbit_scenario_start();
							  
                 break;					                 
        
		     	 case BSP_BUTTON_3:
						     
					       nrf_delay_ms(700);
					       if(BLE_MODE==1){	radio_ble_1Mbit_scenario_start();}
							   if(BLE_MODE==2){	radio_ble_2Mbit_scenario_start();}								 
								 if(BLE_MODE==3){	radio_ble_500Kbit_scenario_start();}
					       
						     						 						 																    						    
                 break;       
        }  
}




// Function to Create timers
static void create_timers()  
{ 
	
	  uint32_t err_code = NRF_SUCCESS;
    
	  uint32_t M_STOP_TIMER_ms = 500;   // half second
	  uint32_t M_STOP_TIMER_ticks;
	
	  uint32_t MEASUREMENT_TIMER_ms = SCAN_INTERVAL; //Time (in miliseconds) between consecutive compare events.
    uint32_t MEASUREMENT_TIMER_ticks;
	
	  uint32_t TIMESTAMP_TIMER_ms = 1000;   // 1 second
	  uint32_t TIMESTAMP_TIMER_ticks;
	
	  nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
		uint32_t TIMER_err_code = NRF_SUCCESS;
    TIMER_err_code = nrf_drv_timer_init(&MEASUREMENT_TIMER, &timer_cfg, MEASUREMENT_TIMER_event_handler);
		TIMER_err_code = nrf_drv_timer_init(&TIMESTAMP_TIMER, &timer_cfg, TIMESTAMP_TIMER_event_handler);
		TIMER_err_code = nrf_drv_timer_init(&M_STOP_TIMER, &timer_cfg, M_STOP_TIMER_event_handler);
    APP_ERROR_CHECK(TIMER_err_code); 
		
		M_STOP_TIMER_ticks    = nrf_drv_timer_ms_to_ticks(&M_STOP_TIMER, M_STOP_TIMER_ms);
    MEASUREMENT_TIMER_ticks  = nrf_drv_timer_ms_to_ticks(&MEASUREMENT_TIMER, MEASUREMENT_TIMER_ms);
    TIMESTAMP_TIMER_ticks    = nrf_drv_timer_ms_to_ticks(&MEASUREMENT_TIMER, TIMESTAMP_TIMER_ms);
		
		nrf_drv_timer_extended_compare(&M_STOP_TIMER, NRF_TIMER_CC_CHANNEL0, M_STOP_TIMER_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
		nrf_drv_timer_extended_compare(&MEASUREMENT_TIMER, NRF_TIMER_CC_CHANNEL0, MEASUREMENT_TIMER_ticks, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);
		nrf_drv_timer_extended_compare(&TIMESTAMP_TIMER, NRF_TIMER_CC_CHANNEL0, TIMESTAMP_TIMER_ticks, NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, true);
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
		
		
		bsp_board_leds_init();

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
	//  nrf_gpio_pin_clear(LED_1_PIN);
    NRF_RADIO->EVENTS_READY = 0U;
    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;
    while (NRF_RADIO->EVENTS_READY == 0U) {/* wait*/}
    		
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;
		
		NRF_RADIO->TASKS_RSSISTART = 1;
    while(!NRF_RADIO->EVENTS_RSSIEND);
    NRF_RADIO->EVENTS_RSSIEND = 0;
		
    // Wait for end of packet 
    while (NRF_RADIO->EVENTS_END == 0U) {/* wait*/} 
		
//		current_tick = nrf_drv_timer_capture(&TIMESTAMP_TIMER,  NRF_TIMER_CC_CHANNEL1);                         
    if (NRF_RADIO->CRCSTATUS == 1U)
    { 	
			  rssi_result = NRF_RADIO->RSSISAMPLE;
        NRF_RADIO->TASKS_RSSISTOP = 1;
			  process_measurement();  			
    } 
		
		if (NRF_RADIO->CRCSTATUS == 0U)
    {          				
     // to do something here when the crc returns error	
   //  nrf_gpio_pin_clear(LED_4_PIN);   		
    }
		
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;
    while (NRF_RADIO->EVENTS_DISABLED == 0U) {/* wait*/}
    
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
          UART_BAUDRATE_BAUDRATE_Baud1M
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
    uint32_t err_code;

    clock_initialization();

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	 
	  NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
	  
	  radio_power_up(); 
	  radio_addr_init();	
	  radio_CRC_init();
		radio_tifs_set();		
    radio_default_init();
	  gpio_config();
		UART_initialize();
		create_timers(); 
		        
    for (;;)
    {			   
         if (tp_sent) {read_packet(); }    //   if trigger packet is sent, start scanning
    }
}

