// Sniffer


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


// Local variables//   
uint8_t RSSI_array[40000];
uint32_t RSSI_array_size;
uint8_t  controlArray[6];
uint8_t  uart_rx_buffer[504];
uint16_t  uart_rx_buffer_idx;
uint32_t array_idx;
uint32_t array_counter;
uint8_t rssi_counter=1;
uint8_t uart_idx=1;
uint8_t recValue;
uint8_t channel_idx;
int8_t p_rssi;
int8_t batch_counter=1;
int8_t idx_counter=1;
uint8_t rssi_result;
uint8_t PER;
uint8_t PACKET_CONFIG;
uint8_t BLE_MODE;
uint8_t TX_POW;
uint8_t CHANNEL;


uint8_t advCh360;
uint8_t scan_idx360;
uint8_t scan_round360;
uint8_t scenario360;
uint8_t batch_counter360=1;
uint8_t lost_packets;

char *radio__mode[]={"","1Mbit", "2Mbit", "500Kbit S=2", "125Kbit S=8"};
char *packet__config[]={"","Avg (Adv), 37 Bytes Payload", "Min, 7 Byte Payload", "Max, 255 Bytes Payload" };   // not including hte header and device address
char *tx__power[]={"", "+8dBm", "0dBm", "-8dBm", "-20dBm"}; 
char *channel_num[]={"", "38", "39", "37"};


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

// Function for sniffing the background noise 
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


// Function for saving background noise to an array 
void sniff_env_rssi_observer()
{
   int value;							
   nrf_drv_gpiote_out_clear(LED_4_PIN);				   			   									
											
	for ( uint16_t a=0; a<RSSI_array_size; a++)
	  {
		  value = do_env_rssi_sampling(channel_idx);
			RSSI_array[a]= value;      
    }
																					 
	 nrf_drv_gpiote_out_set(LED_4_PIN);	
         								 											        	
}

// Function for processing background noise 
void get_env_rssi_data()
{
	while (app_uart_get(&recValue) == NRF_SUCCESS)  // wait until channel idx value is received			
		   {
				 uart_rx_buffer[uart_rx_buffer_idx] = recValue;
		     uart_rx_buffer_idx++;				 				 
		   }
	         	
  if( uart_rx_buffer_idx ==6 && uart_rx_buffer[0]==254)     // for path loss. point to pint tranmission
	 {
		 nrf_gpio_pin_clear(LED_1_PIN);		 
	   BLE_MODE = uart_rx_buffer[1];
	   CHANNEL= uart_rx_buffer[2];
	   PACKET_CONFIG= uart_rx_buffer[3];
	   TX_POW= uart_rx_buffer[4];
	   batch_counter= uart_rx_buffer[5];
		 
		 if( BLE_MODE == 1 && PACKET_CONFIG != 3) { RSSI_array_size = 4000;}
		 if( BLE_MODE == 1 && PACKET_CONFIG == 3) { RSSI_array_size = 8000;}
		 
		 if( BLE_MODE == 2 && PACKET_CONFIG != 3) { RSSI_array_size = 4000;}
		 if( BLE_MODE == 2 && PACKET_CONFIG == 3) { RSSI_array_size = 8000;}
		 
		 if( BLE_MODE == 3 && PACKET_CONFIG == 1) { RSSI_array_size = 6000;}
		 if( BLE_MODE == 3 && PACKET_CONFIG == 2) { RSSI_array_size = 4000;}
		 if( BLE_MODE == 3 && PACKET_CONFIG == 3) { RSSI_array_size = 12000;}
		 
		 if( BLE_MODE == 4 && PACKET_CONFIG == 1) { RSSI_array_size = 8000;}
		 if( BLE_MODE == 4 && PACKET_CONFIG == 2) { RSSI_array_size = 6000;}
		 if( BLE_MODE == 4 && PACKET_CONFIG == 3) { RSSI_array_size = 38000;}
		 
		 
		 
		if (CHANNEL == 37) {channel_idx=2U;   sniff_env_rssi_observer();}    //   channel 37 
		if (CHANNEL == 38) {channel_idx=26U;  sniff_env_rssi_observer();}    //   channel 38
	  if (CHANNEL == 39) {channel_idx=80U;  sniff_env_rssi_observer();}    //   channel 39
		if (CHANNEL ==  4) {channel_idx=12U;  sniff_env_rssi_observer();}    //   channel 4
		if (CHANNEL == 15) {channel_idx=36U;  sniff_env_rssi_observer();}    //   channel 15
		if (CHANNEL == 28) {channel_idx=62U;  sniff_env_rssi_observer();}    //   channel 28
		
		      if(batch_counter==1)                             
					 { 
             printf("\r\n");						 
						 printf("BLE Mode: %s,   Channel: %d,  Packet Configuration: %s,  Tx Power: %s    \r\n", radio__mode[BLE_MODE] , CHANNEL, packet__config[PACKET_CONFIG], tx__power[TX_POW] );
						 printf("---------------------------------------------------------------------\r\n");
						 printf("\r\n");
						 nrf_delay_us(20);
					 } 
					 
					   printf("\r\n\r\n");
		   	     nrf_delay_us(20);
					   printf("Packet batch #%d: \r\n", batch_counter);
		         nrf_delay_us(20);
					   printf("----------------\r\n");
		         nrf_delay_us(20);
					 
					   printf("\r\n");
					   printf("RSSI of Environment:");
					   printf("\r\n");
					   nrf_delay_us(20);
					
					 array_idx= RSSI_array_size/1000;
					 
			for( uint16_t z=0; z < array_idx; z++)
       {					 
				 for (uint16_t y=0; y < 1000; y++)                               
	        { 
				   printf("-%d", RSSI_array[y+array_counter] );
           printf(",");					 
				   nrf_delay_us(20);					 
			    }
				printf("\r\n");
				array_counter+=1000;
			 }
					         
		
		printf("\r\n");
		app_uart_flush(); 
		uart_rx_buffer_idx=0;
		array_counter=0;

    for (uint8_t v=0; v < 6; v++)                                       
	    { 
			  uart_rx_buffer[v]=0;      
			}			
	 }	 
			 
			 		 
		 	 
	if( uart_rx_buffer_idx ==504 && uart_rx_buffer[0]==253)
	 {
		    nrf_gpio_pin_clear(LED_2_PIN);
		         
		         printf("\r\n\r\n");
					   printf("RSSI result: \r\n");
					   for (uint8_t i=2; i < 252; i++)                  
	            { 
								printf("-%d,", uart_rx_buffer[i] );
								nrf_delay_us(20);
			        }
							printf("\r\n");
							nrf_delay_us(20);
							printf("Received Packet Number:");
							nrf_delay_us(20);
							printf("\r\n");
							nrf_delay_us(20);
							
							for (uint16_t y =253; y < 503; y++)           
	            { 
								printf("%d,", uart_rx_buffer[y] );
								nrf_delay_us(20);
			        }
							printf("\r\n");
							
							nrf_delay_us(20);
							printf("PER: %d", uart_rx_buffer[503]);
							nrf_delay_us(20);
							printf("\r\n\r\n");
						

   for (uint16_t v=0; v < 504; v++)                                    
	    { 
			  uart_rx_buffer[v]=0;      
			}	

    app_uart_flush(); 
		uart_rx_buffer_idx=0;			
	 } 
	 
	if( uart_rx_buffer_idx ==5 && uart_rx_buffer[0]==252)       // from here til down is for the 360 degree scenario
	 {  
		  nrf_gpio_pin_clear(LED_3_PIN);
		  advCh360 = uart_rx_buffer[1];
      scan_idx360 = uart_rx_buffer[2];
      scan_round360 = uart_rx_buffer[3];
      scenario360 = uart_rx_buffer[4];
		  RSSI_array_size = 10000;
		 
		  if (advCh360 == 1 && scan_idx360 == 0) {channel_idx=26U; sniff_env_rssi_observer(); printf("Environment RSSI on channel: 38\r\n");}   // ch: 38
			if (advCh360 == 2 && scan_idx360 == 0) {channel_idx=80U; sniff_env_rssi_observer(); printf("Environment RSSI on channel: 39\r\n");}   // ch: 39
			if (advCh360 == 3 && scan_idx360 == 0) {channel_idx=2U;  sniff_env_rssi_observer(); printf("Environment RSSI on channel: 37\r\n");}    // ch: 37
			
			if ( scan_idx360 == 0)
			{
			 array_idx= RSSI_array_size/1000;					 
			 for( uint16_t z=0; z < array_idx; z++)
        {					 
				 for (uint16_t y=0; y < 1000; y++)                               
	        { 
				   printf("-%d", RSSI_array[y+array_counter] );
           printf(",");					 
				   nrf_delay_us(20);					 
			    }
				 printf("\r\n");				
				 array_counter+=1000;
			 }
			nrf_delay_us(20);
			printf("\r\n\r\n");
		 
		}	
   
		 app_uart_flush(); 
		 uart_rx_buffer_idx=0;
		 array_counter=0;

    for (uint8_t v=0; v < 5; v++)                                       
	    { 
			  uart_rx_buffer[v]=0;      
			}
		
	 }
	 
	 if( uart_rx_buffer_idx ==201 && uart_rx_buffer[0]==251)
	 { 
     printf("\r\n");		 
		 printf("Packet batch #%d    Channel: %s     Direction: %d degrees    Scan round: %d \r\n", batch_counter360, channel_num[advCh360], scan_idx360*5, scan_round360);
		 printf("----------------\r\n");
		 printf("\r\n");
		 printf("Received packets:\r\n");
     for (uint8_t i=0; i < 100; i++)                  
	    { 
				printf("%d,", uart_rx_buffer[i+101] );
				if (uart_rx_buffer[i+101]==0) {lost_packets++;}
				nrf_delay_us(20);
			}
     printf("\r\n");
		 nrf_delay_us(20);
		 printf("\r\n");

    printf("Received RSSI:\r\n");
     for (uint8_t i=0; i < 100; i++)                  
	    { 
				if ( uart_rx_buffer[i+1] !=0) {printf("%d,", uart_rx_buffer[i+1]-256 );}
				if ( uart_rx_buffer[i+1] ==0) {printf("%d,", uart_rx_buffer[i+1]);}
				nrf_delay_us(20);
			}
     printf("\r\n");
		 nrf_delay_us(20);
     printf("\r\n");
		 nrf_delay_us(20);
		 printf("PER: %d", lost_packets);
		 nrf_delay_us(20);
		 printf("\r\n");
  //   printf("---------------\r\n");
		 nrf_delay_us(20);
     printf("\r\n");
		 nrf_delay_us(20);
     printf("\r\n");

     for (uint16_t v=0; v < 201; v++)                                    
	    { 
			  uart_rx_buffer[v]=0;      
			}	

     app_uart_flush(); 
		 uart_rx_buffer_idx=0;
     lost_packets=0;
     batch_counter360++;			
		 
	 }
		 
	   			         				         	
}

/* Function for initializing the clock	*/
void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

}

void print_rssi()
{
	array_idx= RSSI_array_size/1000;					 
			 for( uint16_t z=0; z < array_idx; z++)
        {					 
				 for (uint16_t y=0; y < 1000; y++)                               
	        { 
				   printf("-%d", RSSI_array[y+array_counter] );
           printf(",");					 
				   nrf_delay_us(20);					 
			    }
				 printf("\r\n");				
				 array_counter+=1000;
			 }
       printf("\r\n");
			 printf("\r\n");
			 array_counter=0;

}


/* Function for button presses	*/
void button_handler(nrf_drv_gpiote_pin_t pin_no, nrf_gpiote_polarity_t button_action)
{    
        switch (pin_no)
        {
            case BSP_BUTTON_0: 
							
						     channel_idx=2U; //   channel 37
                 RSSI_array_size = 40000;
                 printf("Environment RSSI on channel: 37\r\n\r\n");						
						     sniff_env_rssi_observer();    
						     print_rssi();
						     
		        	          

						     break;
						
						case BSP_BUTTON_1:
							
						       channel_idx=26U;   //   channel 38
						       RSSI_array_size = 40000;
						       printf("Environment RSSI on channel: 38\r\n\r\n");
									 sniff_env_rssi_observer();    
	           			 print_rssi();
						
                 break;								 
                         				
				    case BSP_BUTTON_2:
							
						      channel_idx=80U; //   channel 39
                  RSSI_array_size = 40000;	
                  printf("Environment RSSI on channel: 39\r\n\r\n");						
						      sniff_env_rssi_observer();    
						      print_rssi();
						   						  
                 break;					                 
        
		    	 case BSP_BUTTON_3:
						 						 						 																    						    
                break;       
        }  
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
    uint32_t err_code = NRF_SUCCESS;
    
    clock_initialization();
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);	 
	  NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;	
	  gpio_config();
		UART_initialize();
		
    
    for (;;)
    {			   
				 get_env_rssi_data();
    }
}

