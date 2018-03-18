
#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

	// The Broadcaster has the following format:
  // PDU Type(4bits) | RFU(1bit) | ChSel (1bit) | TxAdd(1bits) | RxAdd(1bit) | Length(8bits) 

#define PACKET_S0_FIELD_LENGTH             1UL     // Packet S0 length field size in bytes
#define PACKET_S1_FIELD_LENGTH             0UL     // Packet S1 length field size in bits
#define PACKET_LENGTH_FIELD_LENGTH         8UL	   // Packet length field size in bits
#define PACKET_BASE_ADDRESS_LENGTH       3UL     // Packet base address length field size in bytes
#define PACKET_STATIC_LENGTH             0UL     // Packet static length in bytes
#define PACKET_PAYLOAD_MAXSIZE           255UL   // Packet payload maximum size in bytes for on nRF52840
#define PACKET_PAYLOAD_MAXSIZE_LE_MIN    7UL     // Packet payload maximum size in bytes for LE min packet
#define PACKET_PAYLOAD_MAXSIZE_LE_ADV    37UL    // Packet payload maximum size in bytes for LE adv packet
#define PACKET_PAYLOAD_MAXSIZE_LE_DATA   255UL   // Packet payload maximum size in bytes for LE data packet
#define	PACKET_CI_FIELD_LENGTH          2UL     // FEC_Packet CI length field size in bits
#define	PACKET_TERM_FIELD_LENGTH        3UL     // FEC_Packet TERM length field size in bits
	
#define PDU_HEADER_LENGTH          2UL          // Length of the PDU header, 2 bytes	
#define PDU_HEADER_S0_VALUE        2UL          // ADV_NONCONN_IND (0010): Non connectable undirected advertising.
#define PDU_HEADER_S1_VALUE        0UL          // S1 value will be omitted in the payload, not required for ble standard
#define PDU_LENGTH_LE_MIN          9UL          // 2 Byets header S0 and LENGTH && 6 Bytes device address
#define PDU_LENGTH_LE_ADV          39UL         // 2 Byets header S0 and LENGTH && 6 Bytes device address && 31 payload
#define PDU_LENGTH_LE_DATA         257UL        // 257 Bytes, BLE data channel max payload
	

#define ADV_ACCESS_ADRS				0x8E89BED6
#define LE_ADDR_LENGTH        6 	

#define B1_ADDR                         1
#define B2_ADDR                         2

#define PACKET_CONFIG_TRIGGER           0
#define PACKET_CONFIG_ADV               1
#define PACKET_CONFIG_MIN               2
#define PACKET_CONFIG_DATA              3

#define BLE_1MBIT           1
#define BLE_2MBIT           2
#define BLE_LR500KBIT       3
#define BLE_LR125KBIT       4

#define TX_PWR_Pos8dBm      1
#define TX_PWR_0dBm         2
#define TX_PWR_Neg8dBm      3
#define TX_PWR_Neg20dBm     4
	
#define ADV_IDX_CH_37				0
#define ADV_IDX_CH_38				1
#define ADV_IDX_CH_39				2
#define ADV_IDX_CH_MAX			3
	
#define RADIO_TXPOWER_MAX_VALUE      15 

#define RADIO_TXPOWER_TXPOWER_0dBm (0x0UL) /*!< 0 dBm */
#define RADIO_TXPOWER_TXPOWER_Pos2dBm (0x2UL) /*!< +2 dBm */
#define RADIO_TXPOWER_TXPOWER_Pos3dBm (0x3UL) /*!< +3 dBm */
#define RADIO_TXPOWER_TXPOWER_Pos4dBm (0x4UL) /*!< +4 dBm */
#define RADIO_TXPOWER_TXPOWER_Pos5dBm (0x5UL) /*!< +5 dBm */
#define RADIO_TXPOWER_TXPOWER_Pos6dBm (0x6UL) /*!< +6 dBm */
#define RADIO_TXPOWER_TXPOWER_Pos7dBm (0x7UL) /*!< +7 dBm */
#define RADIO_TXPOWER_TXPOWER_Pos8dBm (0x8UL) /*!< +8 dBm */
#define RADIO_TXPOWER_TXPOWER_Pos9dBm (0x9UL) /*!< +9 dBm */
#define RADIO_TXPOWER_TXPOWER_Neg30dBm (0xD8UL) /*!<   -40 dBm */
#define RADIO_TXPOWER_TXPOWER_Neg40dBm (0xD8UL) /*!< -40 dBm */
#define RADIO_TXPOWER_TXPOWER_Neg20dBm (0xECUL) /*!< -20 dBm */
#define RADIO_TXPOWER_TXPOWER_Neg16dBm (0xF0UL) /*!< -16 dBm */
#define RADIO_TXPOWER_TXPOWER_Neg12dBm (0xF4UL) /*!< -12 dBm */
#define RADIO_TXPOWER_TXPOWER_Neg8dBm (0xF8UL) /*!< -8 dBm */
#define RADIO_TXPOWER_TXPOWER_Neg4dBm (0xFCUL) /*!< -4 dBm */

// Pins for LED's and buttons.
// The diodes on the DK are connected with the cathodes to the GPIO pin, so         
// clearing a pin will light the LED and setting the pin will turn of the LED.
#define LED_1_PIN                       BSP_LED_0                    // LED 1 on the nRF51-DK or nRF52-DK
#define LED_2_PIN                       BSP_LED_1                    // LED 3 on the nRF51-DK or nRF52-DK
#define LED_3_PIN                       BSP_LED_2                    // LED 3 on the nRF51-DK or nRF52-DK
#define LED_4_PIN                       BSP_LED_3                    // LED 3 on the nRF51-DK or nRF52-DK
#define BUTTON_1_PIN                    BUTTON_1                     // Button 1 on the nRF51-DK or nRF52-DK
#define BUTTON_2_PIN                    BUTTON_2                     // Button 2 on the nRF51-DK or nRF52-DK
#define BUTTON_3_PIN                    BUTTON_3                     // Button 1 on the nRF51-DK or nRF52-DK
#define BUTTON_4_PIN                    BUTTON_4                     // Button 2 on the nRF51-DK or nRF52-DK

#define UART_TX_BUF_SIZE 1024                                        //UART TX buffer size.
#define UART_RX_BUF_SIZE 512
	

void radio_configure(void);


#ifdef __cplusplus
}
#endif

#endif
