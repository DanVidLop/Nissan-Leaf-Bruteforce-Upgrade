
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <util/delay.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include "Descriptors.h"
#include "LUFA/Platform/Platform.h"
#include "LUFA/Drivers/USB/USB.h"
#include "sp_driver.h"
#include "mcp25xx.h"
#include "user_defines.h"
#include "helper_functions.h"

/********************/
/* Custom settings  */
/********************/
#define USB_SERIAL							//Enable SERIAL port

#define UNITY_GIDS					80		// Watts/GID. Used to calculate GIDS. With 80W/GIDS we obtain 800 GIDS.
#define WATTS_PER_VOLT				667		// Watts of power per volt of battery available. 96V (from 300 to 396V) to cover 64000 Wh battery. => 64000Wh/96V.
#define GIDS_MEASURE_CYLES			100		// Number of cycles to add power to GIDS. Used to make less reactive the available Kilometers.
#define BATTERY_FULL_CAPACITY		64000	// 64KWh
#define	GIDS_PER_VOLT				3		// Used at calibration time. We consider linear behaviour of battery.
#define STOCK_GIDS					288		//Stock battery gids
#define ZERO_WATTS_VOLT				600		// Zero power battery voltage 300V In LBC_BatteryVoltageSignal units (0.5V).
#define LBC_MIN_VALID_VOLTS			600		//300V In LBC_BatteryVoltageSignal units (0.5V)
#define LBC_MAX_VALID_VOLTS			800		//400V In LBC_BatteryVoltageSignal units (0.5V)
#define LBC_MIN_VALID_AMPS			-300	//-150A In LBC_BatteryCurrentSignal units (0.5A)
#define LBC_MAX_VALID_AMPS			400		//200A In LBC_BatteryCurrentSignal units (0.5A)
#define LBC_MY_CHARGE_TIME			60		//Minutes
#define CURRENT_SET_WAITING_TIME	10		//10 secs, 500ms each message 0x1DC

				
/*************************************/
/*  Charging status at 0x1F2 command */
/*************************************/
#define CHARGING_QUICK_START	0x40
#define CHARGING_QUICK			0xC0
#define CHARGING_QUICK_END		0xE0
#define CHARGING_SLOW			0x20
#define CHARGING_IDLE			0x60

/********************/
/* Charging params  */
/********************/
#define CHARGING_FORCE_DEFAULT	1
#define CHARGING_VOLT_CUT		784  // Charger stop level..392v In LBC_BatteryVoltageSignal units (0.5V)
#define CHADEMO_MAX_CHARGE_RATE 0x271

//defines
#define TC0_CLKSEL_DIV1_gc		0b0001
#define TC0_CLKSEL_DIV256_gc	0b0110
#define TC0_CLKSEL_DIV1024_gc	0b0111
#define TC0_OVFINTLVL_HI_gc		0b11
#define TC0_OVFINTLVL_LO_gc		0b01
#define TC0_CCAINTLVL_HI_gc		0x03
#define TC0_CCBINTLVL_HI_gc		0x0C
#define TC0_CCCINTLVL_HI_gc		0x30
#define TC0_WGMODE_SINGLESLOPE_bm	0x03

#define OUTBUF_SIZE				2048

#define TXBUFFER_SIZE			16

//function prototypes
void hw_init(void);
void print(char * str, uint8_t len);
void uint32_to_str(char * str, uint32_t num);
void SID_to_str(char * str, uint32_t num);
//void blink(void);
void canframe_to_str(char * str, can_frame_t frame);
void send_can(uint8_t can_bus, can_frame_t frame);
void send_can1(can_frame_t frame);
void send_can2(can_frame_t frame);
void send_can3(can_frame_t frame);
void can_handler(uint8_t can_bus);
void ProcessCDCCommand(void);
void check_can1(void);
void check_can2(void);
void check_can3(void);



USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = {
	.Config = {
		.ControlInterfaceNumber   = 0,
		.DataINEndpoint           = {
			.Address          = CDC_TX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.DataOUTEndpoint =	{
			.Address          = CDC_RX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.NotificationEndpoint =	{
			.Address          = CDC_NOTIFICATION_EPADDR,
			.Size             = CDC_NOTIFICATION_EPSIZE,
			.Banks            = 1,
		},
	},
};
