#include "can-bridge-firmware.h"

//Status						
volatile	uint8_t		poweroff					= 1;	//1 if the car is ready to go and 0 if it is stopped
volatile	uint8_t		calibration_pending			= 1;	//1 if calibration procedure should be fired
volatile	uint8_t		charging_state				= 0;	//current charging status

//GIDS calculation
volatile	uint16_t	power_samples				= 0;	//Number of samples before update GIDS
volatile	float		power_sum					= 0.0;	//Accumulator for samples
volatile	uint16_t	total_gids					= 0;	//GIDS to be reported

//LBC related data
volatile	uint16_t	LBC_MainGids				= 0;
volatile	uint16_t	LBC_StateOfCharge			= 0;
volatile	uint16_t	LBC_BatteryVoltageSignal	= 0;	//LSB = 0.5V
volatile	int16_t		LBC_BatteryCurrentSignal	= 0;	//LSB = 0.5A
volatile	float		LBC_PowerStatus				= 0;	//Current battery charging status

//Delay of charging process start
volatile	uint16_t	charging_wait_b4_set_current = CURRENT_SET_WAITING_TIME;

//Quick Charge current
volatile	uint16_t	chademo_power_rate			= CHADEMO_MAX_CHARGE_RATE; 
//example valid CAN frame
volatile	can_frame_t	static_message = {.can_id = 0x5BC, .can_dlc = 8, .data = {0,0,0,0,0,0,0,0}};


#ifdef USB_SERIAL
	
	uint8_t ReadCalibrationByte( uint8_t index );
	void ProcessCDCCommand(void);

	uint8_t		configSuccess				= false;		//tracks whether device successfully enumerated
	static FILE USBSerialStream;							//fwrite target for CDC
	uint8_t		signature[11];								//signature bytes
	//print variables
	volatile	uint8_t		print_char_limit		= 0;

	//appends string to ring buffer and initiates transmission
	void print(char * str, uint8_t len){
		if((print_char_limit + len) <= 128){
			fwrite(str, len, 1, &USBSerialStream);
			print_char_limit += len;
			} else {
			fwrite("X\n",2,1,&USBSerialStream);
		}
	}
#endif

//variables for ProcessCDCCommand()
volatile	int16_t		cmd, cmd2;
volatile	uint16_t	i = 0, j = 0, k = 0;
volatile	uint32_t	temp;
volatile	uint16_t	ReportStringLength;
			char *		ReportString;	
				
volatile	uint8_t		can_busy			= 0;		//tracks whether the can_handler() subroutine is running

//timer variables
volatile	uint8_t		ten_sec_timer		= 1;		//increments on every sec_timer underflow
volatile	uint16_t	sec_timer			= 1;		//actually the same as ms_timer but counts down from 1000
volatile	uint8_t		sec_interrupt		= 0;		//signals main loop to output debug data every second
volatile	uint16_t	ms_timer			= 1;		//increments on every TCC0 overflow (ever ms)

//because the MCP25625 transmit buffers seem to be able to corrupt messages (see errata), we're implementing
//our own buffering. This is an array of frames-to-be-sent, FIFO. Messages are appended to buffer_end++ as they
//come in and handled according to buffer_pos until buffer_pos == buffer_end, at which point both pointers reset
//the buffer size should be well in excess of what this device will ever see
can_frame_t tx0_buffer[TXBUFFER_SIZE];
uint8_t		tx0_buffer_pos		= 0;
uint8_t		tx0_buffer_end		= 0;

can_frame_t tx2_buffer[TXBUFFER_SIZE];
uint8_t		tx2_buffer_pos		= 0;
uint8_t		tx2_buffer_end		= 0;

can_frame_t tx3_buffer[5];
uint8_t		tx3_buffer_pos		= 0;
uint8_t		tx3_buffer_end		= 0;

void hw_init(void){
	uint8_t caninit;

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, 48000000);		
	
	
	
	//turn off everything we don' t use
	//PR.PRGEN		= PR_AES_bm | PR_RTC_bm | PR_DMA_bm;
	#ifdef USB_SERIAL
		//PR.PRGEN		=  PR_AES_bm | PR_RTC_bm | PR_EVSYS_bm | PR_DMA_bm; // stop the clock to AES,RTC,Event system and DMA controller
		PR.PRGEN		= PR_AES_bm | PR_RTC_bm | PR_DMA_bm;
	#else
		PR.PRGEN		= PR_USB_bm | PR_AES_bm | PR_RTC_bm | PR_EVSYS_bm | PR_DMA_bm; // stop the clock to USB, AES,RTC,Event system and DMA controller
	#endif
	
	PR.PRPA			= PR_ADC_bm | PR_AC_bm;
	PR.PRPC			= PR_TWI_bm | PR_USART0_bm | PR_HIRES_bm;
	PR.PRPD			= PR_TWI_bm | PR_USART0_bm | PR_TC0_bm | PR_TC1_bm;
	PR.PRPE			= PR_TWI_bm | PR_USART0_bm;
	
	//blink output
	PORTB.DIRSET	= 3;
	
	//start 16MHz crystal and PLL it up to 48MHz
	OSC.XOSCCTRL	= OSC_FRQRANGE_12TO16_gc |		//16MHz crystal
	OSC_XOSCSEL_XTAL_16KCLK_gc;						//16kclk startup
	OSC.CTRL	   |= OSC_XOSCEN_bm;				//enable crystal
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));			//wait until ready
	OSC.PLLCTRL		= OSC_PLLSRC_XOSC_gc | 2;		//XTAL->PLL, 2x multiplier (32MHz)
	OSC.CTRL	   |= OSC_PLLEN_bm;					//start PLL
	while (!(OSC.STATUS & OSC_PLLRDY_bm));			//wait until ready
	CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
	CLK.CTRL		= CLK_SCLKSEL_PLL_gc;			//use PLL output as system clock	
	
	//output 16MHz clock to MCP25625 chips (PE0)
	//next iteration: put this on some other port, pin 4 or 7, so we can use the event system
	TCE0.CTRLA		= TC0_CLKSEL_DIV1_gc;						//clkper/1
	TCE0.CTRLB		= TC0_CCAEN_bm | TC0_WGMODE_SINGLESLOPE_bm;	//enable CCA, single-slope PWM
	TCE0.CCA		= 1;										//compare value
	TCE0.PER		= 1;										//period of 1, generates 24MHz output
	
	PORTE.DIRSET	= PIN0_bm;									//set CLKOUT pin to output
	
	//setup CAN pin interrupts
	PORTC.INTCTRL	= PORT_INT0LVL_HI_gc;
	PORTD.INTCTRL	= PORT_INT0LVL_HI_gc | PORT_INT1LVL_HI_gc;	
	
	PORTD.INT0MASK	= PIN0_bm;						//PORTD0 has can1 interrupt
	PORTD.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	PORTD.INT1MASK	= PIN5_bm;						//PORTD5 has can2 interrupt
	PORTD.PIN5CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	#ifndef DISABLE_CAN3
	PORTC.INT0MASK	= PIN2_bm;						//PORTC2 has can3 interrupt
	PORTC.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	#endif
	
	//buffer checking interrupt
	TCC1.CTRLA		= TC0_CLKSEL_DIV1_gc;			//48M/1/4800 ~ 100usec
	TCC1.PER		= 4800;
	TCC1.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//same priority as can interrupts
	
	//we want to optimize performance, so we're going to time stuff
	//48MHz/48=1us timer, which we just freerun and reset whenever we want to start timing something
	//frame time timer
	TCC0.CTRLA		= TC0_CLKSEL_DIV1_gc;
	TCC0.PER		= 48000;						//48MHz/48000=1ms
	TCC0.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//interrupt on overflow
	
	PORTB.OUTCLR	= (1 << 0);
	
	#ifdef USB_SERIAL
		USB_Init(USB_OPT_RC32MCLKSRC | USB_OPT_BUSEVENT_PRILOW); //USB on medium level to improve current sense timing precision
		CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
	#endif
	
	can_system_init:
	//DV: HAve no senese the if, always call can_init(MCP_OPMOD_NORMAL,1);		
	//Init SPI and CAN interface:
	if(RST.STATUS & RST_WDRF_bm){ //if we come from a watchdog reset, we don't need to setup CAN
		caninit = can_init(MCP_OPMOD_NORMAL, 1); //on second thought, we do
	} else {
		caninit = can_init(MCP_OPMOD_NORMAL, 1);
	}
	
	if(caninit){		
		PORTB.OUTSET |= (1 << 0);					//green LED
	} else {		
		PORTB.OUTSET |= (1 << 1);					//red LED
		_delay_ms(10);
		goto can_system_init;
	}
	
	//Set and enable interrupts with round-robin
	XMEGACLK_CCP_Write((void * ) &PMIC.CTRL, PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);//PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm| PMIC_HILVLEN_bm;
	
	wdt_enable(WDTO_15MS);
	
	sei();
}


int main(void){
	//char * str = "";
	hw_init();

    while(1){
		if(!output_can_to_serial){
			if(sec_interrupt){
				sec_interrupt = 0;
				
				//sample text output every second
				//str = "ms 0000\n";
				//int_to_4digit_nodec(ms_timer, (char *) (str + 3));
				//print(str,8);*/
				
				//Example how to dump one parameter to USB Serial
				char *data = "        \n";
				dtostrf((float)LBC_BatteryCurrentSignal/2,3,1,data);
				fwrite(data, 9, 1, &USBSerialStream);
			}
		}
	}
}




#ifdef USB_SERIAL
	/* Event handler for the LUFA library USB Disconnection event. */
	void EVENT_USB_Device_Disconnect(void){
	}

	void EVENT_USB_Device_Connect(void){
	}

	/* Event handler for the LUFA library USB Configuration Changed event. */
	void EVENT_USB_Device_ConfigurationChanged(void){
		configSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
	}


	/* Event handler for the LUFA library USB Control Request reception event. */
	void EVENT_USB_Device_ControlRequest(void){
		CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
	}


	/* services commands received over the virtual serial port */
	void ProcessCDCCommand(void)
	{
		uint16_t	ReportStringLength = 0;
		char *		ReportString = "";
		int16_t	cmd	 = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	
		if(cmd > -1){
			switch(cmd){
				case 48: //0
				break;
			
				case 0: //reset
				case 90: //'Z'
				_delay_ms(1000);
				CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
				RST.CTRL		= RST_SWRST_bm;
				break;
				case 65: //Show info
				sprintf(ReportString,"GIDS = %u\n",LBC_MY_CHARGE_TIME); ReportStringLength = 8;
				
				break; 
				case 64: //@ - dump all CAN messages to USB
				output_can_to_serial = 1 - output_can_to_serial;
				break;
			
				case 255: //send identity
				ReportString = "MUXSAN CAN bridge - v1.0 Leaf\n"; ReportStringLength = 30;
				break;
			
				default: //when all else fails
				ReportString = "Unrecognized Command:   \n"; ReportStringLength = 25;
				ReportString[22] = cmd;
				break;
			}
			if(ReportStringLength){
				fwrite(ReportString, ReportStringLength, 1, &USBSerialStream);
				Endpoint_ClearIN();
			}
		}
	}
#endif

ISR(TCC0_OVF_vect){	
	wdt_reset();
	ms_timer++;
	if(!can_busy) ProcessCDCCommand();
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
	
	//handle print buffer
	if(print_char_limit <= 64) { print_char_limit = 0; }
	else { print_char_limit -= 64; }	
	
	sec_timer--;	
	
	//fires every second
	if(sec_timer == 0){
		PORTB.OUTCLR = (1 << 1);
		sec_timer = 1000;
		sec_interrupt = 1;
		ten_sec_timer--;
		
		//fires every 10 seconds
		if(ten_sec_timer == 0){
			ten_sec_timer = 10;
		}
	}
}

//fires approx. every 100us
ISR(TCC1_OVF_vect){
	check_can1();
	check_can2();
	check_can3();
}

//can1 interrupt
ISR(PORTD_INT0_vect){
	can_busy = 1;
	can_handler(1);
}

//can2 interrupt
ISR(PORTD_INT1_vect){
	can_busy = 1;
	can_handler(2);
}

//can3 receive interrupt
ISR(PORTC_INT0_vect){
	can_busy = 1;
	can_handler(3);
}

//VCM side of the CAN bus (in Muxsan)
void can_handler(uint8_t can_bus){	
	can_frame_t frame;
	uint8_t remain_charge_condition = 0;
		
	/*char strbuf[] = "1|   |                \n";
	if(can_bus == 2){ strbuf[0] = 50; }
	if(can_bus == 3){ strbuf[0] = 51; }
	*/		
	uint8_t flag = can_read(MCP_REG_CANINTF, can_bus);
		
	if (flag & (MCP_RX0IF | MCP_RX1IF)){		
	
		if(flag & MCP_RX0IF){
			can_read_rx_buf(MCP_RX_0, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX0IF, 0x00, can_bus);
		} else {
			can_read_rx_buf(MCP_RX_1, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX1IF, 0x00, can_bus);
		}		
/****************************************************************************/
/*	0x603 => Appears when car is siwtched on								*/
/*	0x11A => Contains info about the current status of the car (onn/off)	*/
/*	0x1DB => Contains info about:											*/
/*				- Instantaneous current consumption							*/
/*				- Current battery voltage									*/
/*	0x1DC => When charging (chademo), controls the power delivered			*/
/*	0x55B => Report State of Charge, not working on models before 2013		*/
/*	0x8BC => Report info about:												*/
/*				- Charge time												*/
/*				- GIDS														*/
/*	0x1FA => Car charging status											*/			
/****************************************************************************/
		switch(frame.can_id){
			
			case 0x603:
				//Calibration procedure: Each time car starts, we reset power status. 
				calibration_pending = 1; // required to calibrate GIDs on power on event 
				break;
			case 0x11A:
				poweroff = (frame.data[1] >> 5) == 4; // 3 => on, 4 => Off
			break;
			case 0x1DB:
				
				//Check voltage reported by the LBC  (10 bits, all 8bits from frame[2], and 2 bits from frame[3] 0xC0)
				LBC_BatteryVoltageSignal = (uint16_t)((frame.data[2] << 2) | ((frame.data[3] & 0xC0) >> 6));
				 
				//Check current reported by the LBC
				LBC_BatteryCurrentSignal = ((frame.data[0] << 3) | ((frame.data[1]) >> 5)); //Data in 2's comp
				if(LBC_BatteryCurrentSignal & 0x0400) LBC_BatteryCurrentSignal |= 0xf800;
				LBC_BatteryCurrentSignal = (int16_t)(-LBC_BatteryCurrentSignal) + 1;
				
				//Calibration time?
				if (calibration_pending == 1) { // on power on event, calibrate GIDs with voltage
					if(LBC_BatteryVoltageSignal > LBC_MIN_VALID_VOLTS && LBC_BatteryVoltageSignal < LBC_MAX_VALID_VOLTS) { // Check if voltage is in range
						LBC_PowerStatus = ((LBC_BatteryVoltageSignal / 2) - ZERO_WATTS_VOLT)* WATTS_PER_VOLT; //Remember, we are working with 0.5V per LSB
						calibration_pending = 0;
					} else {
						calibration_pending = 1;
					}
				} else {
					if(((LBC_BatteryVoltageSignal > LBC_MIN_VALID_VOLTS) && (LBC_BatteryVoltageSignal < LBC_MAX_VALID_VOLTS)) && ((LBC_BatteryCurrentSignal > LBC_MIN_VALID_AMPS) && (LBC_BatteryCurrentSignal < LBC_MAX_VALID_AMPS) )) { // Check current is on range
						if(power_samples < GIDS_MEASURE_CYLES) { // while cycles are not reached keeps increasing power_sum
							power_sum = power_sum + ((float)(LBC_BatteryVoltageSignal/2.0) * (float)(LBC_BatteryCurrentSignal /2.0)) / 3600; // reads power, and accumulate it. the reading is each 10ms,																		  // each 100 measures is a second and 3600 secs per hour
							power_samples++;
						} else {
							LBC_PowerStatus = LBC_PowerStatus - power_sum/100; // decreases from total available power
							power_samples	= 0;
							power_sum		= 0;
						}
					}
				}
				//To avoid going under 0
				if(LBC_PowerStatus < 0) {
					LBC_PowerStatus = 0;
				}
				calc_crc8(&frame);
			break;
			
			case 0x1DC:
				
				if( charging_state == CHARGING_QUICK_START || charging_state == CHARGING_QUICK) {
					chademo_power_rate = ((frame.data[2] << 6) & 0x3C0) | ((frame.data[3] >> 2) & 0x3F);
					switch(frame.data[3] & 0x03) {
						case 0x00:
						NULL;	//Leave for future configurations.
						break;
						case 0x01:
							if(charging_state == CHARGING_QUICK){ chademo_power_rate = 0x3E58;}
						break;
						case 0x02:
							if(charging_state == CHARGING_QUICK){ chademo_power_rate = 0x3FFF;}
						break;
						case 0x03:
							if(charging_state == CHARGING_QUICK_START){ chademo_power_rate = 0x3C64;}
							if(charging_state == CHARGING_QUICK){ chademo_power_rate = 0x3E58; }
						break;
					}
				}
				
				
				frame.data[2] = (frame.data[2] & 0xF0) | ((chademo_power_rate >> 8) & 0xF);
				frame.data[3] = (uint8_t) (chademo_power_rate & 0xFF);
				calc_crc8(&frame);
				
				if (charging_wait_b4_set_current  > 0) {charging_wait_b4_set_current--;};
				//At charging start, we wait for CURRENT_SET_WAITING_TIME to start demanding current
				if(charging_state == CHARGING_QUICK_START) {
					charging_wait_b4_set_current = CURRENT_SET_WAITING_TIME;
				}	
				break;

			case 0x55B:
				
				//Check what SOC the LBC reports  (10 bits, all 8bits from frame[0], and 2 bits from frame[1] 0xC0)
				//state_of_charge = (frame.data[0] << 2) | ((frame.data[1] & 0xC0) >> 6);  //Further improvement idea, modify this also!	
				LBC_StateOfCharge = (((LBC_BatteryVoltageSignal/2 - ZERO_WATTS_VOLT)) * 32) / 3; //State of charge.
				
				frame.data[0]  = (uint8_t) LBC_StateOfCharge >> 2;
				frame.data[1]  = (uint8_t) ((LBC_StateOfCharge << 6) & 0x03) | (frame.data[1] & 0x3F);
				calc_crc8(&frame);
				break;		
				
			case 0x5BC: //This frame contains: GIDS, temp readings, capacity bars and mux
				//When car powered off, we keep GIDS volts calculation. In this case range will go from 0 to 288GIDS (the original range)
				//This is done to avoid problems with certain chademo chargers. It looks like the chademo check the correlation between 
				//Battery pack volts and GIDS.
				
				//Let's check mode: full or current charge
				if(!poweroff) {
					if((charging_state != CHARGING_SLOW) && (charging_state != CHARGING_QUICK) && (charging_state != CHARGING_QUICK_START)) {
						
						total_gids = (LBC_PowerStatus / UNITY_GIDS);

					} else {
						
						total_gids = (uint16_t) ((((LBC_BatteryVoltageSignal /2) - ZERO_WATTS_VOLT) ) * GIDS_PER_VOLT); //Voltage based estimation 3 GIDS/V. We need it becouse of uint16 limits
						
					}
					
				} else {
						//Fix value to allow ABB chargers to work and has no impact on others Chademo chargers tested.
						total_gids = 200; 
				}
				//Build custom frame.
				frame.data[0] = (uint8_t) (total_gids >> 2);
				frame.data[1] = (uint8_t) ((frame.data[1] & 0x3F) | ((total_gids & 0x03) << 6));
				
				//Set the charging time to fixed value: LBC_MY_CHARGE_TIME when charge condition is TODO TODO
				remain_charge_condition = ((frame.data[5] & 0x03) << 3) | ((frame.data[6] >> 5) & 0x07);
				if(remain_charge_condition == 0x00) { 
					//Quick charge and we must sent modified remaining time
					frame.data[6] = (uint8_t) ((LBC_MY_CHARGE_TIME >> 8) | (frame.data[6] & 0xE0));
					frame.data[7] = (uint8_t) (LBC_MY_CHARGE_TIME & 0xFF);
				}
				
				
			break;
			
			case 0x1F2: 
				//Collect the charging state (Needed for 2013+ Leafs, throws DTC otherwise) [VCM->LBC]
				
				charging_state = frame.data[2];
			
			break;
			default:
			break;
			}
		
		
		//if you enable CAN repeating between bus 1 and 2, we end up here	
		if(repeat_can){
			//you can blacklist certain messages or message contents like this, blocking them from both being forwarded and being displayed
			uint8_t blacklist = 0;
			switch(frame.can_id){				
				case 0x59E: 
					//blacklist = 1;
					break;
				default:
					blacklist = 0;					
					break;
			}
			if(!blacklist){
				if(can_bus == 1){send_can2(frame);} else {send_can1(frame);}
								
				if(output_can_to_serial){
					/*SID_to_str(strbuf + 2, frame.can_id);
					canframe_to_str(strbuf + 6, frame);
					sprintf(strbuf,"%f",LBC_PowerStatus);
					print(strbuf,23);
					*/
					char *data = "     \n";
					dtostrf(LBC_PowerStatus,3,2,data);
					
					fwrite(data, 9, 1, &USBSerialStream);
					
				}
			}
		}		
	}
					
	
	if(flag & 0xA0){
		uint8_t flag2 = can_read(MCP_REG_EFLG, can_bus);
		if(flag2 & 0xC0){
			can_write(MCP_REG_EFLG, 0, can_bus); //reset all errors
			ReportString = "CANX RX OVF\n";
			ReportString[3] = 48 + can_bus;
			print(ReportString,12);
		}
		if(flag2 > 0){ PORTB.OUTSET = (1 << 1); }
		if(flag & 0xE0){ can_bit_modify(MCP_REG_CANINTF, (flag & 0xE0), 0x00, can_bus);	}
	}
	can_busy = 0;
}


void send_can(uint8_t can_bus, can_frame_t frame){
	if(can_bus == 1) send_can1(frame);
	if(can_bus == 2) send_can2(frame);
	if(can_bus == 3) send_can3(frame);
}

void send_can1(can_frame_t frame){	
	//put in the buffer
	memcpy(&tx0_buffer[tx0_buffer_end++], &frame, sizeof(frame));
	
	if(tx0_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx0_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can1();
}



void check_can1(void){
	uint8_t reg;
	
	if(tx0_buffer_end != tx0_buffer_pos){
		//check if TXB0 is free use
		reg = can1_read(MCP_REG_TXB0CTRL);
	
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can1_load_txbuff(0, (can_frame_t *) &tx0_buffer[tx0_buffer_pos++]);
			can1_rts(0);
			if(tx0_buffer_pos == tx0_buffer_end){ //end of buffer, reset
				tx0_buffer_end = 0;
				tx0_buffer_pos = 0;
			}
		}
	}
}

void send_can2(can_frame_t frame){
	//put in the buffer
	memcpy(&tx2_buffer[tx2_buffer_end++], &frame, sizeof(frame));
	
	if(tx2_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx2_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can2();
}

void check_can2(void){
	uint8_t reg;
	
	if(tx2_buffer_end != tx2_buffer_pos){
		//check if TXB0 is free use
		reg = can2_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can2_load_txbuff(0, (can_frame_t *) &tx2_buffer[tx2_buffer_pos++]);
			can2_rts(0);
			if(tx2_buffer_pos == tx2_buffer_end){ //end of buffer, reset
				tx2_buffer_end = 0;
				tx2_buffer_pos = 0;
			}
		}
	}
}

void send_can3(can_frame_t frame){
	//put in the buffer
	memcpy(&tx3_buffer[tx3_buffer_end++], &frame, sizeof(frame));
	
	if(tx3_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx3_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can3();
}

void check_can3(void){
	uint8_t reg;
	
	if(tx3_buffer_end != tx3_buffer_pos){
		//check if TXB0 is free use
		reg = can3_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can3_load_txbuff(0, (can_frame_t *) &tx3_buffer[tx3_buffer_pos++]);
			can3_rts(0);
			if(tx3_buffer_pos == tx3_buffer_end){ //end of buffer, reset
				tx3_buffer_end = 0;
				tx3_buffer_pos = 0;
			}
		}
	}
}

