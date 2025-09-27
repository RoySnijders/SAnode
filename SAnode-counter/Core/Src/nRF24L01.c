/*
 * File:   nRF24L01.c
 * Author: roy
 *
 * Created on April 28, 2014, 10:56 PM
 */

#include "main.h"
#include "nRF24L01.h"

extern SPI_HandleTypeDef hspi1;
//-------------------------------------------------------------
//-- LOWER LEVEL command - send SPI package
//-------------------------------------------------------------
uint8_t NRF24L01_ExecuteCommand(uint8_t instruction, uint8_t * data, uint8_t rw , uint16_t len)
{	uint8_t status=0,i;
	uint8_t rxdata[50];

    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);   	// CSN chipselect =0;
    //write register command
    if (HAL_SPI_Transmit(&hspi1, &instruction, 1, 100)!= HAL_OK)
    	status=99;	//@TODO Handle error to be done later

    if(rw==NRF24L01_READ)	//write data or read data and copy data in same data array
    {	//read
    	if(len!=0)
    	{	if (HAL_SPI_TransmitReceive(&hspi1, data, &(rxdata[0]), len, 100)!= HAL_OK)
				status=99;	//@TODO Handle error to be done later
			for(i=0;i<len;i++)
				*(data+i)=rxdata[i];		//copy RX data in data (only for size len otherwise stack overflow issues
    	}
    }else
    {	//write
    	if (HAL_SPI_Transmit(&hspi1, data, len, 100)!= HAL_OK)
			status=99;	// Handle error to be done later
    }

    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);   	// CSN chipselect =1

    return status;
}


//--------------------------------------------------------------------------------------
// MID LEVEL commands
//--------------------------------------------------------------------------------------
uint8_t NRF24L01_WriteRegister(uint8_t reg, uint8_t * data, uint16_t len)
{	return NRF24L01_ExecuteCommand(NRF24L01_W_REGISTER | (reg & NRF24L01_W_REGISTER_DATA), data,  NRF24L01_WRITE,len);
}

uint8_t NRF24L01_ReadRegister(uint8_t reg, uint8_t * data, uint16_t len)
{	return NRF24L01_ExecuteCommand(reg & NRF24L01_R_REGISTER_DATA, data,  NRF24L01_READ,len);
}


//--------------------------------------------------------------------------------------
// Init the RF module with defauft values
//--------------------------------------------------------------------------------------
uint8_t NRF24L01_Init (uint8_t rx_pw_p0, uint8_t auto_ack)
{	uint8_t data[50];

	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);   // CSN chipselect = high

    if(auto_ack == NRF24L01_VAL_AUTO_ACK_ON)
		data[0] = NRF24L01_EN_AA_ENAA_P0;                           //if ENAA , then Enable auto acknowledgment data pipe 0
	else
		data[0] = NRF24L01_EN_AA_ENAA_NONE;
    NRF24L01_WriteRegister(NRF24L01_EN_AA, data, 1);                //Enable ?Auto Acknowledgment? Function Disable this functionality to be compatible with nRF2401

    data[0] = NRF24L01_EN_RXADDR_DEFAULT_VAL;                       //0x03 = Enable data pipe 0 and Enable data pipe 1
	NRF24L01_WriteRegister(NRF24L01_EN_RXADDR, data, 1);

	data[0] = NRF24L01_SETUP_AW_DEFAULT_VAL;                        //0x03 = RX/TX Address field width '00' - Illegal, '01' - 3 bytes, '10' - 4 bytes, '11' = 5 bytes
	NRF24L01_WriteRegister(NRF24L01_SETUP_AW, data, 1);

	data[0] = NRF24L01_SETUP_RETR_DEFAULT_VAL;                      //0x03 = 3 Auto Retransmit Count
	NRF24L01_WriteRegister(NRF24L01_SETUP_RETR, data, 1);

	data[0] = NRF24L01_RF_CH_DEFAULT_VAL;                           //0x02 = ch2 default
	NRF24L01_WriteRegister(NRF24L01_RF_CH, data, 1);

	data[0] = NRF24L01_RF_SETUP_DEFAULT_VAL;                        //=0x0F = '11' = 0dBm, 250kbs
	NRF24L01_WriteRegister(NRF24L01_RF_SETUP, data, 1);


    //-- RX address P0, Receive address data pipe 0. 5 Bytes maximum length. = E7E7E7E7E7
    data[0] = NRF24L01_RX_ADDR_P0_B0_DEFAULT_VAL;
    data[1] = NRF24L01_RX_ADDR_P0_B1_DEFAULT_VAL;
    data[2] = NRF24L01_RX_ADDR_P0_B2_DEFAULT_VAL;
    data[3] = NRF24L01_RX_ADDR_P0_B3_DEFAULT_VAL;
    data[4] = NRF24L01_RX_ADDR_P0_B4_DEFAULT_VAL;
    NRF24L01_SetRX_Address(data, 5, 0);


    //-- RX address P1, Receive address data pipe 0. 5 Bytes maximum length. = C2C2C2C2C2
	data[0] = NRF24L01_RX_ADDR_P1_B0_DEFAULT_VAL;
    data[1] = NRF24L01_RX_ADDR_P1_B1_DEFAULT_VAL;
    data[2] = NRF24L01_RX_ADDR_P1_B2_DEFAULT_VAL;
    data[3] = NRF24L01_RX_ADDR_P1_B3_DEFAULT_VAL;
    data[4] = NRF24L01_RX_ADDR_P1_B4_DEFAULT_VAL;
	NRF24L01_SetRX_Address(data, 5, 1);

    data[0] = NRF24L01_RX_ADDR_P2;
	NRF24L01_SetRX_Address(data, 1, 2);     //-- RX address P2

	data[0] = NRF24L01_RX_ADDR_P3;
	NRF24L01_SetRX_Address(data, 1, 3);     //-- RX address P3

	data[0] = NRF24L01_RX_ADDR_P4;
	NRF24L01_SetRX_Address(data, 1, 4);     //-- RX address P4

	data[0] = NRF24L01_RX_ADDR_P5;
	NRF24L01_SetRX_Address(data, 1, 5);     //-- RX address P5


    //-- TX address = E7E7E7E7E7
    data[0] = NRF24L01_TX_ADDR_B0_DEFAULT_VAL;
    data[1] = NRF24L01_TX_ADDR_B1_DEFAULT_VAL;
    data[2] = NRF24L01_TX_ADDR_B2_DEFAULT_VAL;
    data[3] = NRF24L01_TX_ADDR_B3_DEFAULT_VAL;
    data[4] = NRF24L01_TX_ADDR_B4_DEFAULT_VAL;
    NRF24L01_SetTX_Address(data, 5);

    //--payload length
    data[0] = rx_pw_p0;                                     ////payload length P0 =...
	NRF24L01_WriteRegister(NRF24L01_RX_PW_P0, data, 1);

	data[0] = NRF24L01_RX_PW_P1_DEFAULT_VAL;                //payload length P1 =0
	NRF24L01_WriteRegister(NRF24L01_RX_PW_P1, data, 1);

	data[0] = NRF24L01_RX_PW_P2_DEFAULT_VAL;                //payload length P2 =0
	NRF24L01_WriteRegister(NRF24L01_RX_PW_P2, data, 1);

	data[0] = NRF24L01_RX_PW_P3_DEFAULT_VAL;                //payload length P3 =0
	NRF24L01_WriteRegister(NRF24L01_RX_PW_P3, data, 1);

	data[0] = NRF24L01_RX_PW_P4_DEFAULT_VAL;                //payload length P4 =0
	NRF24L01_WriteRegister(NRF24L01_RX_PW_P4, data, 1);

	data[0] = NRF24L01_RX_PW_P5_DEFAULT_VAL;                //payload length P5 =0
	NRF24L01_WriteRegister(NRF24L01_RX_PW_P5, data, 1);

	data[0] = 0x0b;											//power up in RX mode
	NRF24L01_WriteRegister(NRF24L01_CONFIG, data, 1);		//clear interrupts STATUS=0x70Clear all interrupt flags


    NRF24L01_FlushRX();										// Flush RX FIFOs
    NRF24L01_FlushTX();										// Flush TX FIFOs

    //check registers
    //if(NRF24L01_ReadStatus()!=0x0e) return 1;		// 0000 1110 - no interrupts , RX FIFO Empty ,TX FIFO not full
    if(NRF24L01_ReadConfig()!=0x0b) return 2;		// 0000 1011 - no interrupts , CRC enabled, Power up,  RX control
    if(NRF24L01_ReadRFSetup()!=0x26) return 3;		// 0010 0110 - RF Data Rate = 250kbps, RF_PWR= 0dBm
    return 0;
}


//--------------------------------------------------------------------------------------
// RF channel scanning
//--------------------------------------------------------------------------------------
uint8_t NRF24L01_Scan(void)
{	uint8_t fullscanretry=0, channel_activity_array[NRF24L01_NUMBER_CHANNELS]= { 0 };
	uint8_t channel, best_channel=0;
	uint8_t rfsetup;

	rfsetup=NRF24L01_ReadRFSetup()& 0xF8; //remove last 3 bits for lowest TX power
	NRF24L01_WriteRFSetup(&rfsetup);

	 for (fullscanretry=0;fullscanretry<255;fullscanretry++)
	 {	// Scan the full range number of times
		// To safely change the channel, it's recommended that you stop the current reception-- Go to Standby (DISABLE CE)

		 for (channel = 0; channel < NRF24L01_NUMBER_CHANNELS; channel++)
	 	 {	NRF24L01_WriteRFChannel(&channel); 					// Set the current channel
			NRF24L01_RXmode();									// Resume normal to RX state
			delay_us(40);										//extra time so RPD is able to read // the RDP is a latched signal

			//Measure the Receive Power Detector-->  number of times
			if (NRF24L01_ReadRPD()>0)
				channel_activity_array[channel]++; 				// Increment activity count if RF detected
			NRF24L01_DISABLE; 									// Stop listening goto to Standby-I state
		}
		 delay_us(10);												//wait 10us before rescan
	 }

	// Find the channel with the least activity
	uint16_t min_activity = 0xFFFF;
	for (uint8_t channel = 0; channel < NRF24L01_NUMBER_CHANNELS; channel++)
	{	if (channel_activity_array[channel] < min_activity)
		{	min_activity = channel_activity_array[channel];
			best_channel=channel;
		}
	}

	//channel=NRF24L01_RF_CH_DEFAULT_VAL;
	//NRF24L01_WriteRFChannel(&best_channel);				//keep on default channel for now
	return best_channel;
}


//-----------------------------------------------------------------------------------------------
//	READ REGISTER
//-----------------------------------------------------------------------------------------------

//Read Status register
//returns the value of the STATUS register
uint8_t NRF24L01_ReadStatus(void)
{   uint8_t data=0;

	NRF24L01_ReadRegister(NRF24L01_STATUS, &data,1);
	return data;
}

//Read RF_SETUP register
//returns the value of the RF_SETUP register
uint8_t NRF24L01_ReadRFSetup(void)
{   uint8_t data;

    NRF24L01_ReadRegister(NRF24L01_RF_SETUP, &data, 1);
    return data;
}

//Read RF_CH register
//returns the value of the RF_CH register
uint8_t NRF24L01_ReadRF_CH(void)
{   uint8_t data;

    NRF24L01_ReadRegister(NRF24L01_RF_CH, &data, 1);
    return data;
}

//Read Config register
//returns the value of the CONFIG register
uint8_t NRF24L01_ReadConfig(void)
{   uint8_t data;
	
	NRF24L01_ReadRegister(NRF24L01_CONFIG, &data, 1);
    return data;
}

//Read all RX addresses
void NRF24L01_ReadRX_Address(uint8_t * address, uint8_t len, uint8_t rxpipenum)
{	if(rxpipenum > 5)
		return;
    NRF24L01_ReadRegister(NRF24L01_RX_ADDR_P0 + rxpipenum, address, len);
}

//Read all TX address
void NRF24L01_ReadTX_Address(uint8_t * address, uint8_t len)
{	NRF24L01_ReadRegister(NRF24L01_TX_ADDR, address, len);
}

//Read Received Power Detector (RPD)
//located in register 09, bit 0, triggers at received power levels above -64dBm that are present in the RF channel you receive on.
// If the received power is less than -64 dBm,RDP = 0.
uint8_t NRF24L01_ReadRPD(void)
{   uint8_t data;

	NRF24L01_ReadRegister(NRF24L01_RPD, &data, 1);
    return (data & 0x01); 							// only bit 0 is needed
}

//Read RX_DR_active bit - Receive Data Ready
uint8_t NRF24L01_IRQ_RX_DR_Active(void)
{	if ((NRF24L01_ReadStatus() & NRF24L01_STATUS_RX_DR)!=0)
        return 1;
    else
        return 0;
}


//-----------------------------------------------------------------------------------------------
//	WRITE REGISTER (with values)
//-----------------------------------------------------------------------------------------------

//Set all RX adresses
void NRF24L01_SetRX_Address(uint8_t * address, uint8_t len, uint8_t rxpipenum)
{	if(rxpipenum > 5)
		return;
	NRF24L01_WriteRegister(NRF24L01_RX_ADDR_P0 + rxpipenum, address, len);
}

//Set all TX adress
void NRF24L01_SetTX_Address(uint8_t * address, uint8_t len)
{	NRF24L01_WriteRegister(NRF24L01_TX_ADDR, address, len);
}

//Set RF register
//returns the value of the register
uint8_t NRF24L01_WriteRFSetup(uint8_t * data)
{   return NRF24L01_WriteRegister(NRF24L01_RF_SETUP, data, 1);
}

//Set RF channel
//returns the value of the register
uint8_t NRF24L01_WriteRFChannel(uint8_t * data)
{   return NRF24L01_WriteRegister(NRF24L01_RF_CH, data, 1);
}




//-----------------------------------------------------------------------------------------------
//	SET STATE / COMMANDS
//-----------------------------------------------------------------------------------------------

//Power up and go to standby, coming out of Power Down
void NRF24L01_PWR_UP(void)
{   uint8_t config=0;
    
    NRF24L01_ReadRegister(NRF24L01_CONFIG, &config, 1);
	config |= NRF24L01_CONFIG_PWR_UP;
	NRF24L01_WriteRegister(NRF24L01_CONFIG, &config, 1);
	delay_us(1500); 											//Chip takes 1.5ms to complete, then it enters the Standby mode I
}

//Set in POWERDOWN State
void NRF24L01_PWR_DOWN(void)
{   uint8_t config=0;
    
    NRF24L01_ReadRegister(NRF24L01_CONFIG, &config, 1);
	config &= (~NRF24L01_CONFIG_PWR_UP);
	NRF24L01_WriteRegister(NRF24L01_CONFIG, &config, 1);
	NRF24L01_DISABLE;
}

// Set in TX mode
void NRF24L01_TXmode(uint8_t input)
{	uint8_t config=input;

	config |= NRF24L01_CONFIG_PWR_UP;				//power up
    config &= ~(NRF24L01_CONFIG_PRIM_RX);           //PRIM_RX = 0 = transmit mode
    
    NRF24L01_WriteRegister(NRF24L01_CONFIG, &config, 1);
    NRF24L01_FlushTX();

    NRF24L01_ENABLE;
	delay_us(130);   							 	//TX mode >= 130us
}

// Set in RX mode
void NRF24L01_RXmode(void)
{	uint8_t config;

	NRF24L01_ReadRegister(NRF24L01_CONFIG, &config, 1);
    config |= NRF24L01_CONFIG_PRIM_RX;           	//set flag
	 
    NRF24L01_WriteRegister(NRF24L01_CONFIG, &config, 1);
    NRF24L01_FlushRX();
    
    NRF24L01_ENABLE;
    delay_us(130);   							 	//RX mode >= 130us
}

// Write TX payload
// when transmitdirect=NRF24L01_VAL_TRANSMITDIRECT then the transmit sequence will directlty follow
//
//@TODO Return Value The function returns a boolean value:
//	true: The data was sent successfully, and an acknowledgment (if enabled) was received.
//	false: 	The data transmission failed, possibly due to:
//			The receiver not being available.
//			Signal interference.
///			Auto-ACK being disabled and the packet being dropped after retries.
//
uint8_t NRF24L01_WriteTX_Payload(uint8_t * data, uint16_t len, uint8_t transmitdirect)
{	uint8_t status,fifo;

    status = NRF24L01_ExecuteCommand(NRF24L01_W_TX_PAYLOAD, data, NRF24L01_WRITE,len);
	if(transmitdirect == NRF24L01_VAL_TRANSMITDIRECT)
		NRF24L01_TX_transmit();         //Once you load the packet, toggle the CE pin to send the packet (keeping it high for at least 10 us).

	status = NRF24L01_ReadRegister(NRF24L01_FIFO_STATUS, &fifo, 1);

//	Transmission Confirmation:
//	The STATUS register is monitored for the following flags:
//	TX_DS (Data Sent): Indicates successful transmission.
//	MAX_RT (Maximum Retries): Indicates that the maximum number of retries was reached without acknowledgment.

	return status;
}

//Transmits the current tx payload
//Transmission Start: Transmission begins when the CE (Chip Enable) pin is toggled high for a short duration (typically >10µs).
void NRF24L01_TX_transmit()
{	NRF24L01_ENABLE;
	delay_us(200);
    NRF24L01_DISABLE;
}

//Flush TX buffer
uint8_t NRF24L01_FlushTX()
{	return NRF24L01_ExecuteCommand(NRF24L01_FLUSH_TX, NULL, NRF24L01_WRITE, 0);
}


// Read RX payload
uint8_t NRF24L01_ReadRX_Payload(uint8_t * data, uint16_t len)
{	uint8_t status;

    //Done before this routine:
    //Receiving packets, CE is held high. Once you have received a packet you MUST bring CE low to disable the receiver, and then you execute the R_RX_PAYLOAD operation
	NRF24L01_DISABLE;   	// CE=low;
	status = NRF24L01_ExecuteCommand(NRF24L01_R_RX_PAYLOAD, data,  NRF24L01_READ, len);
	NRF24L01_ENABLE;   	// CE=high;
	
	return status;
}

//Flush RX payload register
uint8_t NRF24L01_FlushRX()
{	return NRF24L01_ExecuteCommand(NRF24L01_FLUSH_RX, NULL, NRF24L01_WRITE, 0);
}

uint8_t NRF24L01ReadDataReady(uint8_t * data, uint16_t len)
{	uint8_t val;
	uint8_t status;

	//Check via interrupt
	//The RX_DR IRQ is asserted by a new packet arrival event. The procedure for handling this interrupt should be:
	//	1) read payload through SPI
	//	2) clear RX_DR IRQ
	//	3) read FIFO_STATUS to check if there are more payloads available in RX FIFO
	//  4) if there are more data in RX FIFO, repeat from step 1).
	//NRF24L01_ExecuteCommand(NRF24L01_STATUS, &val,  NRF24L01_READ, 1);

	//Check FIFO not empty
	NRF24L01_ExecuteCommand(NRF24L01_FIFO_STATUS, &val,  NRF24L01_READ, 1);

	if((val&NRF24L01_FIFO_STATUS_RX_EMPTY)==0)
	{	//FIFO is not empty , 1: RX FIFO empty.	0: Data in RX FIFO.
		status = NRF24L01_ExecuteCommand(NRF24L01_R_RX_PAYLOAD, data,  NRF24L01_READ, len); //read payload
		NRF24L01_IRQ_ClearRX_DR();	//After reading a payload, you must clear the RX_DR bit in the STATUS register by writing 1 to it.
	}
	return (status);
}


//--------------------------------------------------------------------------------------------------------------------
// Clear interrupts
//--------------------------------------------------------------------------------------------------------------------
//clear all interrupts in the status register
void NRF24L01_IRQ_ClearAll()
{	uint8_t data = NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT;
	NRF24L01_WriteRegister(NRF24L01_STATUS, &data, 1); 
}

//clears only the RX_DR interrupt
void NRF24L01_IRQ_ClearRX_DR()
{	uint8_t data = NRF24L01_STATUS_RX_DR;
	NRF24L01_WriteRegister(NRF24L01_STATUS, &data, 1); 
}

//clears the TX_DS interrupt - - Receive Data Send
void NRF24L01_IRQ_ClearTX_DS()
{	uint8_t data = NRF24L01_STATUS_TX_DS;
	NRF24L01_WriteRegister(NRF24L01_STATUS, &data, 1);  
}

//clears only the MAX_RT interrupt
void NRF24L01_IRQ_ClearMax_RT()
{	uint8_t data = NRF24L01_STATUS_MAX_RT;
	NRF24L01_WriteRegister(NRF24L01_STATUS, &data, 1);  
}




