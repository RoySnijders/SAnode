/*
 * File:   nRF24L01.c
 * Author: roy
 *
 * Created on April 28, 2014, 10:56 PM
 */

#include "main.h"
#include "nRF24L01.h"

extern SPI_HandleTypeDef hspi1;


void NRF24L01_Init (uint8_t rx_pw_p0, uint8_t auto_ack)
{	uint8_t data[50];

	HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);   // CSN chipselect = high
   
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

	data[0] = NRF24L01_RF_SETUP_DEFAULT_VAL;                        //=0x0F = '11' ? 0dBm, ?01? ? 2Mbps
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

    NRF24L01_PWR_UP();                					   //power up with DEFAULT CONFIG setting, IRQ is active (0000)
}



//--------------------------------------------------------------------------------------
uint8_t NRF24L01_WriteRegister(uint8_t reg, uint8_t * data, uint16_t len)
{	return NRF24L01_ExecuteCommand(NRF24L01_W_REGISTER | (reg & NRF24L01_W_REGISTER_DATA), data,  NRF24L01_WRITE,len);
}

uint8_t NRF24L01_ReadRegister(uint8_t reg, uint8_t * data, uint16_t len)
{	return NRF24L01_ExecuteCommand(reg & NRF24L01_R_REGISTER_DATA, data,  NRF24L01_READ,len);
}

//-------------------------------------------------------------
//-- send SPI package
//-------------------------------------------------------------
uint8_t NRF24L01_ExecuteCommand(uint8_t instruction, uint8_t * data, uint8_t rw , uint16_t len)
{	uint8_t status,i;
	uint8_t rxdata[50];

    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);   	// CSN chipselect =0;
    //write register command
    if (HAL_SPI_TransmitReceive(&hspi1, &instruction, &status, 1, 100)!= HAL_OK)
    	status=99;	// Handle error to be done later

    if (HAL_SPI_TransmitReceive(&hspi1, data, &(rxdata[0]), len, 100)!= HAL_OK)
    	status=99;	// Handle error to be done later

    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);   	// CSN chipselect =1

    for(i=0;i<len;i++)
    	*(data+i)=rxdata[i];											//copy RX data in data (only for size len otherwise stack overflow issues

    return status;
}


//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
uint8_t NRF24L01_ReadStatus(void)
{   uint8_t data=0;

    return NRF24L01_ExecuteCommand(NRF24L01_NOP, &data,  NRF24L01_READ,0);
}


//returns the value of the RF_SETUP register
uint8_t NRF24L01_ReadRFSetup(void)
{   uint8_t data;

    NRF24L01_ReadRegister(NRF24L01_RF_SETUP, &data, 1);
    return data;
}

uint8_t NRF24L01_WriteRFSetup(uint8_t * data)
{   return NRF24L01_WriteRegister(NRF24L01_RF_SETUP, data, 1);
}

//returns the value of the CONFIG register
uint8_t NRF24L01_ReadConfig(void)
{   uint8_t data;
	
	NRF24L01_ReadRegister(NRF24L01_CONFIG, &data, 1);
    return data;
}

//power up and go to standby, coming out of Power Down
//-----------------------------------------------------------------------------------------------
void NRF24L01_PWR_UP(void)
{   uint8_t config=0;
    
    NRF24L01_ReadRegister(NRF24L01_CONFIG, &config, 1);
	config |= NRF24L01_CONFIG_PWR_UP;
	NRF24L01_WriteRegister(NRF24L01_CONFIG, &config, 1);
	//delay_us(1500);
}

void NRF24L01_PWR_DOWN(void)
{   uint8_t config=0;
    
    NRF24L01_ReadRegister(NRF24L01_CONFIG, &config, 1);
	config &= (~NRF24L01_CONFIG_PWR_UP);
	NRF24L01_WriteRegister(NRF24L01_CONFIG, &config, 1);
	NRF24L01_DISABLE;
}

void NRF24L01_TXmode(uint8_t input)
{	uint8_t config=input;

	config |= NRF24L01_CONFIG_PWR_UP;				//power up
    config &= ~(NRF24L01_CONFIG_PRIM_RX);           //PRIM_RX = 0 = transmit mode
    
    NRF24L01_WriteRegister(NRF24L01_CONFIG, &config, 1);
    NRF24L01_FlushRX();
    NRF24L01_FlushTX();
    
    HAL_Delay(1);// Can be as low as 150us
    
}

void NRF24L01_RXmode(uint8_t config)
{	config |= NRF24L01_CONFIG_PWR_UP;
    config |= NRF24L01_CONFIG_PRIM_RX;           //set flag
	 
    NRF24L01_WriteRegister(NRF24L01_CONFIG, &config, 1);
    NRF24L01_FlushRX();
    NRF24L01_FlushTX();
    
    HAL_Delay(2);    //powerup >= 1.5ms
}



//-----------------------------------------------------------------------------------------------------------------
uint8_t NRF24L01_WriteTX_Payload(uint8_t * data, uint16_t len, uint8_t transmitdirect)
{	uint8_t status;

    status = NRF24L01_ExecuteCommand(NRF24L01_W_TX_PAYLOAD, data, NRF24L01_WRITE,len);
	if(transmitdirect == NRF24L01_VAL_TRANSMITDIRECT)
		NRF24L01_TX_transmit();         //Once you load the packet, you toggle the CE pin to send the packet on its way (keeping it high for at least 10 us).
	return status;
}


uint8_t NRF24L01_FlushRX()
{	return NRF24L01_ExecuteCommand(NRF24L01_FLUSH_RX, NULL, NRF24L01_WRITE, 0);
}

uint8_t NRF24L01_FlushTX()
{	return NRF24L01_ExecuteCommand(NRF24L01_FLUSH_TX, NULL, NRF24L01_WRITE, 0);
}

//transmits the current tx payload
void NRF24L01_TX_transmit()
{	NRF24L01_ENABLE;
    HAL_Delay(1);
    NRF24L01_DISABLE;
}



uint8_t NRF24L01_ReadRX_Payload(uint8_t * data, uint16_t len)
{	uint8_t status;

    //Done before this routine:
    //Receiving packets, CE is held high. Once you have received a packet you MUST bring CE low to disable the receiver, and then you execute the R_RX_PAYLOAD operation
	NRF24L01_DISABLE;   	// CE=low;
	status = NRF24L01_ExecuteCommand(NRF24L01_R_RX_PAYLOAD, data,  NRF24L01_READ, len);
	//PIN_RF_CE=1; 
	
	return status;
}

uint8_t NRF24L01_ReadRPD(void)
{   uint8_t data;
	
	NRF24L01_ReadRegister(NRF24L01_RPD, &data, 1);
    return (data & 0x01); // only bit 0 is needed
}


//--------------------------------------------------------------------------------------------------------------------


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

//clears only the TX_DS interrupt
void NRF24L01_IRQ_ClearTX_DS()
{	uint8_t data = NRF24L01_STATUS_TX_DS;
	NRF24L01_WriteRegister(NRF24L01_STATUS, &data, 1);  
}

//clears only the MAX_RT interrupt
void NRF24L01_IRQ_ClearMax_RT()
{	uint8_t data = NRF24L01_STATUS_MAX_RT;
	NRF24L01_WriteRegister(NRF24L01_STATUS, &data, 1);  
}

uint8_t NRF24L01_IRQ_RX_DR_Active(void)
{	if ((NRF24L01_ReadStatus() & NRF24L01_STATUS_RX_DR)!=0)
        return 1;
    else
        return 0;
}

void NRF24L01_SetRX_Address(uint8_t * address, uint8_t len, uint8_t rxpipenum)
{	if(rxpipenum > 5)
		return;
	NRF24L01_WriteRegister(NRF24L01_RX_ADDR_P0 + rxpipenum, address, len);
}

void NRF24L01_ReadRX_Address(uint8_t * address, uint8_t len, uint8_t rxpipenum)
{	if(rxpipenum > 5)
		return;
    NRF24L01_ReadRegister(NRF24L01_RX_ADDR_P0 + rxpipenum, address, len);    
}

       
void NRF24L01_SetTX_Address(uint8_t * address, uint8_t len)
{	NRF24L01_WriteRegister(NRF24L01_TX_ADDR, address, len);
}

void NRF24L01_ReadTX_Address(uint8_t * address, uint8_t len)
{	NRF24L01_ReadRegister(NRF24L01_TX_ADDR, address, len);    
}
