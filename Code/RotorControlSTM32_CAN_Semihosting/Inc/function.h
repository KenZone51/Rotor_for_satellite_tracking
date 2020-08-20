#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "type.h"
#include "stm32f072rb.h"

uint32_t nbBytesCanFrame[6] = {8,8,5,6,6,5};
uint32_t numberOfDataFrameReceived = 0;
uint32_t nextRemoteFrameId = 0;
uint32_t nextDataFrameId = 0;
uint32_t passNumberToUpdate = 0;

Pass passes[4] = {0};

/***** CAN *****/
void canFilterConfig(void);
void canInit(void);
void sendRemoteFrame();
int concat(int a, int b);
void canEnableIRQ(void);
void receiveDataFrame(void);

/******************************************************************************/
/*                                                                            */
/*                   Controller Area Network (CAN)                            */
/*                                                                            */
/******************************************************************************/

/******************* Configure the CAN to filter the data frames from the RPI ********************/

void canFilterConfig(){

  printf("********** CAN FILTER CONFIGURATION **********\r\n");

  // Enter filter initialization mode
  CAN->FMR |= CAN_FMR_FINIT; // 0: Active filters mode, 1: Initialization mode for the filters

  // Identifier mode for the filters 0 to 5
  CAN->FM1R |= CAN_FM1R_FBM0 | CAN_FM1R_FBM1 | CAN_FM1R_FBM2 | CAN_FM1R_FBM3 | CAN_FM1R_FBM4 | CAN_FM1R_FBM5;  // 0: Mask mode, 1: Identifier mode

  // Identifier mode for the filters 6 to 11
  CAN->FM1R |= CAN_FM1R_FBM6 | CAN_FM1R_FBM7 | CAN_FM1R_FBM8 | CAN_FM1R_FBM9 | CAN_FM1R_FBM10 | CAN_FM1R_FBM11;  // 0: Mask mode, 1: Identifier mode

  // Single 32-bit scale configuration mode for the filters 0 to 5
  CAN->FS1R |= CAN_FS1R_FSC0 | CAN_FS1R_FSC1 | CAN_FS1R_FSC2 | CAN_FS1R_FSC3 | CAN_FS1R_FSC4 | CAN_FS1R_FSC5; // 0: Dual 16-bit scale configuration, 1: Single 32-bit scale configuration

  // Single 32-bit scale configuration mode for the filters 6 to 11
  CAN->FS1R |= CAN_FS1R_FSC6 | CAN_FS1R_FSC7 | CAN_FS1R_FSC8 | CAN_FS1R_FSC9 | CAN_FS1R_FSC10 | CAN_FS1R_FSC11; // 0: Dual 16-bit scale configuration, 1: Single 32-bit scale configuration

  // The message passing through the filter 0 to 5 will be stored in the FIFO 0
  CAN->FFA1R &= ~CAN_FFA1R_FFA0_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R &= ~CAN_FFA1R_FFA1_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R &= ~CAN_FFA1R_FFA2_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R &= ~CAN_FFA1R_FFA3_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R &= ~CAN_FFA1R_FFA4_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R &= ~CAN_FFA1R_FFA5_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1

  CAN->FFA1R |= CAN_FFA1R_FFA6_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R |= CAN_FFA1R_FFA7_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R |= CAN_FFA1R_FFA8_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R |= CAN_FFA1R_FFA9_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R |= CAN_FFA1R_FFA10_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1
  CAN->FFA1R |= CAN_FFA1R_FFA11_Msk; // 0: Filter assigned to FIFO 0, 1: Filter assigned to FIFO 1

  // Activate filter 0 to 5
  CAN->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 | CAN_FA1R_FACT2 | CAN_FA1R_FACT3 | CAN_FA1R_FACT4 | CAN_FA1R_FACT5;

  // Activate filter 6 to 11
  CAN->FA1R |= CAN_FA1R_FACT6 | CAN_FA1R_FACT7 | CAN_FA1R_FACT8 | CAN_FA1R_FACT9 | CAN_FA1R_FACT10 | CAN_FA1R_FACT11;

  /********** FIFO 0 **********/
  /* Filter bank 0 register 1 and filter bank 0 register 2 are both identifier registers */
  /* The data frames will be accepted only if the ID is between 0x01 and 0x06 */

  // Filter bank 0 register 1
  CAN->sFilterRegister[0].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[0].FR1 |= (0x01 << CAN_F0R1_FB21_Pos); // Identifier is 0x01

  // Filter bank 0 register 2
  CAN->sFilterRegister[0].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[0].FR2 |= (0x01 << CAN_F0R2_FB21_Pos); // Identifier is 0x01

  // Filter bank 1 register 1
  CAN->sFilterRegister[1].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[1].FR1 |= (0x02 << CAN_F0R1_FB21_Pos); // Identifier is 0x02

  // Filter bank 1 register 2
  CAN->sFilterRegister[1].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[1].FR2 |= (0x02 << CAN_F0R2_FB21_Pos); // Identifier is 0x02

  // Filter bank 2 register 1
  CAN->sFilterRegister[2].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[2].FR1 |= (0x03 << CAN_F0R1_FB21_Pos); // Identifier is 0x03

  // Filter bank 2 register 2
  CAN->sFilterRegister[2].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[2].FR2 |= (0x03 << CAN_F0R2_FB21_Pos); // Identifier is 0x03

  // Filter bank 3 register 1
  CAN->sFilterRegister[3].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[3].FR1 |= (0x04 << CAN_F0R1_FB21_Pos); // Identifier is 0x04

  // Filter bank 3 register 2
  CAN->sFilterRegister[3].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[3].FR2 |= (0x04 << CAN_F0R2_FB21_Pos); // Identifier is 0x04

  // Filter bank 4 register 1
  CAN->sFilterRegister[4].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[4].FR1 |= (0x05 << CAN_F0R1_FB21_Pos); // Identifier is 0x05

  // Filter bank 4 register 2
  CAN->sFilterRegister[4].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[4].FR2 |= (0x05 << CAN_F0R2_FB21_Pos); // Identifier is 0x05

  // Filter bank 5 register 1
  CAN->sFilterRegister[5].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[5].FR1 |= (0x06 << CAN_F0R1_FB21_Pos); // Identifier is 0x06

  // Filter bank 5 register 2
  CAN->sFilterRegister[5].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[5].FR2 |= (0x06 << CAN_F0R2_FB21_Pos); // Identifier is 0x06

  /********** FIFO 1 **********/

  /* Filter bank 0 register 1 and filter bank 0 register 2 are both identifier registers */
  /* The data frames will be accepted only if the ID is between 0x01 and 0x06 */

  // Filter bank 6 register 1
  CAN->sFilterRegister[6].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[6].FR1 |= (0x01 << CAN_F0R1_FB21_Pos); // Identifier is 0x01

  // Filter bank 6 register 2
  CAN->sFilterRegister[6].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[6].FR2 |= (0x01 << CAN_F0R2_FB21_Pos); // Identifier is 0x01

  // Filter bank 7 register 1
  CAN->sFilterRegister[7].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[7].FR1 |= (0x02 << CAN_F0R1_FB21_Pos); // Identifier is 0x02

  // Filter bank 7 register 2
  CAN->sFilterRegister[7].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[7].FR2 |= (0x02 << CAN_F0R2_FB21_Pos); // Identifier is 0x02

  // Filter bank 8 register 1
  CAN->sFilterRegister[8].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[8].FR1 |= (0x03 << CAN_F0R1_FB21_Pos); // Identifier is 0x03

  // Filter bank 8 register 2
  CAN->sFilterRegister[8].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[8].FR2 |= (0x03 << CAN_F0R2_FB21_Pos); // Identifier is 0x03

  // Filter bank 9 register 1
  CAN->sFilterRegister[9].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[9].FR1 |= (0x04 << CAN_F0R1_FB21_Pos); // Identifier is 0x04

  // Filter bank 9 register 2
  CAN->sFilterRegister[9].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[9].FR2 |= (0x04 << CAN_F0R2_FB21_Pos); // Identifier is 0x04

  // Filter bank 10 register 1
  CAN->sFilterRegister[10].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[10].FR1 |= (0x05 << CAN_F0R1_FB21_Pos); // Identifier is 0x05

  // Filter bank 10 register 2
  CAN->sFilterRegister[10].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[10].FR2 |= (0x05 << CAN_F0R2_FB21_Pos); // Identifier is 0x05

  // Filter bank 11 register 1
  CAN->sFilterRegister[11].FR1 &= ~(0xFFFFFFFF << CAN_F0R1_FB0_Pos); // Clear
  CAN->sFilterRegister[11].FR1 |= (0x06 << CAN_F0R1_FB21_Pos); // Identifier is 0x06

  // Filter bank 5 register 2
  CAN->sFilterRegister[11].FR2 &= ~(0xFFFFFFFF << CAN_F0R2_FB0_Pos); // Clear
  CAN->sFilterRegister[11].FR2 |= (0x06 << CAN_F0R2_FB21_Pos); // Identifier is 0x06

  // Leave filter initialization mode
  CAN->FMR &=~ CAN_FMR_FINIT;

  printf("CAN FILTER CONFIGURATION OK\r\n");

}

/******************* Initialize the CAN peripheral ********************/

void canInit(){

  printf("********** CAN INITIALIZATION **********\r\n");

  // Enable the clock for the GPIOA at 8 Mhz
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // Enable the clock for the CAN at 8 Mhz
  RCC->APB1ENR |= RCC_APB1ENR_CANEN;

  /* PA12 is the TX pin and PA11 is the RX pin */

  // Activate PA12 mode to alternate function
  GPIOA->MODER &= ~GPIO_MODER_MODER12_Msk; // Clear
  GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER12_Pos); // 00: Input mode (reset state), 01: General purpose output mode, 10 : Alternate function mode, 11: Analog mode

  // Configure PA12 as High-Speed Output
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR12_Msk;
  GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEEDR12_Pos); // x0: Low speed, 01: Medium speed, 11: High speed

  // Disable PA12 Pull-up/Pull-down
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR12_Msk; // 00: No pull-up, pull-down, 01: Pull-up, 10: Pull-down, 11: Reserved

  // Alternative mapping for PA12
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL12_Msk; // Clear
  GPIOA->AFR[1] |= (0x04 << GPIO_AFRH_AFSEL12_Pos); // Alternate function as TX pin for the CAN

  //Activate PA11 mode to alternate function
  GPIOA->MODER &= ~GPIO_MODER_MODER11_Msk; // Clear
  GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER11_Pos); // 00: Input mode (reset state), 01: General purpose output mode, 10 : Alternate function mode, 11: Analog mode

  // Configure PA11 as High-Speed Output
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR11_Msk; // Clear
  GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEEDR11_Pos); // x0: Low speed, 01: Medium speed, 11: High speed

  // Disable PA11 Pull-up/Pull-down
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR11_Msk; // 00: No pull-up, pull-down, 01: Pull-up, 10: Pull-down, 11: Reserved

  // Alternative mapping for PA11
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk; // Clear
  GPIOA->AFR[1] |= (0x04 << GPIO_AFRH_AFSEL11_Pos); // Alternate function as RX pin for the CAN

  // Exit freeze mode while using DEBUG
  CAN->MCR &= ~CAN_MCR_DBF_Msk;

  // Initialization request for the CAN
  CAN->MCR |= CAN_MCR_INRQ;

  while ( (CAN->MSR & CAN_MSR_INAK) != CAN_MSR_INAK){

    printf("Wait until INAK bit is set to enter in can initialization mode\r\n");
  } // Wait until we reach initialization mode

  // Exit from sleep mode
  CAN->MCR &= ~CAN_MCR_SLEEP;

  // Time triggered mode disable
  CAN->MCR &= ~CAN_MCR_TTCM;

  // Automatic retransmission disable
  CAN->MCR |= CAN_MCR_NART;

  // Receive fifo locked disable
  CAN->MCR &= ~CAN_MCR_RFLM;

  // Auto wake up disable
  CAN->MCR &= ~CAN_MCR_AWUM;

  // Transmit fifo priority disable
  CAN->MCR &= ~CAN_MCR_TXFP;

  // Normal mode
  CAN->BTR &= ~CAN_BTR_LBKM_Msk; // 0: Loop Back Mode disabled, 1: Loop Back Mode enabled
  CAN->BTR &= ~CAN_BTR_SILM_Msk; // 0: Normal operation, 1: Silent Mode

  /* The bit timing are given from this website : http://www.bittiming.can-wiki.info */

  // Synchronization segment of 1 TQ
  CAN->BTR &= ~CAN_BTR_SJW_Msk;
  CAN->BTR |= ( (0x01 - 0x01) << CAN_BTR_SJW_Pos );

  // Prescaler of 1
  CAN->BTR &= ~CAN_BTR_BRP_Msk;
  CAN->BTR |= ( (0x01 - 0x01) << CAN_BTR_BRP_Pos );

  // Segment 2 of 2 TQ
  CAN->BTR &= ~CAN_BTR_TS2_Msk; // Clear
  CAN->BTR |= ( (0x02 - 0x01) << CAN_BTR_TS2_Pos );

  // Segment 1 of 13 TQ
  CAN->BTR &= ~CAN_BTR_TS1_Msk; // Clear
  CAN->BTR |= ( (0x0D - 0x01) << CAN_BTR_TS1_Pos );

  canFilterConfig();

  // Configure the sources of interruption
  CAN->IER |= CAN_IER_TMEIE; // Interruption on completed transmission request (ok, arbitration loss, abort and error)
  CAN->IER |= CAN_IER_FMPIE0; // Interruption on message pending into the fifo 0
  CAN->IER |= CAN_IER_FMPIE1; // Interruption on message pending into the fifo 1
  CAN->IER |= CAN_IER_FOVIE0; // Interruption on message overrun into the fifo 0
  CAN->IER |= CAN_IER_FOVIE1; // Interruption on message overrun into the fifo 1
  CAN->IER |= CAN_IER_ERRIE; // Interruption on error
  CAN->IER |= CAN_IER_LECIE; // Interruption on last error code


  // Request exit from initialization mode
  CAN->MCR &= ~CAN_MCR_INRQ;

  while ((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK){
    printf("Wait until INAK bit is clear to exit from can initialization mode\r\n");
  } // Wait until we quit initialization mode

  printf("CAN INITIALIZATION OK\r\n");

}

/******************* Send remote frames to the RPI in order to get information about the 4 next passes ********************/

void sendRemoteFrame(){

	nextRemoteFrameId = (numberOfDataFrameReceived >= 6) ? ( ( numberOfDataFrameReceived % 6 ) + 1 ) : (numberOfDataFrameReceived + 1);

	printf("********** SEND REMOTE FRAME WITH ID N°%ld **********\r\n", nextRemoteFrameId);

	uint32_t emptyMailbox;

	// Check that all the Tx mailboxes are not full
	if ( ( (CAN->TSR & CAN_TSR_TME0) != 0U ) || ( (CAN->TSR & CAN_TSR_TME1) != 0U ) || ( (CAN->TSR & CAN_TSR_TME2) != 0U) ){

		// Select an empty transmit mailbox
		emptyMailbox = (CAN->TSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

		printf("The transmit mailbox selected for the ID n°%ld is mailbox %ld\r\n", nextRemoteFrameId, emptyMailbox);

		// IDE bit set to 0 because we use format CAN Strandard format (2.0A)
		CAN->sTxMailBox[emptyMailbox].TIR &= ~(0x01 << 2); // Set bit 2 to 0

		// RTR bit set to 1 because we send a request frame
		CAN->sTxMailBox[emptyMailbox].TIR |= (0x01 << 1); // Set bit 1 to 1

		// Set the identifier field according to the parameter "nextRemoteFrameId"
		CAN->sTxMailBox[emptyMailbox].TIR &= ~(0x00003FFFF << 3); // Clear bits 3 to 20
		CAN->sTxMailBox[emptyMailbox].TIR &= ~(0x000007FF << 21); // Clear bits 21 to 31
		CAN->sTxMailBox[emptyMailbox].TIR = ( (uint32_t) nextRemoteFrameId << 21);

		// Size of the data frame according to the parameter "nbBytesCanFrame"
		CAN->sTxMailBox[emptyMailbox].TDTR &= ~(0xFFFFFFFF << 0); // Clear bits 0 to 3
		CAN->sTxMailBox[emptyMailbox].TDTR |= ( ( (uint32_t) nbBytesCanFrame[nextRemoteFrameId - 1] ) << 0);

		// Ask for transmission request : TXRQ = 1
		CAN->sTxMailBox[emptyMailbox].TIR |= CAN_TI0R_TXRQ;

	}

	else{

		printf("All the mailbox are full\r\n");
	}

}

/*******************  Concatenate to integer into one  ********************/

int concat(int a, int b){

	char s1[20];
	char s2[20];

    // Convert both the integers to string
    sprintf(s1, "%d", a);
    sprintf(s2, "%d", b);

    // Concatenate both strings
    strcat(s1, s2);

    // Convert the concatenated string
    // to integer
    int c = atoi(s1);

    // return the formed integer
    return c;
}

/******************* Receive data frames from the RPI in order to get information about the 4 next passes ********************/

void receiveDataFrame(){

	passNumberToUpdate = (numberOfDataFrameReceived < 6) ? 0 : ( (numberOfDataFrameReceived < 12) ? 1 : ( (numberOfDataFrameReceived < 18) ? 2 : 3) );
	nextDataFrameId = (numberOfDataFrameReceived >= 6) ? ( ( numberOfDataFrameReceived % 6 ) + 1 ) : (numberOfDataFrameReceived + 1);

	// Get the fifo number where the data frame is stored
	uint32_t fifoNumber = ((CAN->RF0R & CAN_RF0R_FMP0) != 0U) ? 0 : 1;
	printf("The data frame was store into the fifo %ld \r\n", fifoNumber);

	// Get the id of the received data frame
	uint32_t receivedDataFrameId = (CAN_RI0R_STID & CAN->sFIFOMailBox[fifoNumber].RIR) >> CAN_TI0R_STID_Pos;
	printf("The identifier received for the data frame is %ld \r\n", receivedDataFrameId);

	// Check if the identifier from the received data frame and the supposed next id to receive are the same
	if(receivedDataFrameId == nextDataFrameId){

		// Get the size of the received data frame
	    uint32_t receivedDataFrameDlc = (CAN_RDT0R_DLC & CAN->sFIFOMailBox[fifoNumber].RDTR) >> CAN_RDT0R_DLC_Pos;
	    printf("The size from the received data frame is %ld bytes \r\n",receivedDataFrameDlc);

	    // Check if the data lenght received and the supposed next data lenght to receive are same
	    if(receivedDataFrameDlc == (nbBytesCanFrame[nextDataFrameId - 1])){

	    	printf("The received data frame has the right size \r\n");

	    	printf("********** RECEIVE DATA FRAME WITH ID N°%ld FOR PASS N°%ld **********\r\n", nextDataFrameId, (passNumberToUpdate + 1));

	    	switch (nextDataFrameId){

	    		uint32_t hundreds, milli, minuteUnit, minuteTeens, second;

	    		// Get the AOS date
	    		case 1:

	    			passes[passNumberToUpdate].passTime.year =  concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA0 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos) ) - '0' ), (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA1 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos) ) - '0' ) ); // Year
	    			printf("The AOS date year is : %ld\r\n", passes[passNumberToUpdate].passTime.year);

	    			passes[passNumberToUpdate].passTime.month = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA3 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA3_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA4 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA4_Pos) ) - '0' ) ); // Month
	    			printf("The AOS date month is : %ld\r\n", passes[passNumberToUpdate].passTime.month);

	    			passes[passNumberToUpdate].passTime.day = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA6 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA6_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA7 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA7_Pos) ) - '0' ) ); // Day
	    			printf("The AOS date day is : %ld\r\n", passes[passNumberToUpdate].passTime.day);

	    			break;

	    		// Get the AOS time
	    		case 2:

	    			passes[passNumberToUpdate].passTime.hour =  concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA0 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos) ) - '0' ), (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA1 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos) ) - '0' ) ); // Year
	    			printf("The AOS time hour is : %ld\r\n", passes[passNumberToUpdate].passTime.hour);
	    			passes[passNumberToUpdate].passTime.minute = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA3 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA3_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA4 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA4_Pos) ) - '0' ) ); // Month
	    			printf("The AOS time minute is : %ld\r\n", passes[passNumberToUpdate].passTime.minute);
	    			passes[passNumberToUpdate].passTime.second = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA6 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA6_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA7 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA7_Pos) ) - '0' ) ); // Day
	    			printf("The AOS time second is : %ld\r\n", passes[passNumberToUpdate].passTime.second);
	    			break;

	    		// Get the transit time
	    		case 3:
	    			minuteUnit = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA0 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA1 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos) ) - '0' ) );
	    			printf("The transit time minute unit is : %ld\r\n", minuteUnit);
	    			minuteTeens = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA3 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA3_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA4 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA4_Pos) ) - '0' ) );
	    			printf("The transit time minute teens is : %ld\r\n", minuteTeens);
	    			passes[passNumberToUpdate].transitTimeMinute = floorf( ( minuteUnit + (minuteTeens /100.f) ) * 100000 ) / 100000;
	    			printf("The total transit time minute is : %f\r\n", passes[passNumberToUpdate].transitTimeMinute);

	    			second = ( (float) (passes[passNumberToUpdate].transitTimeMinute) ) * 60.0 ;
	    			passes[passNumberToUpdate].transitTimeSecond = floorf( second * 100000 ) / 100000;
	    			printf("The total transit time second is : %f\r\n", passes[passNumberToUpdate].transitTimeSecond);
	    			break;

	    		// Get the start point AZ
	    		case 4:

	    			hundreds = concat ( concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA0 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA1 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos) ) - '0' ) ), (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA2 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA2_Pos) ) - '0' ) );
	    			printf("The hundreds from the start point are : %ld\r\n", hundreds);
	    			milli = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA4 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA4_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA5 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA5_Pos) ) - '0' ) );
	    			printf("The milli from the start point are : %ld\r\n", milli);
	    			passes[passNumberToUpdate].startPoint.azCoordinate = floorf( ( hundreds + (milli /100.f) ) * 100000 ) / 100000;
	    			printf("The startPoint.azCoordinate is : %f\r\n", passes[passNumberToUpdate].startPoint.azCoordinate);
	    			passes[passNumberToUpdate].startPoint.elCoordinate = floorf( 0 * 100000 ) / 100000;
	    			printf("The startPoint.elCoordinate is : %f\r\n", passes[passNumberToUpdate].startPoint.elCoordinate);
	    			break;

	    		// Get the end point AZ
	    		case 5:

	    			hundreds = concat ( concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA0 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA1 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos) ) - '0' ) ), (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA2 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA2_Pos) ) - '0' ) );
	    			printf("The hundreds from the end point are : %ld\r\n", hundreds);
	    			milli = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA4 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA4_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA5 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA5_Pos) ) - '0' ) );
	    			printf("The milli from the end point are : %ld\r\n", milli);
	    			passes[passNumberToUpdate].endPoint.azCoordinate = floorf( ( hundreds + (milli /100.f) ) * 100000 ) / 100000;
	    			printf("The endPoint.azCoordinate is : %f\n", passes[passNumberToUpdate].endPoint.azCoordinate);
	    			passes[passNumberToUpdate].endPoint.elCoordinate = floorf( 0 * 100000 ) / 100000;
	    			printf("The endPoint.elCoordinate is : %f\n", passes[passNumberToUpdate].endPoint.elCoordinate);
	    			break;

	    		// Get the middle point EL
	    		case 6:

	    			hundreds = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA0 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA1 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos) ) - '0' ) );
	    			printf("The hundreds from the peak point are : %ld\r\n", hundreds);
	    			milli  = concat( (uint32_t) ( ( (uint8_t) ( (CAN_RDL0R_DATA3 & CAN->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA3_Pos) ) - '0' ) , (uint32_t) ( ( (uint8_t) ( (CAN_RDH0R_DATA4 & CAN->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA4_Pos) ) - '0' ) );
	    			printf("The milli from the peak point are : %ld\r\n", milli);
	    			passes[passNumberToUpdate].peakPoint.azCoordinate = floorf( fabsf( ( (passes[passNumberToUpdate].endPoint.azCoordinate) + (passes[passNumberToUpdate].startPoint.azCoordinate) ) / 2.0 ) * 100000 ) / 100000;
	    			printf("The peakPoint.azCoordinate is : %f\r\n", passes[passNumberToUpdate].peakPoint.azCoordinate);
	    			passes[passNumberToUpdate].peakPoint.elCoordinate = floorf( ( hundreds + (milli /100.f) ) * 100000 ) / 100000;
	    			printf("The peakPoint.elCoordinate is : %f\r\n", passes[passNumberToUpdate].peakPoint.elCoordinate);

	    			// Initialize the average speed for the pass in AZ and in EL
	    			passes[passNumberToUpdate].averageSpeedAz = floorf( ( fabsf( (passes[passNumberToUpdate].endPoint.azCoordinate - passes[passNumberToUpdate].startPoint.azCoordinate) ) / ( (float) (passes[passNumberToUpdate].transitTimeMinute) ) ) * 100000 ) / 100000;
	    			printf("The pass.averageSpeedAz is : %f\r\n", passes[passNumberToUpdate].averageSpeedAz);
	    			passes[passNumberToUpdate].averageSpeedEl = floorf( ( (passes[passNumberToUpdate].peakPoint.elCoordinate * 2.0) / ( (float) (passes[passNumberToUpdate].transitTimeMinute) ) ) * 100000 ) / 100000;
	    			printf("The pass.averageSpeedEl is : %f\r\n", passes[passNumberToUpdate].averageSpeedEl);

	    			// Initialize the time of the tracking to 0
	    			passes[passNumberToUpdate].tracking.currentTimeSecond = floorf( 0 * 100000 ) / 100000;
	    			printf("The pass.tracking.currentTimeSecond is : %f\r\n", passes[passNumberToUpdate].tracking.currentTimeSecond);
	    			passes[passNumberToUpdate].tracking.currentTimeMinute = floorf( 0 * 100000 ) / 100000;
	    			printf("The pass.tracking.currentTimeMinute is : %f\r\n", passes[passNumberToUpdate].tracking.currentTimeMinute);

	    			// Initialize the current speed of the tracking to 0
	    			passes[passNumberToUpdate].tracking.currentSpeedAz = floorf( 0 * 100000 ) / 10000;
	    			printf("The pass.tracking.currentSpeedAz is : %f\r\n", passes[passNumberToUpdate].tracking.currentSpeedAz);
	    			passes[passNumberToUpdate].tracking.currentSpeedEl = floorf( 0 * 100000 ) / 100000;
	    			printf("The pass.tracking.currentSpeedEl is : %f\r\n", passes[passNumberToUpdate].tracking.currentSpeedAz);

	    			// The maximum speed of the tracking in AZ and in EL is two times the value of the average speed
	    			passes[passNumberToUpdate].tracking.maximumSpeedAz = floorf( ( 2 * (passes[passNumberToUpdate].averageSpeedAz) ) * 100000 ) / 100000;
	    			printf("The pass.tracking.maximumSpeedAz is : %f\r\n", passes[passNumberToUpdate].tracking.maximumSpeedAz);
	    			passes[passNumberToUpdate].tracking.maximumSpeedEl = floorf( ( 2 * (passes[passNumberToUpdate].averageSpeedEl) ) * 100000 )/ 100000;
	    			printf("The pass.tracking.maximumSpeedEl is : %f\r\n", passes[passNumberToUpdate].tracking.maximumSpeedEl);

	    			// Set the speed of the motors at 12V in °/min
	    			passes[passNumberToUpdate].tracking.maximumMotorSpeedAz = 131; // 80 rpm at 12V ( 131 / 360 * 220 = 80 )
	    			passes[passNumberToUpdate].tracking.maximumMotorSpeedEl = 16.5; // 10 rpm at 12V ( 16.5 / 360 * 220 = 10 )

	    			break;

	    		}

	    	numberOfDataFrameReceived++;

	    }

	    else{

	    	printf("The size of the received data frame is not the right one \r\n");
	    }
	}

	else {

		printf("The id of the received data frame is not the one we are looking for \r\n");

	}

	// Release the FIFO
	(fifoNumber == 0) ? (CAN->RF0R |= CAN_RF0R_RFOM0) : (CAN->RF1R |= CAN_RF1R_RFOM1);

}

void canEnableIRQ(){

	NVIC_SetPriority(CEC_CAN_IRQn, 1); // Set interruption priority level to 1
	NVIC_EnableIRQ(CEC_CAN_IRQn); // Enable interruption for the NVIC
}

void CEC_CAN_IRQHandler(void){

	uint32_t interrupts = CAN->IER;
	uint32_t msrflags = CAN->MSR;
	uint32_t tsrflags = CAN->TSR;
	uint32_t rf0rflags = CAN->RF0R;
	uint32_t rf1rflags = CAN->RF1R;
	uint32_t esrflags = CAN->ESR;

	/********** TRANSMIT INTERRUPT **********/
	if ((interrupts & CAN_IER_TMEIE) != 0U){

    	// Transmit interrupt mailbox 0
		if ((tsrflags & CAN_TSR_RQCP0) != 0U) {

    		// Clear RQCP0, TXOK0, ALST0 and TERR0 bits
    		CAN->TSR &= ~CAN_TSR_RQCP0_Msk;
    		CAN->TSR &= ~CAN_TSR_TXOK0_Msk;
    		CAN->TSR &= ~CAN_TSR_ALST0_Msk;
    		CAN->TSR &= ~CAN_TSR_TERR0_Msk;

      		// Transmission succeed
    		if ((tsrflags & CAN_TSR_TXOK0) != 0U){

        		printf("Transmission succeed from mailbox 0 ! \r\n");

    		}

    		// Transmission failed due to arbitration lost
      		if ((tsrflags & CAN_TSR_ALST0) != 0U){

          		printf("Transmission fail due to arbitration lost from mailbox 0 ! \r\n");
          		printf("Send new remote frame ! \r\n");
          		sendRemoteFrame();
        	}

        	// Transmission failed due error
        	if ((tsrflags & CAN_TSR_TERR0) != 0U){

          		printf("Transmission fail due to error from mailbox 0 ! \r\n");
          		printf("Send new remote frame ! \r\n");
          		sendRemoteFrame();

        	}

        	// Transmission aborted
        	if ((tsrflags & CAN_TSR_ABRQ0) != 0U) {

        		printf("Transmission aborted from mailbox 0 ! \r\n");
        		printf("Send new remote frame ! \r\n");
        		sendRemoteFrame();

        	}
      	}

    	// Transmit interrupt mailbox 1
    	if ((tsrflags & CAN_TSR_RQCP1) != 0U) {

    		// Clear RQCP0, TXOK0, ALST0 and TERR0 bits
    		CAN->TSR &= ~CAN_TSR_RQCP1_Msk;
    		CAN->TSR &= ~CAN_TSR_TXOK1_Msk;
    		CAN->TSR &= ~CAN_TSR_ALST1_Msk;
    		CAN->TSR &= ~CAN_TSR_TERR1_Msk;

    		// Transmission succeed
    	    if ((tsrflags & CAN_TSR_TXOK1) != 0U){

    	        printf("Transmission succeed from mailbox 1 ! \r\n");

    	    }

    	    // Transmission failed due to arbitration lost
    	    if ((tsrflags & CAN_TSR_ALST1) != 0U){

    	    	printf("Transmission fail due to arbitration lost from mailbox 1 ! \r\n");
    	    	printf("Send new remote frame ! \r\n");
    	    	sendRemoteFrame();
    	    }

    	    // Transmission failed due error
    	    if ((tsrflags & CAN_TSR_TERR1) != 0U){

    	    	printf("Transmission fail due to error from mailbox 1 ! \r\n");
    	    	printf("Send new remote frame ! \r\n");
    	    	sendRemoteFrame();
    	    }

    	    // Transmission aborted
    	    if ((tsrflags & CAN_TSR_ABRQ1) != 0U) {

    	        printf("Transmission aborted from mailbox 1 !\r\n");
    	        printf("Send new remote frame ! \r\n");
    	        sendRemoteFrame();
    	    }
    	}

    	// Transmit interrupt mailbox 2
    	if ((tsrflags & CAN_TSR_RQCP2) != 0U) {

    		// Clear RQCP2, TXOK2, ALST2 and TERR2 bits
    		CAN->TSR &= ~CAN_TSR_RQCP2_Msk;
    	    CAN->TSR &= ~CAN_TSR_TXOK2_Msk;
    	    CAN->TSR &= ~CAN_TSR_ALST2_Msk;
    	    CAN->TSR &= ~CAN_TSR_TERR2_Msk;

    	    // Transmission succeed
    	    if ((tsrflags & CAN_TSR_TXOK2) != 0U){

    	    	printf("Transmission succeed from mailbox 2 !\r\n");

    	    }

    	    // Transmission failed due to arbitration lost
    	    if ((tsrflags & CAN_TSR_ALST2) != 0U){

    	    	printf("Transmission fail due to arbitration lost from mailbox 2 !\r\n");
    	    	printf("Send new remote frame ! \r\n");
    	    	sendRemoteFrame();
    	    }

    	    // Transmission failed due error
    	    if ((tsrflags & CAN_TSR_TERR2) != 0U){

    	    	printf("Transmission fail due to error from mailbox 2 !\r\n");
    	    	printf("Send new remote frame ! \r\n");
    	    	sendRemoteFrame();
    	    }

    	    // Transmission aborted
    	    if ((tsrflags & CAN_TSR_ABRQ2) != 0U) {

    	    	printf("Transmission aborted from mailbox 2 !\r\n");
    	    	printf("Send new remote frame ! \r\n");
    	    	sendRemoteFrame();

    	    }
    	}
	}

	/********** FIFO 0 INTERRUPT **********/

	// Fifo 0 overrun
  	if ((interrupts & CAN_IER_FOVIE0) != 0U){

    	if ((rf0rflags & CAN_RF0R_FOVR0) != 0U) {

      		printf("Fifo 0 overrun ! \r\n");
      		printf("Send new remote frame ! \r\n");
      		sendRemoteFrame();

      		// Clear fifo 0 overrun flag
      		CAN->RF0R &= ~CAN_RF0R_FOVR0_Msk;
    	}
  	}

  	// Fifo 0 full
  	if ((interrupts & CAN_IER_FFIE0) != 0U)	{

    	if ((rf0rflags & CAN_RF0R_FULL0) != 0U)	{

      		printf("Fifo 0 full ! \r\n");

      		// Clear fifo 0 full flag
      		CAN->RF0R &= ~CAN_RF0R_FULL0_Msk;
    	}
  	}

  	// Fifo 0 message pending
  	if ((interrupts & CAN_IER_FMPIE0) != 0U) {

    	// Check if message is still pending
    	if ((CAN->RF0R & CAN_RF0R_FMP0) != 0U) {

    		printf("Fifo 0 message pending ! \r\n");
    		receiveDataFrame();

    		// Ask for informations about the four next passes
    		if (numberOfDataFrameReceived < 24) {
    			sendRemoteFrame();
    		}

    		// Disable CAN IRQ if we already have the informations about the four next passes
    		else {

    			printf("Disable CAN IRQ because we have the informations about the 4 next passes ! \r\n");

    			for(int i = 0; i < 4; i++){

    				printf("***** PASS %d ***** \r\n", i+1);

    				printf("%ld/%ld/%ld\r\n", passes[i].passTime.year, passes[i].passTime.month,passes[i].passTime.day);
    				printf("%ld:%ld:%ld\r\n",passes[i].passTime.hour, passes[i].passTime.minute,passes[i].passTime.second);
    				printf("%f\r\n", passes[i].transitTimeMinute);
    				printf("%f\r\n", passes[i].startPoint.azCoordinate);
    				printf("%f\r\n", passes[i].endPoint.azCoordinate);
    				printf("%f\r\n", passes[i].peakPoint.elCoordinate);
    			}

    			CAN->IER &= ~(CAN_IER_TMEIE); // Interruption on completed transmission request (ok, arbitration loss, abort and error)
    			CAN->IER &= ~(CAN_IER_FMPIE0); // Interruption on message pending into the fifo 0
    			CAN->IER &= ~(CAN_IER_FMPIE1); // Interruption on message pending into the fifo 1
    			CAN->IER &= ~(CAN_IER_FOVIE0); // Interruption on message overrun into the fifo 0
    			CAN->IER &= ~(CAN_IER_FOVIE1); // Interruption on message overrun into the fifo 1
    			CAN->IER &= ~(CAN_IER_ERRIE); // Interruption on error
    			CAN->IER &= ~(CAN_IER_LECIE); // Interruption on last error code

    		}

    	}
  	}

  	/********** FIFO 1 INTERRUPT **********/

  		// Fifo 1 overrun
  	  	if ((interrupts & CAN_IER_FOVIE1) != 0U){

  	    	if ((rf1rflags & CAN_RF1R_FOVR1) != 0U) {

  	      		printf("Fifo 1 overrun ! \r\n");
  	      		printf("Send new remote frame ! \r\n");
  	      		sendRemoteFrame();

  	      		// Clear fifo 1 overrun flag
  	      		CAN->RF1R &= ~CAN_RF1R_FOVR1_Msk;
  	    	}
  	  	}

  	  	// Fifo 1 full
  	  	if ((interrupts & CAN_IER_FFIE1) != 0U)	{

  	    	if ((rf1rflags & CAN_RF1R_FULL1) != 0U)	{

  	      		printf("Fifo 1 full ! \r\n");

  	      		// Clear fifo 1 full flag
  	      		CAN->RF1R &= ~CAN_RF1R_FULL1_Msk;
  	    	}
  	  	}

  	  	// Fifo 1 message pending
  	  	if ((interrupts & CAN_IER_FMPIE1) != 0U) {

  	    	// Check if message is still pending
  	    	if ((CAN->RF1R & CAN_RF1R_FMP1) != 0U) {

  	    		printf("Fifo 1 message pending ! \r\n");
  	    		receiveDataFrame();

  	    		// Ask for informations about the four next passes
  	    		if (numberOfDataFrameReceived < 24) {
  	    		    sendRemoteFrame();
  	    		}

  	    		// Disable CAN IRQ if we already have the informations about the four next passes
  	    		else {

  	    			printf("Disable CAN IRQ because we have the informations about the 4 next passes ! \r\n");
  	    			CAN->IER &= ~(CAN_IER_TMEIE); // Interruption on completed transmission request (ok, arbitration loss, abort and error)
  	    			CAN->IER &= ~(CAN_IER_FMPIE0); // Interruption on message pending into the fifo 0
  	    			CAN->IER &= ~(CAN_IER_FMPIE1); // Interruption on message pending into the fifo 1
  	    			CAN->IER &= ~(CAN_IER_FOVIE0); // Interruption on message overrun into the fifo 0
  	    			CAN->IER &= ~(CAN_IER_FOVIE1); // Interruption on message overrun into the fifo 1
  	    			CAN->IER &= ~(CAN_IER_ERRIE); // Interruption on error
  	    			CAN->IER &= ~(CAN_IER_LECIE); // Interruption on last error code

  	       		}

  	    	}
  	  	}

  	/********** STATUS CHANGE ERROR INTERRUPT **********/

	// Sleep aknowledge
	if ((interrupts & CAN_IER_SLKIE) != 0U) {

    	if ((msrflags & CAN_MSR_SLAKI) != 0U) {

    		printf("Sleep aknowledge for the can ! \r\n");

    		// Clear Sleep interrupt Flag
    		CAN->MSR &= ~CAN_MSR_SLAKI_Msk;
    	}
  	}

	// WakeUp aknowledge
  	if ((interrupts & CAN_IER_WKUIE) != 0U){

    	if ((msrflags & CAN_MSR_WKUI) != 0U){


      	printf("Wakeup aknowledge for the can ! \r\n");

      	// Clear WakeUp Flag
     	CAN->MSR &= ~CAN_MSR_WKUI_Msk;

    	}
  	}

  	// Error
	if ((interrupts & CAN_IER_ERRIE) != 0U){

    	if ((msrflags & CAN_MSR_ERRI) != 0U){

      		// Error Warning Flag
      		if (((interrupts & CAN_IER_EWGIE) != 0U) && ((esrflags & CAN_ESR_EWGF) != 0U)){

        		printf("Error warning flag ! \r\n");

      		}

      		// Error Passive Flag
      		if (((CAN_IER_EPVIE) != 0U) && ((esrflags & CAN_ESR_EPVF) != 0U)) {


        		printf("Error passive flag ! \r\n");

      		}

      		// Bus-off Flag
      		if (((interrupts & CAN_IER_BOFIE) != 0U) && ((esrflags & CAN_ESR_BOFF) != 0U)) {

        		printf("Error bus-off flag ! \r\n");

      		}

      		// Error Code Flag
      		if (((interrupts & CAN_IER_LECIE) != 0U) && ((esrflags & CAN_ESR_LEC) != 0U)) {

      			sendRemoteFrame();

        		switch (esrflags & CAN_ESR_LEC) {

          			case (CAN_ESR_LEC_0):
            		printf("Stuff error ! \r\n");
          			printf("Send new remote frame ! \r\n");
            		break;

          			case (CAN_ESR_LEC_1):
            		printf("Form error ! \r\n");
          			printf("Send new remote frame ! \r\n");
            		break;

          			case (CAN_ESR_LEC_1 | CAN_ESR_LEC_0):
            		printf("Acknowledgment error ! \r\n");
          			printf("Send new remote frame ! \r\n");
            		break;

          			case (CAN_ESR_LEC_2):
            		printf("Bit recessive error ! \r\n");
          			printf("Send new remote frame ! \r\n");
            		break;

          			case (CAN_ESR_LEC_2 | CAN_ESR_LEC_0):
            		printf("Bit dominant error ! \r\n");
          			printf("Send new remote frame ! \r\n");
            		break;

          			case (CAN_ESR_LEC_2 | CAN_ESR_LEC_1):
            		printf("CRC error ! \r\n");
          			printf("Send new remote frame ! \r\n");
           			break;

          			default:
            		break;
        		}


        		// Clear Last error code Flag
        		CAN->ESR &= ~CAN_ESR_LEC_Msk;
      		}
    	}

    	// Clear ERRI Flag
    	CAN->MSR &= ~CAN_MSR_ERRI_Msk;
  	}

}

#endif /* FUNCTION_H_ */
