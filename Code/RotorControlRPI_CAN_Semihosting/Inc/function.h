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
uint32_t numberOfRemoteFrameReceived = 0;
uint32_t nextRemoteFrameId = 0;
uint32_t nextDataFrameId = 0;
uint32_t nbTransmission = 1;
CanFrame tabDataCanFrames[24] = {{.size = 8, .message = (uint8_t*)"20008018"}, {.size = 8, .message = (uint8_t*)"12005004"}, {.size = 5, .message = (uint8_t*)"00050"}, {.size = 6, .message = (uint8_t*)"032050"}, {.size = 6, .message = (uint8_t*)"000000"}, {.size = 5, .message = (uint8_t*)"02006"}, {.size = 8, .message = (uint8_t*)"20007024"}, {.size = 8, .message = (uint8_t*)"12005004"}, {.size = 5, .message = (uint8_t*)"10000"}, {.size = 6, .message = (uint8_t*)"180000"}, {.size = 6, .message = (uint8_t*)"000000"}, {.size = 5, .message = (uint8_t*)"45000"}, {.size = 8, .message = (uint8_t*)"20005007"}, {.size = 8, .message = (uint8_t*)"10055030"}, {.size = 5, .message = (uint8_t*)"01000"}, {.size = 6, .message = (uint8_t*)"020020"}, {.size = 6, .message = (uint8_t*)"174041"}, {.size = 5, .message = (uint8_t*)"35042"}, {.size = 8, .message = (uint8_t*)"20005006"}, {.size = 8, .message = (uint8_t*)"09032030"}, {.size = 5, .message = (uint8_t*)"01000"}, {.size = 6, .message = (uint8_t*)"354055"}, {.size = 6, .message = (uint8_t*)"233005"}, {.size = 5, .message = (uint8_t*)"16037"}};

/********** PASS 1 **********/

// N°1, dateAos : 20-04-06
// N°2, timeAos : 19-50-04
// N°3, transiTime : 10.59
// N°4, startPoint.azCoordinate : 144.29
// N°5, endPoint.azCoordinate : 357.87
// N°6, middlePoint.elCoordinate : 31.51

/********** PASS 2 **********/

// N°7, dateAos : 20-04-06
// N°8, timeAos : 21-24-11
// N°9, transiTime : 10.15
// N°10, startPoint.azCoordinate : 203.66
// N°11, endPoint.azCoordinate : 331.11
// N°12, middlePoint.elCoordinate : 17.29

/********** PASS 3 **********/

// N°13, dateAos : 20-04-07
// N°14, timeAos : 08-50-30
// N°15, transiTime : 11.12
// N°16, startPoint.azCoordinate : 20.20
// N°17, endPoint.azCoordinate : 174.41
// N°18, middlePoint.elCoordinate : 35.42

/********** PASS 4 **********/

// dateAos : 20-04-07
// timeAos : 10-24-30
// transiTime : 10.00
// startPoint.azCoordinate : 354.55
// endPoint.azCoordinate : 233.05
// middlePoint.elCoordinate : 16.37

/***** CAN *****/
void canFilterConfig(void);
void canInit(void);
void sendDataFrame(void);
void receiveRemoteFrame(void);

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
  CAN->sFilterRegister[0].FR1 |= (0x01 << CAN_F0R1_FB21_Pos);

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

  // Automatic retransmission diseable
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
  CAN->IER |= CAN_IER_ERRIE; // Interruption on message pending into the fifo 1CAN->IER |= CAN_IER_ERRIE; // Interruption on message pending into the fifo 1
  CAN->IER |= CAN_IER_LECIE; // Interruption on message pending into the fifo 1


  // Request exit from initialization mode
  CAN->MCR &= ~CAN_MCR_INRQ;

  while ((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK){
    printf("Wait until INAK bit is clear to exit from can initialization mode\r\n");
  } // Wait until we quit initialization mode

  printf("CAN INITIALIZATION OK\r\n");

}

void canEnableIRQ(){

	NVIC_SetPriority(CEC_CAN_IRQn, 1); // Set interruption priority level to 1
	NVIC_EnableIRQ(CEC_CAN_IRQn); // Enable interruption for the NVIC
}

void receiveRemoteFrame(){

	nextRemoteFrameId = (numberOfRemoteFrameReceived >= 6)? ( (numberOfRemoteFrameReceived % 6) + 1) : (numberOfRemoteFrameReceived + 1);

	// Get the fifo number where the remote frame is stored
	uint32_t fifoNumber = ((CAN->RF0R & CAN_RF0R_FMP0) != 0U) ? 0 : 1;
	printf("The data frame received is store into the fifo %ld \r\n", fifoNumber);

	// Get the id of the received remote frame
	uint32_t receivedRemoteFrameId = (CAN_RI0R_STID & CAN->sFIFOMailBox[fifoNumber].RIR) >> CAN_TI0R_STID_Pos;
	printf("The id of the remote frame received is %ld\r\n", receivedRemoteFrameId);

	// Check if the header received and the supposed next id to receive are same
	if(receivedRemoteFrameId == nextRemoteFrameId){

		// Get the data lenght of the received remote frame
		uint32_t receivedRemoteFrameDlc = (CAN_RDT0R_DLC & CAN->sFIFOMailBox[fifoNumber].RDTR) >> CAN_RDT0R_DLC_Pos;
		printf("The size of the remote frame received is %ld\r\n",receivedRemoteFrameDlc);

		// Check if the data lenght received and the supposed next data lenght to receive are same
		if(receivedRemoteFrameDlc == (tabDataCanFrames[numberOfRemoteFrameReceived].size)){

			printf("The remote frame received is the one we are looking for\r\n");
			numberOfRemoteFrameReceived++;
			printf("numberOfRemoteFrameReceived : %ld\r\n", numberOfRemoteFrameReceived);
			sendDataFrame();
		}

		else{
			printf("The data length of the received remote frame is not the one we are looking for\r\n");
		}
	}

	else{
			printf("The id of the received remote frame is not the one we are looking for\r\n");
	}

	// Release the FIFO
	(fifoNumber == 0) ? (CAN->RF0R |= CAN_RF0R_RFOM0) : (CAN->RF1R |= CAN_RF1R_RFOM1);

}

void sendDataFrame(){

	nextDataFrameId = (numberOfRemoteFrameReceived < 6)?  numberOfRemoteFrameReceived : ( (numberOfRemoteFrameReceived % 6 == 0)? 6 : (numberOfRemoteFrameReceived % 6) );

	uint32_t emptyMailbox;

	// Check that all the Tx mailboxes are not full */
	if ( ( (CAN->TSR & CAN_TSR_TME0) != 0U ) && ( (CAN->TSR & CAN_TSR_TME1) != 0U ) && ( (CAN->TSR & CAN_TSR_TME2) != 0U) ){

		// Select an empty transmit mailbox
		emptyMailbox = (CAN->TSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

		printf("The transmit mailbox selected for the data frame with id %ld is in the mailbox %ld\r\n", nextDataFrameId , emptyMailbox);

		// IDE bit set to 0 because we use format CAN Strandard format (2.0A)
		CAN->sTxMailBox[emptyMailbox].TIR &= ~(0x01 << 2); // Set bit 2 to 0

		// RTR bit set to 1 because we send a data frame
		CAN->sTxMailBox[emptyMailbox].TIR |= (0x00 << 1); // Set bit 1 to 1

		// Set the identifier field according to the parameter "nextRemoteFrameId"
		CAN->sTxMailBox[emptyMailbox].TIR &= ~(0x00003FFFF << 3); // Clear bits 3 to 20
		CAN->sTxMailBox[emptyMailbox].TIR &= ~(0x000007FF << 21); // Clear bits 21 to 31
		CAN->sTxMailBox[emptyMailbox].TIR = ( (uint32_t) nextDataFrameId << 21);

		// Size of the data frame according to the parameter "canFrame->size"
		CAN->sTxMailBox[emptyMailbox].TDTR &= ~(0xFFFFFFFF << 0); // Clear bits 0 to 3
		CAN->sTxMailBox[emptyMailbox].TDTR |= ( ( (uint32_t) tabDataCanFrames[(numberOfRemoteFrameReceived-1)].size ) << 0);

		printf("The size of the data frame is %ld and the value is %s\r\n", tabDataCanFrames[(numberOfRemoteFrameReceived-1)].size , tabDataCanFrames[(numberOfRemoteFrameReceived-1)].message);

		// Fill the data
		CAN->sTxMailBox[emptyMailbox].TDHR &= ~(0xFFFFFFFF << 0); //Clear
		CAN->sTxMailBox[emptyMailbox].TDHR |= ( ( (uint32_t) ( tabDataCanFrames[(numberOfRemoteFrameReceived-1)].message[7] ) << CAN_TDH0R_DATA7_Pos) | ( (uint32_t) ( tabDataCanFrames[(numberOfRemoteFrameReceived-1)].message[6] ) << CAN_TDH0R_DATA6_Pos) | ( (uint32_t) ( tabDataCanFrames[(numberOfRemoteFrameReceived-1)].message[5] ) << CAN_TDH0R_DATA5_Pos) | ( (uint32_t) ( tabDataCanFrames[(numberOfRemoteFrameReceived-1)].message[4] ) << CAN_TDH0R_DATA4_Pos) );
		CAN->sTxMailBox[emptyMailbox].TDLR &= ~(0xFFFFFFFF << 0); //Clear
		CAN->sTxMailBox[emptyMailbox].TDLR |= ( ( (uint32_t) ( tabDataCanFrames[(numberOfRemoteFrameReceived-1)].message[3] ) << CAN_TDL0R_DATA3_Pos) | ( (uint32_t) ( tabDataCanFrames[(numberOfRemoteFrameReceived-1)].message[2] ) << CAN_TDL0R_DATA2_Pos) | ( (uint32_t) ( tabDataCanFrames[(numberOfRemoteFrameReceived-1)].message[1] ) << CAN_TDL0R_DATA1_Pos) | ( (uint32_t) ( tabDataCanFrames[(numberOfRemoteFrameReceived-1)].message[0] ) << CAN_TDL0R_DATA0_Pos) );

		// Ask for transmission request : TXRQ = 1
		CAN->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

		if( (numberOfRemoteFrameReceived % 6) == 0){
				printf("*************** TRANSMISSION %ld***********************\r\n", nbTransmission);
				nbTransmission++;
		}
	}

	else {
      printf("All the mailbox are full\r\n");
	}
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

        		printf("Transmission succeed from mailbox 0 !\r\n");

    		}

    		// Transmission failed due to arbitration lost
      		if ((tsrflags & CAN_TSR_ALST0) != 0U){

          		printf("Transmission fail due to arbitration lost from mailbox 0 !\r\n");
        	}

        	// Transmission failed due error
        	if ((tsrflags & CAN_TSR_TERR0) != 0U){

          		printf("Transmission fail due to error from mailbox 0 !\r\n");

        	}

        	// Transmission aborted
        	if ((tsrflags & CAN_TSR_ABRQ0) != 0U) {

        		printf("Transmission aborted from mailbox 0!\r\n");
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

    	        printf("Transmission succeed from mailbox 1 !\r\n");

    	    }

    	    // Transmission failed due to arbitration lost
    	    if ((tsrflags & CAN_TSR_ALST1) != 0U){

    	    	printf("Transmission fail due to arbitration lost from mailbox 1 !\r\n");

    	    }

    	    // Transmission failed due error
    	    if ((tsrflags & CAN_TSR_TERR1) != 0U){

    	    	printf("Transmission fail due to error from mailbox 1 !\r\n");

    	    }

    	    // Transmission aborted
    	    if ((tsrflags & CAN_TSR_ABRQ1) != 0U) {

    	        printf("Transmission aborted from mailbox 1 !\r\n");

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
    	    }

    	    // Transmission failed due error
    	    if ((tsrflags & CAN_TSR_TERR2) != 0U){

    	    	printf("Transmission fail due to error from mailbox 2 !\r\n");

    	    }

    	    // Transmission aborted
    	    if ((tsrflags & CAN_TSR_ABRQ2) != 0U) {

    	    	printf("Transmission aborted from mailbox 2 !\r\n");

    	    }
    	}
	}

	/********** FIFO 0 INTERRUPT **********/

	// Fifo 0 overrun
  	if ((interrupts & CAN_IER_FOVIE0) != 0U){

    	if ((rf0rflags & CAN_RF0R_FOVR0) != 0U) {

      		printf("Fifo 0 overrun ! \r\n");

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
    		receiveRemoteFrame();

    	}
  	}

  	/********** FIFO 1 INTERRUPT **********/

  		// Fifo 1 overrun
  	  	if ((interrupts & CAN_IER_FOVIE1) != 0U){

  	    	if ((rf1rflags & CAN_RF1R_FOVR1) != 0U) {

  	      		printf("Fifo 1 overrun ! \r\n");

  	      		// Clear fifo 0 overrun flag
  	      		CAN->RF1R &= ~CAN_RF1R_FOVR1_Msk;
  	    	}
  	  	}

  	  	// Fifo 1 full
  	  	if ((interrupts & CAN_IER_FFIE1) != 0U)	{

  	    	if ((rf1rflags & CAN_RF1R_FULL1) != 0U)	{

  	      		printf("Fifo 1 full ! \r\n");

  	      		// Clear fifo 0 full flag
  	      		CAN->RF1R &= ~CAN_RF1R_FULL1_Msk;
  	    	}
  	  	}

  	  	// Fifo 0 message pending
  	  	if ((interrupts & CAN_IER_FMPIE1) != 0U) {

  	    	// Check if message is still pending
  	    	if ((CAN->RF1R & CAN_RF1R_FMP1) != 0U) {

  	    		printf("Fifo 1 message pending ! \r\n");
  	    		receiveRemoteFrame();

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

        		switch (esrflags & CAN_ESR_LEC) {

          			case (CAN_ESR_LEC_0):
            		printf("Stuff error ! \r\n");
            		break;

          			case (CAN_ESR_LEC_1):
            		printf("Form error ! \r\n");
            		break;

          			case (CAN_ESR_LEC_1 | CAN_ESR_LEC_0):
            		printf("Acknowledgment error ! \r\n");
            		break;

          			case (CAN_ESR_LEC_2):
            		printf("Bit recessive error ! \r\n");
            		break;

          			case (CAN_ESR_LEC_2 | CAN_ESR_LEC_0):
            		printf("Bit dominant error ! \r\n");
            		break;

          			case (CAN_ESR_LEC_2 | CAN_ESR_LEC_1):
            		printf("CRC error ! \r\n");
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
