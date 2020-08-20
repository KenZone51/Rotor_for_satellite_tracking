#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f072rb.h"
#include "function.h"

/***** CAN *****/
extern void canInit(void);
extern void canEnableIRQ(void);
extern void sendRemoteFrame(void);

/***** PWM *****/
extern void pwmInit(void);
extern void home(void);

extern void initialise_monitor_handles();

int main(void){

	initialise_monitor_handles();

	canInit();

	pwmInit();

	canEnableIRQ();

	sendRemoteFrame();

	home();

	while(1){

	}
}
