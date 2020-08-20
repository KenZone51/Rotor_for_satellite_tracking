#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "stm32f072rb.h"
#include "function.h"

extern void initialise_monitor_handles();

extern void canInit();
extern void canEnableIRQ();

int main(void){

	initialise_monitor_handles();

	canInit();

	canEnableIRQ();

	while(1){

	}
}
