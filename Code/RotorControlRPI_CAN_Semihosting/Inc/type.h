#ifndef TYPE_H_
#define TYPE_H_

#include <stdint.h>

typedef struct {
	uint32_t size;
	uint8_t* message;
} CanFrame;

#endif /* TYPE_H_ */
