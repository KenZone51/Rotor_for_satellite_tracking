#ifndef TYPE_H_
#define TYPE_H_

#include <stdint.h>

typedef struct {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} Time;

typedef struct {
	float azCoordinate;
	float elCoordinate;
} Point;

typedef struct {

	float currentTimeMinute; // Unit : min
	float currentTimeSecond; // Unit : sec
	float currentSpeedAz; // Unit : °/min
	float currentSpeedEl; // Unit : °/min
	float maximumSpeedAz; // Unit : °/min
	float maximumSpeedEl; // Unit : °/min
	float maximumMotorSpeedAz; // Unit : °/min at 12V
	float maximumMotorSpeedEl; // Unit : °/min at 12V
	uint8_t directionSpeedAz; // 1 : Clockwise, 0 : No-clockwise
	uint8_t directionSpeedEl; // 1 : Clockwise, 0 : No-clockwise

} Tracking;

typedef struct {

	Time passTime;
	float transitTimeMinute; // Unit : min
	float transitTimeSecond; // Unit : sec
	Point startPoint;
	Point endPoint;
	Point peakPoint;
	float averageSpeedAz; // Unit : °/min
	float averageSpeedEl; // Unit : °/min
	Tracking tracking;

} Pass;


#endif /* TYPE_H_ */
