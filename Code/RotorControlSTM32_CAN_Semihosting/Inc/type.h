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
	float azCoordinate; // azimut coordinate (°)
	float elCoordinate; // elevation coordinate (°)
} Point;

typedef struct {

	float currentTimeMinute; // Current time of the tracking (min)
	float currentTimeSecond; // Current time of the tracking (sec)
	float currentSpeedAz; // Current speed of the tracking in az (°/min)
	float currentSpeedEl; // Current speed of the tracking in el (°/min)
	float maximumSpeedAz; // Maximum speed of the tracking in az (°/min)
	float maximumSpeedEl; // Maximum speed of the tracking in el (°/min)
	float maximumMotorSpeedAz; // Maximum speed of the az motor (°/min at 12V)
	float maximumMotorSpeedEl; // Maximum speed of the el motor (°/min at 12V)
	uint8_t directionSpeedAz; // Direction speed of the tracking in az (1 : Clockwise, 0 : Counter-clockwise)
	uint8_t directionSpeedEl; // Direction speed of the tracking in el (1 : Clockwise, 0 : Counter-clockwise)

} Tracking;

typedef struct {

	Time passTime; // Starting date and time of the pass
	float transitTimeMinute; // Duration of the pass (min)
	float transitTimeSecond; // Duration of the pass (sec)
	Point startPoint; // Starting point of the pass (azCoordinate; elCordinate)
	Point endPoint; // End point of the pass (azCoordinate; elCordinate)
	Point peakPoint; // Middle point of the pass (azCoordinate; elCordinate)
	float averageSpeedAz; // Average speed in az on the pass (°/min)
	float averageSpeedEl; // Average speed in el on the pass (°/min)
	Tracking tracking; // A structure to describe the tracking

} Pass;


#endif /* TYPE_H_ */
