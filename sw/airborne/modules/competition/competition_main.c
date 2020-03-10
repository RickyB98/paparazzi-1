#include "modules/competition/competition_main.h"

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include "math.h"
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define HEADING_M 0
#define HEADING_RATE_M 1

float _headingRate = 0;
float _speed = 0;
float _heading = 0;

int mode = HEADING_M;

void setHeadingRate(float headingRate) {
	_headingRate = headingRate;
	mode = HEADING_RATE_M;
}

void setHeading(float heading) {
	_heading = heading;
	mode = HEADING_M;
}

void setSpeed(float speed) {
	_speed = speed;
}


void competition_init()
{
		
}

void competition_loop()
{
	switch (mode) {
		case HEADING_M: {
			guidance_h_set_guided_heading(_heading);
		}
		break;
		case HEADING_RATE_M: 
		default: {
			guidance_h_set_guided_heading_rate(_headingRate);
		}
		break;
	}

	struct FloatEulers * eulers = stateGetNedToBodyEulers_f();
	
	guidance_h_set_guided_vel(_speed * cos(eulers->psi), _speed * sin(eulers->psi));

}

void competition_event()
{
}
