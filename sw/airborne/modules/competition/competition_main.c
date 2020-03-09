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

float _headingRate = 0;
float _speed = 0;

void setHeadingRate(float headingRate) {
	_headingRate = headingRate;
}

void setSpeed(float speed) {
	_speed = speed;
}


void competition_init()
{
		
}

void competition_loop()
{

	guidance_h_set_guided_heading_rate(_headingRate);
	struct FloatEulers * eulers = stateGetNedToBodyEulers_f();
	
	guidance_h_set_guided_vel(_speed * cos(eulers->psi), _speed * sin(eulers->psi));

}

void competition_event()
{
}
