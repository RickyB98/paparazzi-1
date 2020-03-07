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

void competition_init()
{
}

int k = 0;

void competition_loop()
{
	struct NedCoor_f *ned = stateGetPositionNed_f();

	fprintf(stderr, "pos: %f, %f, %f\n", ned->x, ned->y, ned->z);

	double targetN = 2 * cos((k % 10) / 10. * 2 * 3.14159);
	double targetE = 2 * sin((k % 10) / 10. * 2 * 3.14159);

	guidance_h_set_guided_pos(targetN, targetE);

	if (abs(ned->x - targetN) < .05 && abs(ned->y - targetE) < .05)
	{
		++k;
	}
}

void competition_event()
{
}
