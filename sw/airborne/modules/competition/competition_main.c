#ifdef __cplusplus
extern "C" {
#endif

#define NAV_C

#include "competition_main.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/competition/horizon_detection/horizon_detection.h"
#include "modules/competition/opticflow/opticflow.h"
#include "state.h"
#include "stdio.h"
#include "stdlib.h"
#include "time.h"

#define ABS(x) (((x) < 0. ? -(x) : (x)))

#define STATE_WAIT_HEADING 0
#define STATE_CONTINUE 1
#define STATE_OUT_OF_OBSTACLE_ZONE 2
#define STATE_FIND_HEADING 3

#define HEADING_M 0
#define HEADING_RATE_M 1

int current_state = STATE_CONTINUE;

float straight_speed = 1.;

int hold = 0;

float _headingRate = 0;
float _speed = 0;
float _heading = 0;

int mode = HEADING_M;

bool of_na = false;

void competition_init() {
  CompetitionInit();
  guidance_h_SetMaxSpeed(10.0f);
  guidance_h.gains.v = 150.;
}

uint8_t tick = 0;

int hdCount = 0;

void competition_loop() {
  CompetitionLoop();

  struct NedCoor_f *ned = stateGetPositionNed_f();
  struct NedCoor_f *accel = stateGetAccelNed_f();
  struct FloatEulers *eulers = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  if (!InsideObstacleZone(ned->x, ned->y)) {
    current_state = STATE_OUT_OF_OBSTACLE_ZONE;
  } else if (current_state == STATE_OUT_OF_OBSTACLE_ZONE) {
    current_state = STATE_FIND_HEADING;
    //current_state = STATE_CONTINUE;
    printf("[OOOZ] Find heading\n");
  }

  switch (current_state) {
  case STATE_OUT_OF_OBSTACLE_ZONE: {
    float psi = atan2(ned->y, ned->x) + M_PI;

    while (psi > 2 * M_PI) {
      psi -= 2 * M_PI;
    }
    while (psi < 0) {
      psi += 2 * M_PI;
    }

    float diff = ABS(psi - eulers->psi);

    while (diff > M_PI) {
      diff -= 2 * M_PI;
    }
    if (ABS(diff) < .3) {
      //printf("[OOOZ] Going back in...\n");
      setSpeed(0.5);
      setHeadingRate(0);
    } else {
      setSpeed(0);
      setHeadingRate(M_PI_2);
    }
    opticflow_reset();
  } break;
  case STATE_CONTINUE: {
    float accelY = ACCEL_FLOAT_OF_BFP(accel->y);
    float accelZ = ACCEL_FLOAT_OF_BFP(accel->z);

    // Opticflow (only check when accel < 1e-2)
    if (sqrt(pow(rates->p, 2) + pow(rates->q, 2) + pow(rates->r, 2)) < .05 &&
        ABS(eulers->phi) < 1e-1 &&
        (!stateIsSideslipValid() || ABS(stateGetSideslip_f()) < 1e-1)) {
      if (of_na) {
        of_na = false;
        printf("[OF] Available\n");
      }
      switch (of_get_suggested_action()) {
      case OF_ACT_STRAIGHT: {
        setSpeed(straight_speed);
        setHeadingRate(0.);
      } break;
      case OF_ACT_LEFT: {
        fprintf(stderr, "[OF_ACT] Go left\n");
        setSpeed(0);
        setHeadingRate(-60 * M_PI / 180.);
        hold = 30;
        current_state = STATE_WAIT_HEADING;
      } break;
      case OF_ACT_RIGHT: {
        fprintf(stderr, "[OF_ACT] Go right\n");
        setSpeed(0);
        setHeadingRate(60 * M_PI / 180.);
        hold = 30;
        //printf("[OF_ACT] Find heading\n");
        current_state = STATE_WAIT_HEADING;
      } break;
      }
    } else {
      // fprintf(stderr, "[OF] N/A %d\n", tick++);
      if (!of_na) {
        of_na = true;
        printf("[OF] N/A\n");
      }
      opticflow_reset();
    }
    if (current_state != STATE_CONTINUE)
      break;

    int height = hdGetHorizonHeight();
    if (height > 20 && height < 200/* && of_na*/) {
      int obstacleArray[IMAGE_WIDTH];
      hdGetObstacleArray((int *)obstacleArray);

      int begins[30];
      int ends[30];
      int currentSegment = 0;
      bool inSegment = false;

      float pitchThreshold = 20 * M_PI / 180.;
      
      float upperBound = Max(10, Min(- 340 * eulers->theta, 240));

      for (uint16_t i = 0; i < IMAGE_WIDTH; ++i) {
        int val = obstacleArray[i];
        if (inSegment && (val < 0 || val >= upperBound)) {
          ends[currentSegment++] = i;
          inSegment = false;
        } else if (!inSegment && (val >= 0 && val < upperBound)) {
          begins[currentSegment] = i;
          inSegment = true;
        }
        if (currentSegment >= 30)
          break;
      }
      if (inSegment && currentSegment < 30) {
        ends[currentSegment++] = 519;
      }

      int totalLength = 0;
      //printf("[HD_SEG] ");
      for (int j = 0; j < currentSegment; ++j) {
        //printf("#%d - begin: %d, end: %d\n", j, begins[j], ends[j]);
        //printf("%d ", ends[j] - begins[j]);
        //if (ends[j] - begins[j] < 20)
        //  continue;
        totalLength += ends[j] - begins[j];
      }
      //printf("\n");
      if (totalLength > 60) {
        setSpeed(0);
        current_state = STATE_FIND_HEADING;
        printf("[HD_ACT] Find heading\n");
      }
    }

  } break;
  case STATE_FIND_HEADING: {
    int hdBestHeading = hdGetBestHeading() - 260;
    //setHeading(eulers->psi + hdBestHeading / 240. * 45 * M_PI / 180.);

    float rate = hdBestHeading / 260. * M_PI_4;
    printf("-> rate: %f (bestHeading w/offset: %d)\n", rate, hdBestHeading);
    setHeadingRate(rate);
    setSpeed(0);
    hold = 20;
    current_state = STATE_WAIT_HEADING;
  } break;
  case STATE_WAIT_HEADING: {
    setSpeed(0);
    if (hold > 0) {
      --hold;
    } else {
      setHeadingRate(0.0f);
      current_state = STATE_CONTINUE;
    }
  } break;
  }
  switch (mode) {
  case HEADING_M: {
    /*float diff = ABS(_heading - eulers->psi);

    while (diff > M_PI) {
      diff -= 2 * M_PI;
    }
    if (ABS(diff) < .3) {
      guidance_h_set_guided_heading_rate(_heading);
    } else {
      guidance_h_set_guided_heading_rate((diff >= 0 ? -1 : 1) * M_PI_4);
    }*/
    guidance_h_set_guided_heading(_heading);
  } break;
  case HEADING_RATE_M:
  default: {
    guidance_h_set_guided_heading_rate(_headingRate);
  } break;
  }

  guidance_h_set_guided_vel(_speed * cos(eulers->psi),
                            _speed * sin(eulers->psi));
}

void setHeadingRate(float headingRate) {
  _headingRate = headingRate;
  mode = HEADING_RATE_M;
}

void setHeading(float heading) {
  _heading = heading;
  mode = HEADING_M;
}

void setSpeed(float speed) { _speed = speed; }

#ifdef __cplusplus
}
#endif
