#ifndef PAPARAZZI_HORIZON_DETECTION_H
#define PAPARAZZI_HORIZON_DETECTION_H

extern void horizon_detection_init();
extern void horizon_detection_loop();

extern uint8_t cf_ymin;
extern uint8_t cf_ymax;
extern uint8_t cf_umin;
extern uint8_t cf_umax;
extern uint8_t cf_vmin;
extern uint8_t cf_vmax;
extern bool draw;
#endif //PAPARAZZI_HORIZON_DETECTION_H
