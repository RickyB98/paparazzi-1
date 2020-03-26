#ifndef PAPARAZZI_HORIZON_DETECTION_H
#define PAPARAZZI_HORIZON_DETECTION_H

struct image_t * horizon_detection_callback(struct image_t *img);
extern void horizon_detection_init();
extern void horizon_detection_loop();

// color filter settings
extern uint8_t cf_ymin;
extern uint8_t cf_ymax;
extern uint8_t cf_umin;
extern uint8_t cf_umax;
extern uint8_t cf_vmin;
extern uint8_t cf_vmax;
// RANSAC Horizon settings
extern bool draw;
extern uint8_t ransac_threshold;
extern uint8_t ransac_iter;
extern uint8_t sec_horizon_threshold;
extern uint8_t obstacle_threshold;
#endif //PAPARAZZI_HORIZON_DETECTION_H
