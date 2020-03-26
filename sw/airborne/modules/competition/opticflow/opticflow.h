#ifndef PAPARAZZI_OPTICFLOW_H
#define PAPARAZZI_OPTICFLOW_H

#include <modules/computer_vision/lib/vision/image.h>

#define OF_ACT_LEFT 0
#define OF_ACT_STRAIGHT 1
#define OF_ACT_RIGHT 2

extern void opticflow_init();
extern void opticflow_loop();

extern uint8_t max_points;
extern uint8_t reset_below_points;
extern uint8_t pyramid;
extern uint8_t fast9_threshold;

struct image_t* store_image(struct image_t* img);
void draw_current_corners(struct image_t *img, struct point_t* positive_points, int positive_points_size);

extern uint8_t of_get_suggested_action();

void parse_images(struct point_t **positive_points, int* positive_points_size);

void opticflow_reset();


#ifdef __cplusplus
extern "C" {
#endif

void OpticflowInit();
void OpticflowLoop();

#ifdef __cplusplus
}
#endif

#endif //PAPARAZZI_OPTICFLOW_H
