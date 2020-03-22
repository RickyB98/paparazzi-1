#ifndef PAPARAZZI_OPTICFLOW_H
#define PAPARAZZI_OPTICFLOW_H

#include <modules/computer_vision/lib/vision/image.h>

extern void opticflow_init();
extern void opticflow_loop();

struct image_t* store_image(struct image_t* img);

void parse_images();

#endif //PAPARAZZI_OPTICFLOW_H
