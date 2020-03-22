#include "opticflow_cxx.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "opticflow.h"
#include <modules/computer_vision/cv.h>

void opticflow_init() {
    cv_add_to_device(&COMPETITION_CAMERA_FRONT, store_image, 5);
    OpticflowInit();
}

void opticflow_loop() {
    OpticflowLoop();
}

#ifdef __cplusplus
}
#endif