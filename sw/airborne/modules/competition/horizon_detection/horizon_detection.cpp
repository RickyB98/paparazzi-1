#include "horizon_detection_cxx.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "horizon_detection.h"

#include "modules/computer_vision/cv.h"

void horizon_detection_init() {
    HorizonDetectionInit();
    cv_add_to_device(&COMPETITION_CAMERA_FRONT, horizonDetection, 5);
}
void horizon_detection_loop() {
    HorizonDetectionLoop();
}

#ifdef __cplusplus
}
#endif