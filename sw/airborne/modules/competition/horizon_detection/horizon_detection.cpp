#include "horizon_detection_cxx.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "horizon_detection.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "modules/computer_vision/cv.h"

struct image_t * horizon_detection_callback(struct image_t *img){
    
}

void horizon_detection_init() {
    HorizonDetectionInit();
    cv_add_to_device(&COMPETITION_CAMERA_FRONT, horizonDetection, 5);
}
void horizon_detection_loop() {
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED){
        return;
    }
    HorizonDetectionLoop();
}

#ifdef __cplusplus
}
#endif