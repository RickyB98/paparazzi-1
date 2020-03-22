#include "horizon_detection_cxx.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "horizon_detection.h"

void horizon_detection_init() {
    HorizonDetectionInit();
}
void horizon_detection_loop() {
    HorizonDetectionLoop();
}

#ifdef __cplusplus
}
#endif