#include "competition_main_cxx.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "competition_main.h"

void competition_init() {
    CompetitionInit();
}

void competition_loop() {
    CompetitionLoop();
}

#ifdef __cplusplus
}
#endif
