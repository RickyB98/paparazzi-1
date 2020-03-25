#ifndef PAPARAZZI_COMPETITION_MAIN_H
#define PAPARAZZI_COMPETITION_MAIN_H



extern void competition_init();
extern void competition_loop();
// extern void competition_datalink();

extern void setHeadingRate(float headingRate);
extern void setHeading(float heading);
extern void setSpeed(float speed);

#ifdef __cplusplus
extern "C" {
#endif

void CompetitionInit();
void CompetitionLoop();

#ifdef __cplusplus
}
#endif

#endif //PAPARAZZI_COMPETITION_MAIN_H
