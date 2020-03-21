#ifndef PAPARAZZI_HORIZON_DETECTION_HPP
#define PAPARAZZI_HORIZON_DETECTION_HPP

#include <opencv2/core/core.hpp>
#include "modules/computer_vision/lib/vision/image.h"

typedef struct Dot
{
    float x;
    float y;
} Dot;

cv::Mat image_edges(struct image_t *img);
struct image_t * horizonDetection(struct image_t *img);
int followHorizonRight(cv::Mat *edge_image, Dot *p, int *horizon);
int followHorizonLeft(cv::Mat *edge_image, Dot *p, int y_lim, int *horizon);
bool isFloor(struct image_t *img, int x, int y, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M);
cv::Mat findHorizonCandidate(struct image_t *img, cv::Mat *edge_image, Dot *p);
extern void ransacHorizon(int *horizon, int *best_horizon);

void HorizonDetectionInit();
void HorizonDetectionLoop();

#endif //PAPARAZZI_HORIZON_DETECTION_HPP
