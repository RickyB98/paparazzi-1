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
bool isFloor(struct image_t *img, int x, int y);
cv::Mat findHorizonCandidate(struct image_t *img, cv::Mat *edge_image, Dot *p);

void HorizonDetectionInit();
void HorizonDetectionLoop();


#endif //PAPARAZZI_HORIZON_DETECTION_HPP
