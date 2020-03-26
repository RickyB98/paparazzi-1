#ifndef PAPARAZZI_HORIZON_DETECTION_HPP
#define PAPARAZZI_HORIZON_DETECTION_HPP

#include "horizon_detection.h"

#include <opencv2/core/core.hpp>
#include "modules/computer_vision/lib/vision/image.h"

cv::Mat image_edges(struct image_t *img);
struct image_t *horizonDetection(struct image_t *img);
int followHorizonRight(cv::Mat *edge_image, Dot *p, int *horizon);
int followHorizonLeft(cv::Mat *edge_image, Dot *p, int y_lim, int *horizon);
bool isFloor(struct image_t *img, int x, int y);
void findHorizonCandidate(struct image_t *img, cv::Mat *edge_image, Dot *p);

#endif //PAPARAZZI_HORIZON_DETECTION_HPP
