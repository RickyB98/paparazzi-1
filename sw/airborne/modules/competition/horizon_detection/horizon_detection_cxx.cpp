#include "horizon_detection_cxx.hpp"
#include "horizon_detection.h"

#include "modules/computer_vision/lib/vision/image.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <modules/computer_vision/opencv_contour.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

using namespace std;
using namespace cv;

#define MINIMUM_HORIZON_SEGMENT_LENGTH 5
#define RANSAC_TIMEOUT_LIM 10
#define BEST_HORIZON_QUAL_THRESHOLD 300
#define SIZE_OF_DANGER_ZONE 50
#define DANGER_LIMIT 2

struct contour_estimation cont_est;
struct contour_threshold cont_thres;

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

void findObstacles(int *obstacles, int *horizon, horizon_line_t *horizon_line,
                   int limit_left, int limit_right);
void drawHorizon(struct image_t *img, int *obstacles, horizon_line_t *horizon,
                 int limit_left, int limit_right);

RNG rng;

// Color filter settings
uint8_t cf_ymin = 0;
uint8_t cf_ymax = 0;
uint8_t cf_umin = 0;
uint8_t cf_umax = 0;
uint8_t cf_vmin = 0;
uint8_t cf_vmax = 0;
// RANSAC Horizon settings
bool draw = false;
uint8_t ransac_threshold = 1;
uint8_t ransac_iter = 1;
uint8_t sec_horizon_threshold = 1;
uint8_t obstacle_threshold = 1;

// Global variables
// enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
// float max_speed = 0.5f;
// float heading_change_rate = 20.0f * M_PI/180.0f;

Mat image_edges(struct image_t *img) {
  // Create a new image, using the original bebop image.
  Mat M(img->h, img->w, CV_8UC2, img->buf); // original
  Mat image, edge_image, thresh_image;

  // convert UYVY in paparazzi to YUV in opencv
  cvtColor(M, M, CV_YUV2GRAY_Y422);
  // cvtColor(M, M, CV_RGB2YUV);

  // Threshold all values within the indicted YUV values.
  inRange(M, Scalar(cont_thres.lower_y, cont_thres.lower_u, cont_thres.lower_v),
          Scalar(cont_thres.upper_y, cont_thres.upper_u, cont_thres.upper_v),
          thresh_image);

  /// Find contours
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  // edge_image = thresh_image;
  edge_image = M;
  int edgeThresh = 35;
  Canny(edge_image, edge_image, edgeThresh, edgeThresh * 3);
  findContours(edge_image, contours, hierarchy, CV_RETR_EXTERNAL,
               CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
  return edge_image;
}

/**
 * Checks the color of a single pixel in a YUV422 image.
 * 1 means that it passes the filter, 0 that it does not.
 *
 * @param[in] im The input image to filter
 * @param[in] x The x-coordinate of the pixel
 * @param[in] y The y-coordinate of the pixel
 * @return The success of the filter.
 */

bool isFloor(struct image_t *img, int x, int y) {
  // odd pixels are uy
  // even pixels are vy

  // Is the pixel inside the image?
  if (x < 0 || x >= img->w - 50 || y < 0 || y >= img->h) {
    return false;
  }

  // Take the right place in the buffer:
  uint8_t *buf = (uint8_t *)(img->buf);
  uint8_t *yp, *up, *vp;
  if (x % 2 == 0) {
    // Even x
    up = &buf[y * 2 * img->w + 2 * x];     // U
    yp = &buf[y * 2 * img->w + 2 * x + 1]; // Y1
    vp = &buf[y * 2 * img->w + 2 * x + 2]; // V
    // yp = &buf[y * 2 * img->w + 2 * x + 3]; // Y2
  } else {
    // Uneven x
    up = &buf[y * 2 * img->w + 2 * x - 2]; // U
    // yp = &buf[y * 2 * img->w + 2 * x - 1]; // Y1
    vp = &buf[y * 2 * img->w + 2 * x];     // V
    yp = &buf[y * 2 * img->w + 2 * x + 1]; // Y2
  }

  if ((*yp >= cf_ymin) && (*yp <= cf_ymax) && (*up >= cf_umin) &&
      (*up <= cf_umax) && (*vp >= cf_vmin) && (*vp <= cf_vmax)) {
    // the pixel passes:
    return true;
  } else {
    // the pixel does not:
    return false;
  }
}

void findHorizonCandidate(struct image_t *img, Mat *edge_image, Dot *p) {

  int x;
  bool onFloor;
  int onEdge;
  int y;
  Mat track(img->h, img->w, CV_8UC1);
  x = p->x;
  y = p->y;

  onFloor = isFloor(img, x, y);

  track.data[y * img->w + x] = 255;
  // cout << "nnnnnn" << endl;
  while (!onFloor && x < img->w) {
    x++;
    onFloor = isFloor(img, x, y);
    track.data[y * img->w + x] = 255;
  }
  // move up to closest edge
  onEdge = (edge_image->data[y * edge_image->cols + x] > 0);
  while (onEdge == 0 && x < img->w - 50 - 1) {
    x++;
    onEdge = (edge_image->data[y * edge_image->cols + x] > 0);
    track.data[y * img->w + x] = 255;
  }
  p->x = x;
  p->y = y;
}

int followHorizonLeft(Mat *edge_image, Dot *p, int y_lim, int *horizon) {

  int x;
  int y;
  x = p->x;
  y = p->y;

  while (y > y_lim && x > 0 && x < edge_image->cols + 1) {
    y--;
    if (edge_image->data[y * edge_image->cols + x] >
        0) { // edge continues right

    } else if (edge_image->data[y * edge_image->cols + x - 1] >
               0) { // edge continues bottom right
      x--;
    } else if (edge_image->data[y * edge_image->cols + x + 1] >
               0) { // edge continues top right
      x--;
    } else { // increase step
      y--;
      if (edge_image->data[y * edge_image->cols + x] > 0) {
        horizon[y + 1] = x;
      } else if (edge_image->data[y * edge_image->cols + x - 1] > 0) {
        horizon[y + 1] = x;
        x--;
      } else if (edge_image->data[y * edge_image->cols + x + 1] > 0) {
        horizon[y + 1] = x;
        x++;
      } else {
        y = y + 2; // if the edge does not continue, revert to last know edge
                   // position
        break;
      }
    }
    horizon[y] = x;
  }
  return y;
}

int followHorizonRight(Mat *edge_image, Dot *p, int *horizon) {

  int x;
  int y;
  x = p->x;
  y = p->y;

  while (y < edge_image->rows - 1 && x > 0 && x < edge_image->cols + 1) {
    y++;
    if (edge_image->data[y * edge_image->cols + x] >
        0) { // edge continues right

    } else if (edge_image->data[y * edge_image->cols + x - 1] >
               0) { // edge continues bottom right
      x--;
    } else if (edge_image->data[y * edge_image->cols + x - 2] >
               0) { // edge continues bottom right
      x = x - 2;
    } else if (edge_image->data[y * edge_image->cols + x + 1] >
               0) { // edge continues top right
      x++;
    } else if (edge_image->data[y * edge_image->cols + x + 2] >
               0) { // edge continues top right
      x = x + 2;
    } else { // increase step
      y++;
      if (edge_image->data[y * edge_image->cols + x] > 0) {
        horizon[y - 1] = x;
      } else if (edge_image->data[y * edge_image->cols + x - 1] > 0) {
        horizon[y - 1] = x;
        x--;
      } else if (edge_image->data[y * edge_image->cols + x - 2] > 0) {
        horizon[y - 1] = x;
        x = x - 2;
      } else if (edge_image->data[y * edge_image->cols + x + 1] > 0) {
        horizon[y - 1] = x;
        x++;
      } else if (edge_image->data[y * edge_image->cols + x + 2] > 0) {
        horizon[y - 1] = x;
        x = x + 2;
      } else if (edge_image->data[(y + 1) * edge_image->cols + x] > 0) {
        horizon[y - 1] = x;
        horizon[y] = x;
        y++;
      } else if (edge_image->data[(y + 1) * edge_image->cols + x - 1] > 0) {
        horizon[y - 1] = x;
        horizon[y] = x;
        y++;
        x--;
      } else if (edge_image->data[(y + 1) * edge_image->cols + x - 2] > 0) {
        horizon[y - 1] = x;
        horizon[y] = x;
        y++;
        x = x - 2;
        ;
      } else if (edge_image->data[(y + 1) * edge_image->cols + x + 1] > 0) {
        horizon[y - 1] = x;
        horizon[y] = x;
        y++;
        x++;
      } else if (edge_image->data[(y + 1) * edge_image->cols + x + 2] > 0) {
        horizon[y - 1] = x;
        horizon[y] = x;
        y++;
        x = x + 2;
      } else {
        y = y - 2; // if the edge does not continue, revert to last know edge
                   // position
        break;
      }
    }
    horizon[y] = x;
  }
  return y;
}

void ransacHorizon(int *horizon_array, horizon_line_t *best_horizon_line,
                   int limit_left, int limit_right) {
  int first = limit_left;
  int last = limit_right;

  // initialize ransac horizon variables
  int quality[ransac_iter] = {0};
  float error[ransac_iter] = {0.0f};
  float m[ransac_iter] = {0.0f};
  float b[ransac_iter] = {0.0f};

  int best_quality = 0;
  int best_error = ransac_threshold * (IMAGE_WIDTH + 1);
  float best_m = 0;
  float best_b = 0;

  uint8_t timeout_counter = 0;
  int i, j;
  for (i = 0; i < ransac_iter; i++) {
    // pick two different, non-zero points on the horizon, within [first,last]
    int s1, s2;
    timeout_counter = 0;

    do {
      s1 = rng.uniform(first, last + 1);
      timeout_counter++;
    } while (horizon_array[s1] == 0 && timeout_counter < RANSAC_TIMEOUT_LIM);

    do {
      s2 = rng.uniform(first, last + 1);
      timeout_counter++;
    } while ((s1 == s2 || horizon_array[s2] == 0) &&
             timeout_counter < RANSAC_TIMEOUT_LIM);

    if (timeout_counter >= RANSAC_TIMEOUT_LIM) {
      continue;
    }
    // calculate horizon based on s1 and s2
    m[i] = (float)(horizon_array[s2] - horizon_array[s1]) / (float)(s2 - s1);
    b[i] = horizon_array[s1] - m[i] * s1;

    // calculate error and quality
    float dx = 0;
    for (j = first; j < last; j++) {
      dx = abs(horizon_array[j] - m[i] * j - b[i]);
      if (dx < ransac_threshold) {
        error[i] += dx;
        quality[i]++;
      } else {
        error[i] += ransac_threshold;
      }
    }

    // update best horizon
    if (error[i] < best_error) {
      best_error = error[i];
      best_m = m[i];
      best_b = b[i];
      best_quality = quality[i];
    }
  }

  // calculate new limits
  float local_error;
  int limit[2] = {first, last};

  for (i = first; i <= last; i++) {
    local_error = horizon_array[i] - best_m * i - best_b;
    if (best_m > 0 && local_error < ransac_threshold &&
        i <= best_quality + 15) {
      limit[1] = i;

    } else if (best_m < 0 && local_error < ransac_threshold &&
               i <= IMAGE_WIDTH - best_quality) {
      limit[0] = i;

    } else {
      continue;
    }
  }

  // for (i=first; i<=last´; i++){
  //     local_error = horizon[i] - best_m*i - best_b;
  //     if (best_m > 0 && local_error<ransac_threshold){
  //         limit[1] = i;

  //     }
  //     else if (best_m < 0 && local_error<ransac_threshold){
  //         limit[0] = i;

  //     }
  //     else{
  //         continue;
  //     }
  // }

  best_horizon_line->m = best_m;
  best_horizon_line->b = best_b;
  best_horizon_line->quality = best_quality;
  best_horizon_line->limits[0] = limit[0];
  best_horizon_line->limits[1] = limit[1];
}

void findObstaclesFullHorizon(int *obstacleArray, int *horizonArray,
                              full_horizon_t *fullHorizon) {
  if (fullHorizon->isCompound) {
    findObstacles(obstacleArray, horizonArray, &fullHorizon->left, 0,
                  fullHorizon->intersect);
    findObstacles(obstacleArray, horizonArray, &fullHorizon->right,
                  fullHorizon->intersect + 1, IMAGE_WIDTH - 1);
  } else {
    findObstacles(obstacleArray, horizonArray, fullHorizon->main, 0,
                  IMAGE_WIDTH - 1);
  }
}

void findObstacles(int *obstacles, int *horizon, horizon_line_t *horizon_line,
                   int limit_left, int limit_right) {
  int i;
  int diff;

  for (i = Max(0, limit_left); i < Min(IMAGE_WIDTH, limit_right + 1); i++) {
    diff = (int)round(horizon[i] - horizon_line->m * i - horizon_line->b);
    if (abs(diff) > obstacle_threshold) {
      obstacles[i] = horizon[i];
    } else if(horizon_line->m == 0 && horizon_line->b==0 && horizon[i] ==0){
        obstacles[i] = 0;
    }
    else {
      obstacles[i] = -1;
      continue;
    }
  }
}

void drawFullHorizon(struct image_t *img, int *obstacles_array,
                     full_horizon_t *horizon) {
  if (horizon->isCompound) {
    drawHorizon(img, obstacles_array, &horizon->left, 0, horizon->intersect);
    drawHorizon(img, obstacles_array, &horizon->right, horizon->intersect + 1,
                IMAGE_WIDTH - 1);
  } else {
    drawHorizon(img, obstacles_array, horizon->main, 0, IMAGE_WIDTH - 1);
  }
}

void drawHorizon(struct image_t *img, int *obstacles, horizon_line_t *horizon,
                 int limit_left, int limit_right) {
  uint8_t *buffer = (uint8_t *)img->buf;

  // Go through all the pixels
  for (uint16_t y = Max(0, limit_left); y < Min(IMAGE_WIDTH, limit_right + 1);
       y++) {
    
    int x;

    if (obstacles[y] >= 0) {
      x = obstacles[y];
    } else {
      x = (int)round(horizon->m * y + horizon->b);
    }

    if (x >= img->w || x < 0) {
      continue;
    }

    // get corresponding pixels
    uint8_t *yp, *up, *vp;
    if (x % 2 == 0) {
      // Even x
      up = &buffer[y * 2 * img->w + 2 * x];     // U
      yp = &buffer[y * 2 * img->w + 2 * x + 1]; // Y1
      vp = &buffer[y * 2 * img->w + 2 * x + 2]; // V
      // yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
    } else {
      // Uneven x
      up = &buffer[y * 2 * img->w + 2 * x - 2]; // U
      // yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
      vp = &buffer[y * 2 * img->w + 2 * x];     // V
      yp = &buffer[y * 2 * img->w + 2 * x + 1]; // Y2
    }

    if (obstacles[y] >= 0) {
      *yp = 128;
      *up = 100;
      *vp = 255;
    } else {
      *yp = 128;
      *up = 0;
      *vp = 100;
    }
  }
}

void drawHorizonArray(struct image_t *img, int *horizon_array) {
  uint8_t *buffer = (uint8_t *)img->buf;

  // Go through all the pixels
  for (uint16_t y = 0; y < IMAGE_WIDTH; y++) {
    int x = horizon_array[y];
    if (x >= img->w || x < 0) {
      continue;
    }

    // get corresponding pixels
    uint8_t *yp; //, *up, *vp;
    if (x % 2 == 0) {
      // Even x
      // up = &buffer[y * 2 * img->w + 2 * x];      // U
      yp = &buffer[y * 2 * img->w + 2 * x + 1]; // Y1
      // vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
      // yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
    } else {
      // Uneven x
      // up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
      // yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
      // vp = &buffer[y * 2 * img->w + 2 * x];      // V
      yp = &buffer[y * 2 * img->w + 2 * x + 1]; // Y2
    }

    *yp = 255;
  }
}

void getHorizonArray(struct image_t *img, int *horizon) {
  Mat edge_image = image_edges(img);

  int x = 0;
  int y = 0;
  int y_max = 0;
  int y_min = 0;
  int i;
  while (y < IMAGE_WIDTH) {
    Dot p;
    p.x = x;
    p.y = y;
    findHorizonCandidate(img, &edge_image, &p);
    if (x == (img->w - 1)) {
      x = p.x;
      y = p.y;
      x = 0;
      y++;
      cout << "Does this ever get executed?" << endl;
      continue;
    } else {
      y = p.y;
      x = p.x;
      horizon[y] = x;
      // can limit y_lim to y_max to avoid overwriting past edges, however, it
      // would be helpful to know which one is better other idea: do snake
      // horizon > ransacHorizon > second snake horizon only keeping lines close
      // to the ransac Horizon
      y_min = followHorizonLeft(&edge_image, &p, 0, (int *)horizon);
      y_max = followHorizonRight(&edge_image, &p, (int *)horizon);
      // could go right first and use distance to decide if we want to overwrite
      // when moving left
      y = y_max + 1;
      // p.y = y_max +1;
      x = 0;
      // p.x = 0;
      // if the segment is too short, scrap it
      if (y_max - y_min < MINIMUM_HORIZON_SEGMENT_LENGTH) {
        for (i = y_min; i <= y_max; i++) {
          // NOTE: because "followHorizonLeft()" can overwrite valid horizon,
          // this could remove information
          horizon[i] = 0;
        }
      }
    }
  }
}

full_horizon_t mergeHorizonLines(horizon_line_t *horizon1,
                                 horizon_line_t *horizon2) {
  full_horizon_t fullHorizon;

  fullHorizon.isCompound = false;
  fullHorizon.left = {0.0f, 0.0f, 0, {0, IMAGE_WIDTH - 1}};
  fullHorizon.right = {0.0f, 0.0f, 0, {0, IMAGE_WIDTH - 1}};
  fullHorizon.main = &fullHorizon.left;
  fullHorizon.intersect = 0;

  // order left/right
  if (horizon1->m > horizon2->m) {
    fullHorizon.left = *horizon1;
    fullHorizon.right = *horizon2;
  } else {
    fullHorizon.left = *horizon2;
    fullHorizon.right = *horizon1;
  }
  // determine main horizon
  int sec_horizon_quality = 0;
  if (fullHorizon.left.quality > fullHorizon.right.quality) {
    fullHorizon.main = &fullHorizon.left;
    sec_horizon_quality = fullHorizon.right.quality;
  } else {
    fullHorizon.main = &fullHorizon.right;
    sec_horizon_quality = fullHorizon.left.quality;
  }
  // evaluate secondary horizon viability
  if (sec_horizon_quality < sec_horizon_threshold ||
      horizon1->m == horizon2->m) {
    // second horizon bad quality or parallel
    fullHorizon.isCompound = false;
  } else {
    fullHorizon.isCompound = true;
    fullHorizon.intersect =
        (int)floor((horizon1->b - horizon2->b) / (horizon2->m - horizon1->m));
  }
  return fullHorizon;
}

void HorizonDetectionInit() {
  cont_thres.lower_y = 16;
  cont_thres.lower_u = 135;
  cont_thres.lower_v = 80;
  cont_thres.upper_y = 100;
  cont_thres.upper_u = 175;
  cont_thres.upper_v = 165;
}

void HorizonDetectionLoop() {
  /*
  int local_obstacle[IMAGE_WIDTH];
  pthread_mutex_lock(&obstacle_mutex);
  memcpy(local_obstacle, global_obstacle, sizeof(int)*IMAGE_WIDTH);
  pthread_mutex_unlock(&obstacle_mutex);
  int danger_zone_start = (int)(IMAGE_WIDTH - SIZE_OF_DANGER_ZONE)/2;
  int danger_zone_end = (int)(IMAGE_WIDTH + SIZE_OF_DANGER_ZONE)/2;

  int i;
  int danger_count = 0;
  float speed = max_speed;
  int safest_direction = findBestHeadingDirection((int*) local_obstacle);
  switch (navigation_state)
  {
  case SAFE:
      for (i=danger_zone_start; i<=danger_zone_end; i++){
          if (local_obstacle[i]>=0){
              danger_count++;
          }
      }

      if (danger_count > DANGER_LIMIT){
          navigation_state = OBSTACLE_FOUND;
      }
      else{
          speed = max_speed/(danger_count);
          guidance_h_set_guided_body_vel(speed, 0);
      }
      break;

  case OBSTACLE_FOUND:
      // stop
      guidance_h_set_guided_body_vel(0, 0);
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

  case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_guided_heading_rate(heading_change_rate);
      //yaw
      // if there is no obstacle ahead, change state to SAFE
      break;

  case OUT_OF_BOUNDS:
      // stop
      // change state to REENTER_ARENA
      break;

  case REENTER_ARENA:
      // yaw
      // if there is clear path into arena, chage state to SAFE
      break;

  default:
      break;
  }*/
}
