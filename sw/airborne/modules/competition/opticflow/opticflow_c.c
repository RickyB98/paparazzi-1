#include "opticflow.h"

#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#include <modules/computer_vision/cv.h>
#include <modules/computer_vision/lib/vision/image.h>
#include <modules/computer_vision/lib/vision/lucas_kanade.h>
#include <modules/computer_vision/lib/vision/fast_rosten.h>
#include <modules/computer_vision/opticflow/linear_flow_fit.h>

#include <modules/computer_vision/video_capture.h>

#include <generated/airframe.h>

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"

#include "state.h"
#include "subsystems/abi.h"

#include "math.h"
#include <time.h>

struct image_t *img1 = NULL;
struct image_t *img2 = NULL;

uint16_t num_corners = 0;
uint16_t ret_corners_length = 0;

// Telemetry
uint8_t max_points = 150;
uint8_t pyramid = 2;
uint8_t reset_below_points = 15;
uint8_t fast9_threshold = 45;

float factor = 2000.;

struct point_t *ret_corners;
struct linear_flow_fit_info info;
bool firstRun = true;
bool reset = false;

float xfoeTot = 0, yfoeTot = 0;
float xfoe = 0, yfoe = 0;

#define AVERAGES 60

uint8_t currentAvgIdx = 0;
float xfoeVec[AVERAGES], yfoeVec[AVERAGES];

uint8_t suggested_action = OF_ACT_STRAIGHT;

struct image_t *store_image(struct image_t *img)
{
  if (img1 != NULL)
  {
    free(img1->buf);
    free(img1);
  }
  img1 = img2;
  img2 = (struct image_t *)malloc(sizeof(struct image_t));
  memcpy(img2, img, sizeof(struct image_t));
  image_create(img2, img->w, img->h, IMAGE_GRAYSCALE);
  image_to_grayscale(img, img2);
  parse_images();
  draw_current_corners(img);
  return img;
}

void draw_current_corners(struct image_t *img)
{
  for (int i = 0; i < num_corners; ++i)
  {
    struct point_t p = ret_corners[i];

    for (int dx = -2; dx <= 2; ++dx)
    {
      for (int dy = -2; dy <= 2; ++dy)
      {
        uint16_t x = Max(0, Min(img->w - 1, p.x + dx));
        uint16_t y = Max(0, Min(img->h - 1, p.y + dy));
        uint8_t *yp = &(img->buf[y * 2 * img->w + 2 * x + 1]);
        *yp = 255;
      }
    }
  }
  for (int dx = -2; dx <= 2; ++dx)
  {
    for (int dy = -2; dy <= 2; ++dy)
    {
      //uint16_t x = Max(0, Min(img->w - 1, xfoeTot + dx));
      //uint16_t y = Max(0, Min(img->h - 1, yfoeTot + dy));
      uint16_t x = Max(0, Min(img->w - 1, xfoe / factor + dx));
      uint16_t y = Max(0, Min(img->h - 1, yfoe / factor + dy));
      uint8_t *up = &(img->buf[y * 2 * img->w + 2 * x]);
      *up = 255;
    }
  }
}

// A function to generate a random permutation of arr[] 
void randomize (struct point_t* arr, int n) 
{ 
    // Use a different seed value so that we don't get same 
    // result each time we run this program 
    srand (time(NULL)); 
  
    // Start from the last element and swap one by one. We don't 
    // need to run for the first element that's why i > 0 
    for (int i = n-1; i > 0; i--) 
    { 
        // Pick a random index from 0 to i 
        int j = rand() % (i+1); 
  
        // Swap arr[i] with the element at random index 
        struct point_t p = arr[i];
        arr[i] = arr[j];
        arr[j] = p;
    } 
} 

void parse_images()
{
  if (img1 == NULL)
    return;
  //fprintf(stderr, "BEGIN\n");

  if (firstRun || num_corners < reset_below_points || reset)
  {
    free(ret_corners);
    ret_corners = malloc(sizeof(struct point_t) * max_points);
    num_corners = 0;
    ret_corners_length = max_points;
    firstRun = false;
    reset = false;
    fast9_detect(img1, fast9_threshold, 10, 20, 20, &num_corners, &ret_corners_length, &ret_corners, NULL);
    for (int i = 0; i < num_corners; ++i) {
      ret_corners[i].count = 0;
    }
  }

  //fprintf(stderr, "points before: %d, ", num_corners);
  struct flow_t *flow = opticFlowLK(img2, img1, ret_corners, &num_corners, 5, (uint16_t) factor, 50, 1000, max_points, pyramid, 0);
  //fprintf(stderr, "points after: %d\n", num_corners);

  // updates ret_corners positions after detecting flow

  int k = 0;
  for (int i = 0; i < num_corners; ++i)
  {
    if (flow[i].pos.count > 30) continue;
    ret_corners[k].x = (uint32_t) round((flow[i].pos.x + flow[i].flow_x) / factor);
    ret_corners[k].y = (uint32_t) round((flow[i].pos.y + flow[i].flow_y) / factor);
    //ret_corners[k].x_sub = (uint16_t) ((flow[i].pos.x + flow[i].flow_x) % (uint16_t) factor);
    //ret_corners[k].y_sub = (uint16_t) ((flow[i].pos.y + flow[i].flow_y) % (uint16_t) factor);
    ret_corners[k].count = flow[i].pos.count;
    ++k;
  }
  num_corners = k;
  
  //recentre the origin of the x-y axis

  //get rotational velocities from optitrack
  //struct FloatRates *rates = stateGetBodyRates_f();

  if (num_corners < 2)
    return;

  // computes time to contact and xfoe/yfoe (failing) into info
  analyze_linear_flow_field(flow, num_corners, 100.0f, 100, num_corners, img1->w, img1->h, &info);


  //fprintf(stderr, "[INFO] xfoe: %f, yfoe: %f\n", xfoeTot, yfoeTot);
  //fprintf(stderr, "[INFO] xfoe: %f, yfoe: %f\n", info.focus_of_expansion_x, info.focus_of_expansion_y);
  //fprintf(stderr, "[INFO] TTC: %f, err: %f\n", info.time_to_contact, info.fit_error);

  xfoeVec[currentAvgIdx % AVERAGES] = info.focus_of_expansion_x;
  yfoeVec[currentAvgIdx % AVERAGES] = info.focus_of_expansion_y;

  for (uint8_t i = 0; i < AVERAGES; ++i) {
    xfoe += xfoeVec[i];
    yfoe += yfoeVec[i];
  }
  xfoe /= AVERAGES;
  yfoe /= AVERAGES;

  // OVERRIDE: XFOE AND YFOE CENTER
  xfoe = 120 * factor;
  yfoe = 260 * factor;

  ++currentAvgIdx;

  int d = 0;

  float avg_py = 0;
  int close = 0;
  bool first = true;
  for (int i = 0; i < num_corners; i++)
  {
    //calculate time to contact

    struct point_t p = flow[i].pos;
    float px = (p.x / factor) - 120;
    float py = (p.y / factor) - 260;

    float u = flow[i].flow_x / factor;
    float v = flow[i].flow_y / factor;

    if (py * v < 0) continue;
    if (px * u < 0) continue;
    if (abs(u) < 2 || abs(v) < 2) continue;

    float tt = /*((px - (xfoe / factor - 120)) / u +*/ (py - (yfoe / factor - 260)) / v;// / 2.;

    //fprintf(stderr, "[TTC] %f, %f; %f, %f - ttc: %f\n", px - (xfoe / factor - 120), u, (py - (yfoe / factor - 260)), v, tt);
    if (abs(tt) < 100 && abs(py) < 100)
    {
      close++;
      if (first) {
      //fprintf(stderr, "------\n");
        first = false;
      }
      //fprintf(stderr, "[TTC] %f, %f - ttc: %f\n", px, py, tt);
      avg_py += (py >= 0 ? 1 : -1) * 1 / (1 + 1/50. * abs(py));
    }
  }
  if (close > 1) {
    if (avg_py > 0) {
      suggested_action = OF_ACT_LEFT;
      //fprintf(stderr, "[OF_ACT] Go left (num: %d, py: %f)\n", close, avg_py);
    } else {
      suggested_action = OF_ACT_RIGHT;
      //fprintf(stderr, "[OF_ACT] Go right (num: %d, py: %f)\n", close, avg_py);
    }
  } else {
    suggested_action = OF_ACT_STRAIGHT;
    //fprintf(stderr, "[OF_ACT] Go straight\n");
  }

}

uint8_t of_get_suggested_action() {
  return suggested_action;
}