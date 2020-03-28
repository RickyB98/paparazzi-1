#ifdef __cplusplus
extern "C" {
#endif

#include "opticflow.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <modules/computer_vision/cv.h>
#include <modules/computer_vision/lib/vision/fast_rosten.h>
#include <modules/computer_vision/lib/vision/image.h>
#include <modules/computer_vision/lib/vision/lucas_kanade.h>
#include <modules/computer_vision/opticflow/linear_flow_fit.h>

#include <modules/computer_vision/video_capture.h>

#include <generated/airframe.h>

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"

#include "state.h"
#include "subsystems/abi.h"

#include "math.h"
#include <time.h>

#define ABS(x) (((x) < 0 ? -(x) : (x)))

#define MAX_TRACK 30

#ifndef OPTICFLOW_DRAW
#define OPTICFLOW_DRAW TRUE
#endif

#define OPTICFLOW_TIMING

typedef struct point_tracking_s {
  int count;
  struct flow_t flows[MAX_TRACK];
  uint32_t dts[MAX_TRACK];
  bool valid;
} point_tracking_t;

struct image_t *img1 = NULL;
struct image_t *img2 = NULL;

uint16_t num_corners = 0;
uint16_t ret_corners_length = 0;

// Telemetry
uint8_t max_points = 150;
uint8_t pyramid = 2;
uint8_t reset_below_points = 20;
uint8_t fast9_threshold = 15;

float factor = 2000.;

struct point_t *ret_corners;

point_tracking_t *tracking;

uint16_t vcc = 0;

bool firstRun = true;
bool reset = false;

float xfoe = 0, yfoe = 0;

#define AVERAGES 60

uint8_t currentAvgIdx = 0;
float xfoeVec[AVERAGES], yfoeVec[AVERAGES];

uint8_t suggested_action = OF_ACT_STRAIGHT;


void opticflow_init() {
  cv_add_to_device(&COMPETITION_CAMERA_FRONT, store_image, 25);
  OpticflowInit();
}

void opticflow_loop() { OpticflowLoop(); }

void opticflow_reset() { reset = true; }

struct image_t *store_image(struct image_t *img) {
  #ifdef OPTICFLOW_TIMING
  clock_t tic = clock();
  #endif
  if (img1 != NULL) {
    free(img1->buf);
    free(img1);
  }
  img1 = img2;
  img2 = (struct image_t *)malloc(sizeof(struct image_t));
  // memcpy(img2, img, sizeof(struct image_t));
  image_create(img2, img->w, img->h, IMAGE_GRAYSCALE);
  image_to_grayscale(img, img2);
  img2->pprz_ts = img->pprz_ts;
  img2->ts = img->ts;
  img2->eulers = img->eulers;
  
  int positive_points_size = 30;
  struct point_t *positive_points = malloc(positive_points_size * sizeof(struct point_t));
  
  parse_images((struct point_t **)&positive_points, &positive_points_size);
  draw_current_corners(img, (struct point_t *)positive_points,
                       positive_points_size);
  free(positive_points);
  #ifdef OPTICFLOW_TIMING
  clock_t toc = clock();
  double timing = ((double) (toc - tic)) / CLOCKS_PER_SEC;
  printf("[OF_TIMING] %f (max freq: %f)\n", timing, 1./timing);
  #endif
  return img;
}

void draw_current_corners(struct image_t *img, struct point_t *positive_points,
                          int positive_points_size) {
#if OPTICFLOW_DRAW
  for (int i = 0; i < num_corners; ++i) {
    if (!tracking[i].valid)
      continue;
    struct point_t p = ret_corners[i];

    for (int dx = -2; dx <= 2; ++dx) {
      for (int dy = -2; dy <= 2; ++dy) {
        uint16_t x = Min(img->w - 1, p.x + dx);
        uint16_t y = Min(img->h - 1, p.y + dy);
        uint8_t *yp = (uint8_t *)&(img->buf[y * 2 * img->w + 2 * x + 1]);
        *yp = 255;
      }
    }
  }
  //printf("positive_points_size: %d\n", positive_points_size);
  for (int i = 0; i < positive_points_size; ++i) {
    for (int dx = -2; dx <= 2; ++dx) {
      for (int dy = -2; dy <= 2; ++dy) {
        uint16_t x = Min(img->w - 1, positive_points[i].x + dx);
        uint16_t y = Min(img->h - 1, positive_points[i].y + dy);
        uint8_t *vp = (uint8_t *)&(img->buf[y * 2 * img->w + 2 * x]);
        *vp = 255;
      }
    }
  }
  for (int dx = -2; dx <= 2; ++dx) {
    for (int dy = -2; dy <= 2; ++dy) {
      uint16_t x = Max(0, Min(img->w - 1, xfoe / factor + dx));
      uint16_t y = Max(0, Min(img->h - 1, yfoe / factor + dy));
      uint8_t *up = (uint8_t *)&(img->buf[y * 2 * img->w + 2 * x]);
      *up = 255;
    }
  }
#endif
}

void parse_images(struct point_t **positive_points, int *positive_points_size) {
  if (img1 == NULL) {
    *positive_points_size = 0;
    return;
  }

  if (firstRun || vcc < reset_below_points || reset) {
    if (ret_corners != NULL) {
      free(ret_corners);
      ret_corners = NULL;
    }
    if (tracking != NULL) {
      free(tracking);
      tracking = NULL;
    }
    ret_corners_length = 20;
    ret_corners = malloc(ret_corners_length * sizeof(struct point_t));
    num_corners = 0;
    firstRun = false;
    reset = false;
    #ifdef OPTICFLOW_TIMING
    clock_t fast9_tic = clock();
    #endif
    fast9_detect(img1, fast9_threshold, 20, 30, 30, &num_corners,
                 &ret_corners_length, &ret_corners, NULL);
    #ifdef OPTICFLOW_TIMING
    clock_t fast9_toc = clock();
    double fast9_time = (double) (fast9_toc - fast9_tic) / CLOCKS_PER_SEC;
    printf("[OF_LK] %f - max freq: %f\n", fast9_time, 1/fast9_time);
    #endif

    //printf(stderr, "done fast9\n");
    if (num_corners <= 0) {
      reset = true;
      *positive_points_size = 0;
      return;
    }

    tracking = malloc(num_corners * sizeof(point_tracking_t));
    // num_corners = ret_corners_length;
    /*rintf("max points: %d, ret_corners_length: %d, num_corners: %d\n"
           max_points, ret_corners_length, num_corners);*/

    for (int i = 0; i < num_corners; ++i) {
      ret_corners[i].count = 0;
      tracking[i].count = 0;
      tracking[i].valid = true;
    }
  }

  struct point_t *valid_corners = malloc(num_corners * sizeof(struct point_t));
  int *valid_corners_idx = malloc(num_corners * sizeof(int));

  vcc = 0;
  for (int i = 0; i < num_corners; ++i) {
    if (tracking[i].valid) {
      int idx = vcc++;
      valid_corners_idx[idx] = i;
      valid_corners[idx] = ret_corners[i];
    }
  }

  // printf("points before: %d, ", num_corners);
  #ifdef OPTICFLOW_TIMING
  clock_t lk_tic = clock();
  #endif
  struct flow_t *flow =
      opticFlowLK(img2, img1, valid_corners, &vcc, 5, (uint16_t)factor, 50, 250,
                  max_points, pyramid, 1);
  #ifdef OPTICFLOW_TIMING
  clock_t lk_toc = clock();
  double lk_time = (double) (lk_toc - lk_tic) / CLOCKS_PER_SEC;
  printf("[OF_LK] %f - max freq: %f\n", lk_time, 1/lk_time);
  #endif

  // updates ret_corners positions after detecting flow

  for (int i = 0; i < vcc; ++i) {
    int idx = valid_corners_idx[i];
    if (flow[i].error == LARGE_FLOW_ERROR || flow[i].pos.count >= 60) {
      tracking[idx].valid = false;
      continue;
    }

    if (tracking[idx].count >= MAX_TRACK) {
      for (int p = 1; p < MAX_TRACK; ++p) {
        tracking[idx].flows[p - 1] = tracking[idx].flows[p];
        tracking[idx].dts[p - 1] = tracking[idx].dts[p];
      }
      --tracking[idx].count;
    }

    int x = tracking[idx].count++;

    tracking[idx].flows[x] = flow[i];
    tracking[idx].dts[x] = img2->pprz_ts - img1->pprz_ts;
    ret_corners[idx].x =
        (uint32_t)round((flow[i].pos.x + flow[i].flow_x) / factor);
    ret_corners[idx].y =
        (uint32_t)round((flow[i].pos.y + flow[i].flow_y) / factor);
    ret_corners[idx].count = flow[i].pos.count;
  }

  free(valid_corners);
  free(valid_corners_idx);

  // printf("points after: %d\n", vcc);

  // OVERRIDE: XFOE AND YFOE CENTER
  float offset0 = 65; // 60
  float theta0 = -10 / 180. * M_PI; // 10/180

  struct FloatEulers *eulers = stateGetNedToBodyEulers_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  float front_speed = sqrt(pow(vel->x, 2) + pow(vel->y, 2) + pow(vel->z, 2));
  if (stateIsSideslipValid()) {
    front_speed *= cos(stateGetSideslip_f());
  }
  //printf("front_speed: %f\n", front_speed);

  xfoe = (120 + offset0 + (eulers->theta - theta0) * offset0 / theta0) * factor;
  yfoe = 260 * factor;

  float avg_py = 0;
  int close = 0;
  bool first = true;

  for (int i = 0; i < num_corners; i++) {
    // calculate time to contact
    if (!tracking[i].valid)
      continue;

    float x_dist = 0;
    float y_dist = 0;

    float px0 = (tracking[i].flows[0].pos.x / factor) - 120;
    float py0 = (tracking[i].flows[0].pos.y / factor) - 260;
    float tx = 0;
    float ty = 0;

    float dt = 0;
    // printf(stderr, "tracking[%d].count = %d\n", i, tracking[i].count);

    int to_average = 0;
    for (int k = 0; k < tracking[i].count; ++k) {
      struct flow_t flow_k = tracking[i].flows[k];
      struct point_t p = flow_k.pos;

      float px = (p.x / factor) - 120;
      float py = (p.y / factor) - 260;

      float u = 0;
      float v = 0;
      dt = 0;
      for (int j = 0; j <= k; ++j) {
        u += tracking[i].flows[j].flow_x;
        v += tracking[i].flows[j].flow_y;
        dt += tracking[i].dts[j];
      }
      u /= factor;
      v /= factor;
      dt /= 1e6;

      if (ABS(u) < 1e-2 || ABS(v) < 1e-2) {
        continue;
      }
      if (u * px < 0 || v * py < 0) {
        continue;
      }

      ++to_average;
      float magic = front_speed * dt; 

      // printf("px0: %f, px: %f, py0: %f, py: %f, k: %d, dt: %f, magic: %f,
      // speed: %f\n", px0, px, py0, py, k, dt, magic, front_speed);

      x_dist += px0 * px * magic / 120. / u;
      y_dist += py0 * py * magic / 260. / v;

      tx += (px - (xfoe / factor - 120)) / u * magic / front_speed;
      ty += (py - (yfoe / factor - 260)) / v * magic / front_speed;
    }

    if (to_average == 0) {
      //fprintf(stderr, "Nothing to average.\n");
      goto stop;
      return;
    }

    x_dist /= to_average;
    y_dist /= to_average;
    tx /= to_average;
    ty /= to_average;

    tx -= dt;
    ty -= dt;
   
    //fprintf(stderr,"x_dist  = %f, y_dist  = %f, mean TTC = %f \n", x_dist, y_dist, lala);
    if (ABS(y_dist) < .3 && ty > 0 && tx > 0 && ABS(x_dist) < .4) {
      //fprintf(stderr, "y_dist lower than treshold");
      if (first) {
        //fprintf(stderr, "+-+-+-+-+-+-+-+-+-+-+\n");
        first = false;
      }
      //fprintf(stderr, "[TTC] y: %f, tx: %f, ty: %f\n", y_dist, tx, ty);
      if ((front_speed * ty < 2 || front_speed * tx < 1) && to_average >= 12) { // to_average >= 10
        ++close;
        //fprintf(stderr, "POINT [%d] mean TTC: %f = ", i, lala); //check
        /* if (front_speed * ty < 1.2){
          fprintf(stderr, "Decision based on ------->> Y \n");
        }
        if (front_speed * tx < .5){
          fprintf(stderr, "Decision based on ------->> X \n");
        } */
        if (close - 1 < *positive_points_size) {
          struct point_t used = tracking[i].flows[tracking[i].count - 1].pos;
          used.x /= (uint32_t)factor;
          used.y /= (uint32_t)factor;
          (*positive_points)[close - 1] = used;
        }

        avg_py += (y_dist >= 0 ? 1 : -1) * 1 / (1 + 2 * ABS(y_dist));
      }
    }
  stop:
    continue;
  }
  //fprintf(stderr, "Points counted: %d", close);

  if (close > 0) {
    if (avg_py > 0) {
      suggested_action = OF_ACT_LEFT;
      //fprintf(stderr, "Average py: %f\n", avg_py);
      //fprintf(stderr, "[OF_ACT] Go left (num: %d, py: %f)\n", close, avg_py);
    } else {
      suggested_action = OF_ACT_RIGHT;
      //fprintf(stderr, "Average py: %f\n", avg_py);
      //fprintf(stderr, "[OF_ACT] Go right (num: %d, py: %f)\n", close, avg_py);
    }
  } else {
      suggested_action = OF_ACT_STRAIGHT;
      //fprintf(stderr, "[OF_ACT] Go straight\n");
  }

  *positive_points_size = Min(close, *positive_points_size);
}

uint8_t of_get_suggested_action() { return suggested_action; }

#ifdef __cplusplus
}
#endif