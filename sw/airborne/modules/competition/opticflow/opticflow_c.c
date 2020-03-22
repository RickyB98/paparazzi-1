#include "opticflow.h"

#include <stdio.h>

#include <modules/computer_vision/cv.h>
#include <modules/computer_vision/lib/vision/image.h>
#include <modules/computer_vision/lib/vision/lucas_kanade.h>
#include <modules/computer_vision/lib/vision/fast_rosten.h>

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

struct image_t *store_image(struct image_t *img)
{
  if (img1 != NULL)
  {
    free(img1->buf);
    free(img1);
  }
  img1 = img2;
  img2 = (struct image_t *) malloc(sizeof(struct image_t));
  memcpy(img2, img, sizeof(struct image_t));
  image_create(img2, img->w, img->h, IMAGE_GRAYSCALE);
  image_to_grayscale(img, img2);
  parse_images();
  return NULL;
}

#define MAX_POINTS 35

uint16_t num_corners = 0;
uint16_t ret_corners_length = 0;

struct point_t* ret_corners;
bool firstRun = true;

void parse_images()
{
  if (img1 == NULL)
    return;
  //fprintf(stderr, "BEGIN\n");
  if (firstRun || num_corners < 5)
  {
    free(ret_corners);
    ret_corners = malloc(sizeof(struct point_t) * MAX_POINTS);
    num_corners = 0;
    ret_corners_length = MAX_POINTS;
    firstRun = false;
    fast9_detect(img1, 20, 10, 20, 20, &num_corners, &ret_corners_length, &ret_corners, NULL);
  }

  /* for (int i = 0; i < num_corners; ++i) {
      fprintf(stderr, "[POINT] pos: %d, %d\n", ret_corners[i].x, ret_corners[i].y);
    } */

  //fprintf(stderr, "points before: %d, ", num_corners);

  struct flow_t *flow = opticFlowLK(img2, img1, ret_corners, &num_corners, 5, 10, 10, 2, MAX_POINTS, 2, 0);
  //fprintf(stderr, "points after: %d\n", num_corners);
  /* for (int i = 0; i < num_corners; ++i)
    {
      struct point_t pos = flow[i].pos;
      int16_t flow_x = flow[i].flow_x;
      int16_t flow_y = flow[i].flow_y;

      fprintf(stderr, "[FLOW] pos: %f, %f - flow_x: %f - flow_y: %f\n", pos.x / 10., pos.y / 10., flow_x / 10., flow_y / 10.);
    } */

  //recentre the origin of the x-y axis

  //get rotational velocities from optitrack
  struct FloatRates *rates = stateGetBodyRates_f();

  float xfoeTot = 0, yfoeTot = 0;
  for (int i = 0; i < num_corners / 2; ++i)
  {
    struct point_t p1 = flow[i].pos;
    float p1y = (p1.x / 10.) - 120;
    float p1x = (p1.y / 10.) - 260;
    float f1y = flow[i].flow_x;
    float f1x = flow[i].flow_y;
    struct point_t p2 = flow[i + num_corners / 2].pos;
    float p2y = (p2.x / 10.) - 120;
    float p2x = (p2.y / 10.) - 260;
    float f2y = flow[i + num_corners / 2].flow_x;
    float f2x = flow[i + num_corners / 2].flow_y;

    float u1 = f1x; // - rates->r - rates->p * p1x + rates->q * p1y * p1x - rates->r * pow(p1x, 2);
    //float cu1 = - rates->r - rates->p * p1x + rates->q * p1y * p1x - rates->r * pow(p1x, 2);
    float v1 = f1y; // - rates->r * p1x + rates->q + rates->q * pow(p1x, 2) - rates->r * p1x * p1y;
    //float cv1 = - rates->r * p1x + rates->q + rates->q * pow(p1x, 2) - rates->r * p1x * p1y;

    float u2 = f2x; //- rates->r - rates->p * p2x + rates->q * p2y * p2x - rates->r * pow(p2x, 2);
    //float cu2 =- rates->r - rates->p * p2x + rates->q * p2y * p2x - rates->r * pow(p2x, 2);
    float v2 = f2y; // - rates->r * p2x + rates->q + rates->q * pow(p2x, 2) - rates->r * p2x * p2y;
    //float cv2 = - rates->r * p2x + rates->q + rates->q * pow(p2x, 2) - rates->r * p2x * p2y;

    //fprintf(stderr, "[couple] u1: %f, v1: %f, u2: %f, v2: %f \n", f1x, f1y, f2x, f2y);
    float det = (v2 * u1 - u2 * v1);
    if (det == 0)
    {
      det = 1;
      fprintf(stderr, "++++++++++++++Determinant to small++++++++++++++++++\n");
    }

    //float xfoe = (u2 * p2y - v2 * p2x - u2 * p1y + (p1x * u2 * v1) / u1) * (u1 / (u2 * v1 - u1 * v2));
    xfoeTot += (-u2 * (v1 * p1x - u1 * p1y) + u1 * (v2 * p2x - u2 * p2y)) / det;
    yfoeTot += (-v2 * (v1 * p1x - u1 * p1y) + v1 * (v2 * p2x - u2 * p2y)) / det;
    //fprintf(stderr, "Print determinant: %f \n", det);
    // fprintf(stderr, "elemento 1: %f, elemento 2: %f      \n ",(-u2 * (v1 * p1x - u1 * p1y) + u1 * (v2 * p2x - u2 * p2y)),(-v2 * (v1 * p1x - u1 * p1y) + v1 * (v2 * p2x - u2 * p2y)));
  }
  fprintf(stderr, "Working, num corner: %d    \n", num_corners);
  xfoeTot /= num_corners / 2;
  yfoeTot /= num_corners / 2;

  if ((xfoeTot < 260) && (xfoeTot > -260) && (yfoeTot < 120) && (yfoeTot > -120))
  {
    fprintf(stderr, "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- FOE sensato +--+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ \n");
  }

  //fprintf(stderr, "[rates] p: %f, q: %f, r: %f\n", rates->p, rates->q, rates->r);
  fprintf(stderr, "xfoeTot: %f, yfoeTot: %f\n", xfoeTot, yfoeTot);

  //float tt[25];
  //int indx[25];
  //int j = 0;
  int d = 0;
  for (int i = 0; i < num_corners; i++)
  {
    //calculate time to contact

    struct point_t p = flow[i].pos;
    float px = (p.y / 10.) - 260;
    float py = (p.x / 10.) - 120;
    float v = flow[i].flow_x;
    float u = flow[i].flow_y;
    float tt = ((px - xfoeTot) / u + (py - yfoeTot) / v) / 2;
    fprintf(stderr, "Time to contact: %f \n", tt);

    if ((tt > -4) && (tt < 4) && (px < -40) && (px > 40))
    {
      if (px > 0)
      {
        d++;
      }
      if (px < 0)
      {
        d--;
      }
      //indx(j) = i;
      //j++;
    }
  }
  fprintf(stderr, "Direction: %d ùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùùù\n", d);
  if (d > 0)
  {
    fprintf(stderr, "Stop and turn right \n");
  }

  if (d < 0)
  {
    fprintf(stderr, "Stop and turn left \n");
  }
}