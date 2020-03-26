#ifndef PAPARAZZI_HORIZON_DETECTION_H
#define PAPARAZZI_HORIZON_DETECTION_H

#include <modules/computer_vision/lib/vision/image.h>

struct image_t *horizon_detection_callback(struct image_t *img);

extern void horizon_detection_init();
extern void horizon_detection_loop();

int findBestHeadingDirection(int *obstacles);

/**
 * Access the obstacle array calculated with horizon detection
 * For each pixel on the real-world horizontal axis
 * the array contains the height in the image at which the obstacle
 * starts. If the path is clear, it contains the value "-1"
 * @param[in] obstacleArray: Pointer to an array in which to return the values
 * must be of size IMAGE_WIDTH
 */
void hdGetObstacleArray(int *obstacleArray);

/**
 * Returns the central (real-world horizontal) pixel of the widest area without
 * obstacle in the image (image horizontal dimension is IMAGE_WIDTH)
 * @return[out] central pixel of the widest obstacle free area
 */
int hdGetBestHeading(void);

/**
 * Calculate the height of the horizon in the real-world horizontal
 * middle of the image
 * @return[out] height (in pixel) of the horizon in the middle of the image
 */
int hdGetHorizonHeight(void);

// color filter settings
extern uint8_t cf_ymin;
extern uint8_t cf_ymax;
extern uint8_t cf_umin;
extern uint8_t cf_umax;
extern uint8_t cf_vmin;
extern uint8_t cf_vmax;

// RANSAC Horizon settings
extern bool draw;
extern uint8_t ransac_threshold;
extern uint8_t ransac_iter;
extern uint8_t sec_horizon_threshold;
extern uint8_t obstacle_threshold;

#define IMAGE_WIDTH 520
typedef struct Dot {
  float x;
  float y;
} Dot;

typedef struct horizon_line_s {
  float m;       // = 0;
  float b;       // = 0;
  int quality;   // = 0;
  int limits[2]; // = {0, IMAGE_WIDTH-1};
} horizon_line_t;

typedef struct full_horizon_s {
  bool isCompound; // = false;
  horizon_line_t left;
  horizon_line_t right;
  horizon_line_t *main; // = &this->left;
  int intersect;        // = 0;
} full_horizon_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Find the edges that limit the floor area to the top
 * from a YUV422 image.
 *
 * @param[in] img: the input image
 * @param[in] horizon: the array used to store the floor limits
 */
void getHorizonArray(struct image_t *img, int *horizon);

/**
 * Use RANSAC to find the best line to approximate the horizon
 * within the limits in the horizon array
 *
 * @param[in] horizon_array: array returned by getHorizonArray()
 * @param[in] best_horizon_line: the line that best approximates horizon_array
 * within the limits
 * @param[in] limit_left: first (leftmost) pixel to consider
 * @param[in] limit_right: last (rightmost) pixel to consider
 */
void ransacHorizon(int *horizon_array, horizon_line_t *best_horizon_line,
                   int limit_left, int limit_right);

/**
 * Determine obstacles based on the horizon array and two provided horizon
 * lines. Decides if both lines should be used, based on the line's qualities.
 * Obstacles are returned in the obstacles_array, where the value indicates the
 * pixel at which the obstacle starts. -1 indicates that there is no obstacle.
 *
 * @param[in] obstacles_array: pointer to the array in which the obstacles will
 * be put.
 * @param[in] horizon_array: the array that contains the edges that limit the
 * floor vertically
 * @param[in] horizon_line1: first horizon line to consider
 * @param[in] horizon_line2: second horizon line to consider
 */
void findObstacles_2H(int *obstacles_array, int *horizon_array,
                      horizon_line_t *horizon_line1,
                      horizon_line_t *horizon_line2);

/**
 * Determine obstacles based on the horizon array and one provided horizon line.
 * Obstacles are returned in the obstacles_array, where the value indicates the
 * pixel at which the obstacle starts. -1 indicates that there is no obstacle.
 *
 * @param[in] obstacles_array: pointer to the array in which the obstacles will
 * be put.
 * @param[in] horizon_array: the array that contains the edges that limit the
 * floor vertically
 * @param[in] horizon_line: horizon line to consider
 */
void findObstacles_1H(int *obstacles_array, int *horizon_array,
                      horizon_line_t *horizon_line);

/**
 * Draws the horizon lines on the image, using green for clear path and red if
 * obstacles were detected. Decides if both lines should be used depending on
 * quality, using the same criteria as findObstacles_2H()
 *
 * @param[in] img: image to draw on
 * @param[in] obstacles_array: pointer to the array which contains obstacle
 * information
 * @param[in] horizon_line1: first horizon line to consider
 * @param[in] horizon_line2: second horizon line to consider
 */
void drawHorizon_2H(struct image_t *img, int *obstacles_array,
                    horizon_line_t *horizon_line1,
                    horizon_line_t *horizon_line2);

/**
 * Draws the horizon line on the image, using green for clear path and red if
 * obstacles were detected.
 *
 * @param[in] img: image to draw on
 * @param[in] obstacles_array: pointer to the array which contains obstacle
 * information
 * @param[in] horizon_line: horizon line to consider
 */
void drawHorizon_1H(struct image_t *img, int *obstacles_array,
                    horizon_line_t *horizon_line);

/**
 * WARNING: Doesn't seem to work at the moment, removes other drawing from img
 * Highlights the limits of the floor that were detected and stored in the
 * horizon_array
 *
 *
 * @param[in] img: image to draw on
 * @param[in] horizon_array: pointer to the array which contains obstacle
 * information
 */
void drawHorizonArray(struct image_t *img, int *horizon_array);

full_horizon_t mergeHorizonLines(horizon_line_t *horizon1, horizon_line_t *horizon2);

void findObstaclesFullHorizon(int *obstacleArray, int *horizonArray, full_horizon_t *fullHorizon);

void drawFullHorizon(struct image_t *img, int *obstacles_array, full_horizon_t *horizon);

void HorizonDetectionInit();
void HorizonDetectionLoop();

#ifdef __cplusplus
}
#endif

#endif // PAPARAZZI_HORIZON_DETECTION_H