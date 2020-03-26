#ifndef PAPARAZZI_HORIZON_DETECTION_H
#define PAPARAZZI_HORIZON_DETECTION_H

extern void horizon_detection_init();
extern void horizon_detection_loop();

/**
 * Access the obstacle array calculated with horizon detection
 * For each pixel on the real-world horizontal axis
 * the array contains the height in the image at which the obstacle
 * starts. If the path is clear, it contains the value "-1" 
 * @param[in] obstacleArray: Pointer to an array in which to return the values 
 * must be of size IMAGE_WIDTH 
 */
void hdGetObstacleArray(int* obstacleArray);

/**
 * Returns the central (real-world horizontal) pixel of the widest area without obstacle
 * in the image (image horizontal dimension is IMAGE_WIDTH)
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
#endif //PAPARAZZI_HORIZON_DETECTION_H
