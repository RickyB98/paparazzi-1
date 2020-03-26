#include "horizon_detection_cxx.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "horizon_detection.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "modules/computer_vision/cv.h"

// Global variables
int gl_obstacles[IMAGE_WIDTH] = {0};
static pthread_mutex_t obstacle_mutex;


struct image_t * horizon_detection_callback(struct image_t *img){
    if (img->h != IMAGE_WIDTH){
        // IMAGE_WIDTH is the baseline on the image if the horizon was horizontal.
        // Because the image is rotated, this is compared to the height
        printf("IMAGE_WIDTH (%d) does not match real image width (%d)\n", IMAGE_WIDTH, img->h);
        return NULL;
    }

    int horizon_a[IMAGE_WIDTH] = {0};
    printf("getting Horizon Array...\n");
    getHorizonArray(img, (int*) horizon_a);

    horizon_line_t best_horizon_l, sec_horizon_l;
    printf("calculating Horizon...\n");
    ransacHorizon((int*)horizon_a, &best_horizon_l, 0, IMAGE_WIDTH-1);
    if (best_horizon_l.m > 0){
        ransacHorizon((int*)horizon_a, &sec_horizon_l, best_horizon_l.limits[1], IMAGE_WIDTH-1);
    }
    else if (best_horizon_l.m < 0){
        ransacHorizon((int*)horizon_a, &sec_horizon_l, 0, best_horizon_l.limits[1]);
    }
    else{
        // Don't calculate a second horizon
    }
    printf("horizonline 1 is : m=%f, quality=%d\n", best_horizon_l.m, best_horizon_l.quality);
    printf("horizonline 2 is : m=%f, quality=%d\n", sec_horizon_l.m, sec_horizon_l.quality);
    int obstacles_a[IMAGE_WIDTH] = {0};
    printf("Looking for obstacles...\n");
    findObstacles_2H((int*) obstacles_a, (int*) horizon_a, &best_horizon_l, &sec_horizon_l);
    printf("obstacles[10] is %d\n", obstacles_a[10]);
    pthread_mutex_lock(&obstacle_mutex);
    memcpy(gl_obstacles, obstacles_a, sizeof(int)*IMAGE_WIDTH);
    pthread_mutex_unlock(&obstacle_mutex);

    if (draw){
        printf("Drawing the Horizon...\n");
        drawHorizon_2H(img, (int*) obstacles_a, &best_horizon_l, &sec_horizon_l);
        drawHorizonArray(img, (int*) horizon_a);
    }
    return NULL;
}

void horizon_detection_init() {
    HorizonDetectionInit();
    cv_add_to_device(&COMPETITION_CAMERA_FRONT, horizon_detection_callback, 5);
    pthread_mutex_init(&obstacle_mutex, NULL);
}

void horizon_detection_loop() {
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED){
        return;
    }
    HorizonDetectionLoop();
}

#ifdef __cplusplus
}
#endif