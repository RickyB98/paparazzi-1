#ifdef __cplusplus
extern "C" {
#endif

#include "horizon_detection.h"
#include "modules/computer_vision/cv.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"


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
    getHorizonArray(img, (int*) horizon_a);

    horizon_line_t best_horizon_l, sec_horizon_l;
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

    int obstacles_a[IMAGE_WIDTH] = {0};

    findObstacles_2H((int*) obstacles_a, (int*) horizon_a, &best_horizon_l, &sec_horizon_l);

    pthread_mutex_lock(&obstacle_mutex);
    memcpy(gl_obstacles, obstacles_a, sizeof(int)*IMAGE_WIDTH);
    pthread_mutex_unlock(&obstacle_mutex);

    if (draw){
        drawHorizon_2H(img, (int*) obstacles_a, &best_horizon_l, &sec_horizon_l);
    }
    return NULL;
}

void horizon_detection_init() {
    HorizonDetectionInit();
    
    // Default values floor filter settings
    cf_ymin = HORIZON_DETECTION_CF_YMIN;
    cf_ymax = HORIZON_DETECTION_CF_YMAX;
    cf_umin = HORIZON_DETECTION_CF_UMIN;
    cf_umax = HORIZON_DETECTION_CF_UMAX;
    cf_vmin = HORIZON_DETECTION_CF_VMIN;
    cf_vmax = HORIZON_DETECTION_CF_VMAX;

    // Default values ransac horizon settings
    ransac_threshold = HORIZON_DETECTION_RANSAC_THRESHOLD;
    ransac_iter = HORIZON_DETECTION_RANSAC_ITER;
    sec_horizon_threshold = HORIZON_DETECTION_SECONDARY_HORIZON_THRESHOLD;
    obstacle_threshold = HORIZON_DETECTION_OBSTACLE_THRESHOLD;
    
    cv_add_to_device(&COMPETITION_CAMERA_FRONT, horizon_detection_callback, 5);
    pthread_mutex_init(&obstacle_mutex, NULL);
}

void horizon_detection_loop() {
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED){
        return;
    }
    int local_obstacles[IMAGE_WIDTH] = {0};
    pthread_mutex_lock(&obstacle_mutex);
    memcpy(local_obstacles, gl_obstacles, sizeof(int)*IMAGE_WIDTH);
    pthread_mutex_unlock(&obstacle_mutex);

    int bestHeading = findBestHeadingDirection((int*) local_obstacles);
    printf("best heading is %d\n", bestHeading);
    HorizonDetectionLoop();
}


int findBestHeadingDirection(int* obstacles){
    bool safe_section = false;
    int best_first = 0;
    int best_last = 0;
    int best_count = 0;
    int current_first = 0;
    int current_count = 0;
    int i;
    for(i=0;i<IMAGE_WIDTH;i++){
        if(safe_section){
            if (obstacles[i]<0){
                // continuing safe section
                current_count++;
            }
            else{
                // safe section ended
                safe_section = false;
                if (current_count>best_count){
                    best_first = current_first;
                    best_last = i-1;
                    best_count = current_count;
                }
            }
        }
        else{
            if (obstacles[i]<0){
                // entering a safe section
                safe_section = true;
                current_first = i;
                current_count = 1;
            }
        }
    }
    if(safe_section && current_count>best_count){
        // last point of the array was safe, and it was the best section
        best_first = current_first;
        best_last = i;
    }
    // return center pixel of safe section
    return (int)(best_last-best_first)/2;
}

#ifdef __cplusplus
}
#endif