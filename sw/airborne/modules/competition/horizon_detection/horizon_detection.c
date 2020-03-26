#ifdef __cplusplus
extern "C" {
#endif

#include "horizon_detection.h"

#include <stdio.h>
#include "modules/computer_vision/cv.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"


// local function declarations
struct image_t * horizon_detection_callback(struct image_t *img);
int findBestHeadingDirection(int* obstacles);
int findCenterHorizonHeight(full_horizon_t *fullHorizon);

// Global variables and get functions
int gl_obstacles[IMAGE_WIDTH] = {0};
static pthread_mutex_t obstacle_mutex;

full_horizon_t gl_fullHorizon;
static pthread_mutex_t horizon_mutex;

void hdGetObstacleArray(int* obstacleArray){
    pthread_mutex_lock(&obstacle_mutex);
    memcpy(obstacleArray, gl_obstacles, sizeof(int)*IMAGE_WIDTH);
    pthread_mutex_unlock(&obstacle_mutex);
}

int hdGetBestHeading(void){
    int loc_obstacles[IMAGE_WIDTH] = {0};
    pthread_mutex_lock(&obstacle_mutex);
    memcpy(loc_obstacles, gl_obstacles, sizeof(int)*IMAGE_WIDTH);
    pthread_mutex_unlock(&obstacle_mutex);

    return findBestHeadingDirection((int*) loc_obstacles);
}

int hdGetHorizonHeight(void){
    full_horizon_t local_horizon;
    pthread_mutex_lock(&horizon_mutex);
    memcpy(&local_horizon,&gl_fullHorizon,sizeof(full_horizon_t));
    pthread_mutex_unlock(&horizon_mutex);

    return findCenterHorizonHeight(&local_horizon);
}


struct image_t * horizon_detection_callback(struct image_t *img){
    if (img->h != IMAGE_WIDTH){
        // IMAGE_WIDTH is the baseline on the image if the horizon was horizontal.
        // Because the image is rotated, this is compared to the height
        printf("IMAGE_WIDTH (%d) does not match real image width (%d)\n", IMAGE_WIDTH, img->h);
        return NULL;
    }

    int horizon_a[IMAGE_WIDTH] = {0};
    getHorizonArray(img, (int*) horizon_a);

    horizon_line_t best_horizon_l = {0.0f, 0.0f, 0, {0, IMAGE_WIDTH-1}}, sec_horizon_l = {0.0f, 0.0f, 0, {0, IMAGE_WIDTH-1}};
    
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
    // calculate and save full horizon
    full_horizon_t fullHorizon = mergeHorizonLines(&best_horizon_l, &sec_horizon_l);
    pthread_mutex_lock(&horizon_mutex);
    memcpy(&gl_fullHorizon, &fullHorizon, sizeof(fullHorizon));
    pthread_mutex_unlock(&horizon_mutex);

    // calculate and save obstacles
    int obstacles_a[IMAGE_WIDTH] = {0};
    findObstaclesFullHorizon((int*) obstacles_a, (int*) horizon_a, &fullHorizon);
    pthread_mutex_lock(&obstacle_mutex);
    memcpy(gl_obstacles, obstacles_a, sizeof(int)*IMAGE_WIDTH);
    pthread_mutex_unlock(&obstacle_mutex);

    if (draw){
        drawFullHorizon(img, (int*) obstacles_a, &fullHorizon);
    }
    return NULL;
}

void horizon_detection_init() {
    HorizonDetectionInit();
    
    gl_fullHorizon.isCompound = false;
    horizon_line_t init = {0.0f, 0.0f, 0, {0, IMAGE_WIDTH-1}};
    gl_fullHorizon.left = init;
    gl_fullHorizon.right = init;
    gl_fullHorizon.main = &gl_fullHorizon.left;
    gl_fullHorizon.intersect = 0;

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
    pthread_mutex_init(&horizon_mutex, NULL);
}

void horizon_detection_loop() {
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED){
        return;
    }
    int bestHeading = hdGetBestHeading();
    int centerHeight = hdGetHorizonHeight();
    printf("best heading: %d, horizon height: %d\n", bestHeading,centerHeight);
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

int findCenterHorizonHeight(full_horizon_t *fullHorizon){
    int height = 0;
    int y = (int) IMAGE_WIDTH/2;
    if (fullHorizon->isCompound){
        if (fullHorizon->intersect > y){
            height = fullHorizon->left.m*y + fullHorizon->left.b;
        }
        else {
            height = fullHorizon->right.m*y + fullHorizon->right.b;
        }
        
    }
    else{
        height = fullHorizon->main->m*y + fullHorizon->main->b;
    }
    
    return height;
}

#ifdef __cplusplus
}
#endif