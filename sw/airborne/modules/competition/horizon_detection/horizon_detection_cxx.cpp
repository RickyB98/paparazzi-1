#include "horizon_detection_cxx.hpp"

#include <iostream>
#include <modules/computer_vision/cv.h>
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/colorfilter.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv_contour.h>

#include <string>

using namespace std;
using namespace cv;

#define IMAGE_WIDTH 520
#define RANSAC_TIMEOUT_LIM 10
#define BEST_HORIZON_QUAL_THRESHOLD 300



struct contour_estimation cont_est;
struct contour_threshold cont_thres;

typedef struct horizon_line_s {
    float m = 0;
    float b = 0;
    int quality = 0;
    int limits[2] = {0, IMAGE_WIDTH-1};
} horizon_line_t;

/**
 * Estimates the horizon line from an array of horizon candidates using RANSAC.
 * @param[in] horizon The Array of horizon candidates
 * @param[in] best_horizon a horizon_line struct that defines the limits within which 
 * horizon candidates are considered. After returning, contains m, b and quality of the best horizon.
 */
void ransacHorizon(int *horizon, horizon_line_t *best_horizon);
void findObstacles_2H(int *obstacles, int *horizon,horizon_line_t *left_horizon, horizon_line_t *right_horizon);
void findObstacles_1H(int *obstacles, int *horizon, horizon_line_t *best_horizon);
void findObstacles(int *obstacles, int *horizon, horizon_line_t *horizon_line, int limit_left, int limit_right);
void drawHorizon_2H(struct image_t *img, int *obstacles, horizon_line_t *left_horizon, horizon_line_t *right_Horizon);
void drawHorizon_1H(struct image_t *img, int *obstacles, horizon_line_t *best_horizon);
void drawHorizon(struct image_t *img, int *obstacles, horizon_line_t *horizon, int limit_left, int limit_right);
void drawHorizonArray(struct image_t *img, int *horizon);
RNG rng;

// Color filter settings
uint8_t cf_ymin = 0;
uint8_t cf_ymax = 0;
uint8_t cf_umin = 0;
uint8_t cf_umax = 0;
uint8_t cf_vmin = 0;
uint8_t cf_vmax = 0;
// RANSAC Horizon settings
bool draw = true;
uint8_t ransac_threshold = 1;
uint8_t ransac_iter = 1;
uint8_t sec_horizon_threshold = 1;
uint8_t obstacle_threshold = 1;

Mat image_edges(struct image_t *img)
{
    // Create a new image, using the original bebop image.
    Mat M(img->h, img->w, CV_8UC2, img->buf); // original
    Mat image, edge_image, thresh_image;

    // convert UYVY in paparazzi to YUV in opencv
    cvtColor(M, M, CV_YUV2GRAY_Y422);
    //cvtColor(M, M, CV_RGB2YUV);

    // Threshold all values within the indicted YUV values.
    inRange(M, Scalar(cont_thres.lower_y, cont_thres.lower_u, cont_thres.lower_v), Scalar(cont_thres.upper_y, cont_thres.upper_u, cont_thres.upper_v), thresh_image);

    /// Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    //edge_image = thresh_image;
    edge_image = M;
    int edgeThresh = 35;
    Canny(edge_image, edge_image, edgeThresh, edgeThresh * 3);
    findContours(edge_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
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

bool isFloor(struct image_t *img, int x, int y)
{
    // odd pixels are uy
    // even pixels are vy

    // Is the pixel inside the image?
    if (x < 0 || x >= img->w -50 || y < 0 || y >= img->h)
    {
        return false;
    }

    // Take the right place in the buffer:
    uint8_t *buf = (uint8_t *)(img->buf);
    uint8_t *yp, *up, *vp;
    if (x % 2 == 0) {
    // Even x
    up = &buf[y * 2 * img->w + 2 * x];      // U
    yp = &buf[y * 2 * img->w + 2 * x + 1];  // Y1
    vp = &buf[y * 2 * img->w + 2 * x + 2];  // V
    //yp = &buf[y * 2 * img->w + 2 * x + 3]; // Y2
    } else {
    // Uneven x
    up = &buf[y * 2 * img->w + 2 * x - 2];  // U
    //yp = &buf[y * 2 * img->w + 2 * x - 1]; // Y1
    vp = &buf[y * 2 * img->w + 2 * x];      // V
    yp = &buf[y * 2 * img->w + 2 * x + 1];  // Y2
    }


    if ((*yp >= cf_ymin) && (*yp <= cf_ymax) && 
        (*up >= cf_umin) && (*up <= cf_umax) && 
        (*vp >= cf_vmin) && (*vp <= cf_vmax))
    {
        // the pixel passes:
        return true;
    }
    else
    {
        // the pixel does not:
        return false;
    }
    
}

Mat findHorizonCandidate(struct image_t *img, Mat *edge_image, Dot *p)
{
    
    int x;
    bool onFloor;
    int onEdge;
    int y;
    Mat track(img->h, img->w, CV_8UC1);
    x = p->x;
    y = p->y;
    
    onFloor = isFloor(img, y, x);
    
    track.data[y * img->w + x] = 255;
        //cout << "nnnnnn" << endl;
    while (!onFloor && x < img->w)
    {
        x++;
        onFloor = isFloor(img, y, x);
        track.data[y * img->w + x] = 255;
    }
    // move up to closest edge
    onEdge = (edge_image->data[y * edge_image->cols + x] > 0);
    while (onEdge == 0 && x < img->w- 50 - 1)
    {
        x++;
        onEdge = (edge_image->data[y * edge_image->cols + x] > 0);
        track.data[y * img->w + x] = 255;
    }
    p->x = x;
    p->y = y;
    return track;
}

int followHorizonLeft(Mat *edge_image, Dot *p, int y_lim, int *horizon)
{

    int x;
    int y;
    x = p->x;
    y = p->y;

    while (y > y_lim && x > 0 && x < edge_image->cols + 1)
    {
        y--;
        if (edge_image->data[y * edge_image->cols + x] > 0)
        { // edge continues right
        
        }
        else if (edge_image->data[y * edge_image->cols + x - 1] > 0)
        { // edge continues bottom right
            x--;
        }
        else if (edge_image->data[y * edge_image->cols + x + 1] > 0)
        { //edge continues top right
            x--;
        }
        else
        { //increase step
            y--;
            if (edge_image->data[y * edge_image->cols + x] > 0)
            {
                horizon[y + 1] = x;
            }
            else if (edge_image->data[y * edge_image->cols + x - 1] > 0)
            {
                horizon[y + 1] = x;
                x--;
            }
            else if (edge_image->data[y * edge_image->cols + x + 1] > 0)
            {
                horizon[y + 1] = x;
                x++;
            }
            else
            {
                y = y + 2; //if the edge does not continue, revert to last know edge position
                break;
            }
        }
        horizon[y] = x;
    }
    return y;
}

int followHorizonRight(Mat *edge_image, Dot *p, int *horizon)
{

    int x;
    int y;
    x = p->x;
    y = p->y;

     
    while (y < edge_image->rows - 1 && x > 0 && x < edge_image->cols+1)
    {
        y++;
        if (edge_image->data[y * edge_image->cols + x] > 0)
        { // edge continues right
            
        }
        else if (edge_image->data[y * edge_image->cols + x - 1] > 0)
        { // edge continues bottom right
            x--;
        }
        else if (edge_image->data[y * edge_image->cols + x - 2] > 0)
        { // edge continues bottom right
            x=x-2;
        }
        else if (edge_image->data[y * edge_image->cols + x + 1] > 0)
        { // edge continues top right
            x++;
        }
        else if (edge_image->data[y * edge_image->cols + x + 2] > 0)
        { // edge continues top right
            x=x+2;
        }
        else
        { // increase step
            y++;
            if (edge_image->data[y * edge_image->cols + x] > 0)
            {
                horizon[y - 1] = x;
            }
            else if (edge_image->data[y * edge_image->cols + x - 1] > 0)
            {
                horizon[y - 1] = x;
                x--;
            }
            else if (edge_image->data[y * edge_image->cols + x - 2] > 0)
            {
                horizon[y - 1] = x;
                x=x-2;
            }
            else if (edge_image->data[y * edge_image->cols + x + 1] > 0)
            {
                horizon[y - 1] = x;
                x++;
            }
            else if (edge_image->data[y * edge_image->cols + x + 2] > 0)
            {
                horizon[y - 1] = x;
                x=x+2;
            }
            else if (edge_image->data[(y+1) * edge_image->cols + x ] > 0)
            {
                horizon[y - 1] = x;
                horizon[y] = x;
                y++;
            }
            else if (edge_image->data[(y+1) * edge_image->cols + x - 1] > 0)
            {
                horizon[y - 1] = x;
                horizon[y] = x;
                y++;x--;
            }
            else if (edge_image->data[(y+1) * edge_image->cols + x - 2] > 0)
            {
                horizon[y - 1] = x;
                horizon[y] = x;
                y++;x=x-2;;
            }
            else if (edge_image->data[(y+1) * edge_image->cols + x + 1] > 0)
            {
                horizon[y - 1] = x;
                horizon[y] = x;
                y++;x++;
            }
            else if (edge_image->data[(y+1) * edge_image->cols + x + 2] > 0)
            {
                horizon[y - 1] = x;
                horizon[y] = x;
                y++;x=x+2;
            }
            else
            {
                y = y - 2; // if the edge does not continue, revert to last know edge position
                break;
            }
        }
        horizon[y] = x;
    }
    return y;
}

void ransacHorizon(int *horizon, horizon_line_t *best_horizon_line)
{
    int first = best_horizon_line->limits[0];
    int last = best_horizon_line->limits[1];
    

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
    int i,j;
    for (i = 0; i < ransac_iter; i++)
    {
        // pick two different, non-zero points on the horizon, within [first,last]
        int s1, s2;
        timeout_counter=0;

        do{
            s1 = rng.uniform(first, last+1);
            timeout_counter++;
        } while (horizon[s1]==0 && timeout_counter<RANSAC_TIMEOUT_LIM);

        do{
            s2 = rng.uniform(first, last+1);
            timeout_counter++;
        } while ( (s1 == s2 || horizon[s2]==0 ) && timeout_counter<RANSAC_TIMEOUT_LIM);

        if (timeout_counter >= RANSAC_TIMEOUT_LIM){
            continue;
        }
        // calculate horizon based on s1 and s2
        m[i] = (float) (horizon[s2] - horizon[s1]) / (float) (s2 - s1);
        b[i] = horizon[s1] - m[i] * s1;

        // calculate error and quality
        float dx = 0;
        for (j = 0; j <IMAGE_WIDTH; j++)
        {
            dx = abs(horizon[j] - m[i] * j - b[i]);
            if (dx < ransac_threshold)
            {
                error[i] += dx;
                quality[i]++;
            }
            else
            {
                error[i] += ransac_threshold;
            }
        }
 
        // update best horizon
        if (error[i] < best_error)
        {
            best_error = error[i];
            best_m = m[i];
            best_b = b[i];
            best_quality = quality[i];
        }
    }

    // calculate new limits
    float local_error;
    int limit[2] = {first, last};
    
    for (i=first; i<last; i++){
        local_error = horizon[i] - abs(best_m*i + best_b);
        if (best_m > 0 && local_error<ransac_threshold){
            limit[1] = i; //-2; -> what was that for?
        }
        else if (best_m < 0 && local_error<ransac_threshold){
            limit[0] = i; //+2;
            break;
        }
        else{
            continue;
        }
    }

    best_horizon_line->m = best_m;
    best_horizon_line->b = best_b;
    best_horizon_line->quality = best_quality;
    best_horizon_line->limits[0] = limit[0];
    best_horizon_line->limits[1] = limit[1];
    
}

void findObstacles_2H(int *obstacles, int *horizon, horizon_line_t *left_horizon, horizon_line_t *right_horizon){
    int horizon_intersect = (int) floor((left_horizon->b-right_horizon->b)/(right_horizon->m-left_horizon->m));
    findObstacles(obstacles, horizon, left_horizon, 0, horizon_intersect);
    findObstacles(obstacles, horizon, right_horizon, horizon_intersect+1, IMAGE_WIDTH-1);
}

void findObstacles_1H(int *obstacles, int *horizon, horizon_line_t *best_horizon){
    findObstacles(obstacles, horizon, best_horizon, 0, IMAGE_WIDTH-1);
}

void findObstacles(int *obstacles, int *horizon, horizon_line_t *horizon_line, int limit_left, int limit_right){
    int i;
    int diff;
    for (i=limit_left; i<= limit_right; i++){
        diff = (int) round(horizon[i] - horizon_line->m*i - horizon_line->b );
        if (abs(diff) > obstacle_threshold){
            obstacles[i] = horizon[i];
        }
        else{
            obstacles[i] = -1;
            continue;}
    }
}

void drawHorizon_2H(struct image_t *img, int *obstacles, horizon_line_t *left_horizon, horizon_line_t *right_horizon){
    int horizon_intersect = (int) floor((left_horizon->b-right_horizon->b)/(right_horizon->m-left_horizon->m));
    drawHorizon(img, obstacles, left_horizon, 0, horizon_intersect);
    drawHorizon(img, obstacles, right_horizon, horizon_intersect+1, IMAGE_WIDTH-1);
}

void drawHorizon_1H(struct image_t *img, int *obstacles, horizon_line_t *best_horizon){
    drawHorizon(img, obstacles, best_horizon, 0, IMAGE_WIDTH-1);
}

void drawHorizon(struct image_t *img, int *obstacles, horizon_line_t *horizon, int limit_left, int limit_right){
    uint8_t *buffer = (uint8_t*) img->buf;

    // Go through all the pixels
    for (uint16_t y = limit_left; y < limit_right; y++) {
        int x = (int) round(horizon->m*y + horizon->b );
        if (x>=img->w || x<0){continue;}

        //get corresponding pixels
        uint8_t *yp, *up, *vp;
        if (x % 2 == 0) {
            // Even x
            up = &buffer[y * 2 * img->w + 2 * x];      // U
            yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
            vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
            //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
        } 
        else {
            // Uneven x
            up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
            //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
            vp = &buffer[y * 2 * img->w + 2 * x];      // V
            yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
        }

        if ( obstacles[y] >= 0) {
            *yp = 128;
            *up = 100;
            *vp = 255;
        }
        else{
            *yp = 128;
            *up = 0;
            *vp = 100;
        }
    }
}

void drawHorizonArray(struct image_t *img, int *horizon){
    uint8_t *buffer = (uint8_t*) img->buf;

    // Go through all the pixels
    for (uint16_t y = 0; y < IMAGE_WIDTH; y++) {
        int x = horizon[y];
        if (x>=img->w || x<0){continue;}

        //get corresponding pixels
        uint8_t *yp;//, *up, *vp;
        if (x % 2 == 0) {
            // Even x
            //up = &buffer[y * 2 * img->w + 2 * x];      // U
            yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
            //vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
            //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
        } 
        else {
            // Uneven x
            //up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
            //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
            //vp = &buffer[y * 2 * img->w + 2 * x];      // V
            yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
        }

        *yp = 255;
    }
}


struct image_t * horizonDetection(struct image_t *img)
{
    int x = 0;
    int y = 0;
    int i;
    Mat track;
    int y_max = 0;
    int y_min = 0;
    int horizon[IMAGE_WIDTH] = {0};
    
    Mat edge_image = image_edges(img);
    cv::imwrite("cnn.png", edge_image);
    while (y < img->h)
    {
        Dot p;
        p.x = x;
        p.y = y;
        
        track = findHorizonCandidate(img, &edge_image, &p);
            
        if (x == (img->w - 1))
        {
            x = p.x;
            y = p.y;
            x = 0;
            y++;
            continue;
        }
        else
        {
            y = p.y;
            x = p.x;
            horizon[y] = x;
            //can limit y_lim to y_max to avoid overwriting past edges, however, it would be helpful to know which one is better
            //other idea: do snake horizon > ransacHorizon > second snake horizon only keeping lines close to the ransac Horizon
            
            y_min = followHorizonLeft(&edge_image, &p, 0, (int*) horizon);
            y_max = followHorizonRight(&edge_image, &p, (int*) horizon);
            // could go right first and use distance to decide if we want to overwrite when moving left
            y = y_max + 2;
            p.y = y_max + 2;
            x = 0;
            p.x = 0;
            // if the segment is too short, scrap it
            if (y_max - y_min < 5)
            {
                for (i = y_min; i <= y_max; i++)
                {
                    horizon[i] = 0;
                }
            }
        }
    }
 
    // calculate principal horizon
    horizon_line_t best_horizon_line;
    ransacHorizon((int*)horizon, &best_horizon_line);
    int obstacle[IMAGE_WIDTH] = {0};
    // check for secondary horizon
    horizon_line_t sec_horizon_line;
    if (best_horizon_line.m > 0){
        sec_horizon_line.limits[0] = best_horizon_line.limits[1];
    }
    else if (best_horizon_line.m < 0) {
        sec_horizon_line.limits[1] = best_horizon_line.limits[0];
    }
    else
    {
        sec_horizon_line.limits[0]=best_horizon_line.limits[0];
        sec_horizon_line.limits[1]=best_horizon_line.limits[1];
    }
    
    ransacHorizon((int*)horizon, &sec_horizon_line);

    // find obstacles using the horizon lines
    if (sec_horizon_line.quality > sec_horizon_threshold && (best_horizon_line.m !=0 || sec_horizon_line.m != 0)){
        // Continue with two horizon lines
        if (best_horizon_line.m > sec_horizon_line.m){
            findObstacles_2H((int*) obstacle,(int*) horizon, &best_horizon_line, &sec_horizon_line);
            if (draw){
                drawHorizon_2H(img, (int*) obstacle, &best_horizon_line, &sec_horizon_line);
            }
        }
        else {
            findObstacles_2H((int*) obstacle,(int*) horizon, &sec_horizon_line, &best_horizon_line);
            if (draw){
                drawHorizon_2H(img, (int*) obstacle, &sec_horizon_line, &best_horizon_line);
            }
        }
    }
    else {
        // Only use main horizon
        findObstacles_1H((int*) obstacle,(int*) horizon, &best_horizon_line);
        if (draw){
            drawHorizon_1H(img, (int*) obstacle, &best_horizon_line);
        }
    }

    if (draw){
        drawHorizonArray(img, (int*) horizon);
    }
    return NULL;
}

void HorizonDetectionInit() {
    cont_thres.lower_y = 16;  cont_thres.lower_u = 135; cont_thres.lower_v = 80;
    cont_thres.upper_y = 100; cont_thres.upper_u = 175; cont_thres.upper_v = 165;

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
}

void HorizonDetectionLoop() {

}
