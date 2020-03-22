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
#define RANSAC_ITERATIONS 20
#define RANSAC_THRESHOLD 5
#define RANSAC_TIMEOUT_LIM 20
#define SECONDARY_HORIZON_THRESHOLD 40
#define OBSTACLE_THRESHOLD 5


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
void findObstacles_2H(int *obstacles, int *horizon, horizon_line_t *left_horizon, horizon_line_t *right_horizon);
void findObstacles_1H(int *obstacles, int *horizon, horizon_line_t *best_horizon);
void findObstacles(int *obstacles, int *horizon, horizon_line_t *horizon, int limit_left, int limit_right);

RNG rng(12345);

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
 * @param[in] y_m The Y minimum value
 * @param[in] y_M The Y maximum value
 * @param[in] u_m The U minimum value
 * @param[in] u_M The U maximum value
 * @param[in] v_m The V minimum value
 * @param[in] v_M The V maximum value
 * @return The success of the filter.
 */

bool isFloor(struct image_t *img, int x, int y, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M)
{
    // odd pixels are uy
    // even pixels are vy
    // adapt x, so that we always have u-channel in index 0:
    if (x % 2 == 1)
    {
        x--;
    }

    // Is the pixel inside the image?
    if (x < 0 || x >= img->w || y < 0 || y >= img->h)
    {
        return false;
    }

    // Take the right place in the buffer:
    uint8_t *buf = (uint8_t *)(img->buf);
    buf += 2 * (y * (img->w) + x); // each pixel has two bytes

    if (
        (buf[1] >= y_m) && (buf[1] <= y_M) && (buf[0] >= u_m) && (buf[0] <= u_M) && (buf[2] >= v_m) && (buf[2] <= v_M))
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
    Mat track(img->w, img->h, CV_8UC1);
    x = p->x;
    y = p->y;
    
    onFloor = isFloor(img, y, x, 0, 0, 0, 0, 0, 0);
cout << "aaaa" << endl;
    track.data[y * img->w + x] = 255;
    cout << "nnnnnn" << endl;
    while (!onFloor && x < img->w - 1)
    {
        x++;
        onFloor = isFloor(img, y, x, 0, 0, 0, 0, 0, 0);
        track.data[y * img->w + x] = 255;
    }
    // move up to closest edge
    onEdge = (edge_image->data[y * edge_image->rows + x] > 0);
    while (onEdge == 0 && x < img->w - 1)
    {
        x++;
        onEdge = (edge_image->data[y * edge_image->rows + x] > 0);
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

    while (y > y_lim && x > 0 && x < edge_image->rows - 1)
    {
        y--;
        if (edge_image->data[y * edge_image->cols + x] > 0)
        { // edge continues right
            continue;
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

    while (y < edge_image->cols - 1 && x > 0 && x < edge_image->rows - 1)
    {
        y++;
        if (edge_image->data[y * edge_image->cols + x] > 0)
        { // edge continues right
            continue;
        }
        else if (edge_image->data[y * edge_image->cols + x - 1] > 0)
        { // edge continues bottom right
            x--;
        }
        else if (edge_image->data[y * edge_image->cols + x + 1] > 0)
        { // edge continues top right
            x++;
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
            else if (edge_image->data[y * edge_image->cols + x + 1] > 0)
            {
                horizon[y - 1] = x;
                x++;
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
    int N = sizeof(horizon);
    // find non-zero range
    int first = best_horizon_line->limits[0];
    int last = best_horizon_line->limits[1];

    // initialize ransac horizon variables
    int quality[RANSAC_ITERATIONS] = {0};
    int error[RANSAC_ITERATIONS] = {0.0f};
    float m[RANSAC_ITERATIONS] = {0.0f};
    float b[RANSAC_ITERATIONS] = {0.0f};

    int best_quality = 0;
    int best_error = RANSAC_THRESHOLD * (N + 1);
    float best_m = 0;
    float best_b = 0;

    uint8_t timeout_counter = 0;
    bool timeout = false;

    int i,j;
    for (i = 0; i < RANSAC_ITERATIONS; i++)
    {
        // pick two different, non-zero points on the horizon, within [first,last]
        timeout = false;
        int s1, s2;
        do{
            s1 = int(round(first + (last-first)*rand()/RAND_MAX));
            timeout_counter++;
        } while (horizon[s1]==0 && timeout_counter<RANSAC_TIMEOUT_LIM);
        
        do{
            s2 = int(round(first + (last-first)*rand()/RAND_MAX));
            timeout_counter++
        } while ( (s1 == s2 || horizon[s2]==0 ) && timeout_counter<RANSAC_TIMEOUT_LIM);

        // calculate horizon based on s1 and s2
        m[i] = (horizon[s2] - horizon[s1]) / (s2 - s1);
        b[i] = horizon[s1] - m[i] * s1;

        // calculate error and quality
        for (j = 0; j < N; j++)
        {
            int dx = abs(horizon[j] - m[i] * j - b[i]);

            if (dx < RANSAC_THRESHOLD)
            {
                error[i] += dx;
                quality[i]++;
            }
            else
            {
                error[i] += RANSAC_THRESHOLD;
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
        local_error = horizon[i] - best_m*i - best_b;
        if (best_m > 0 && local_error<RANSAC_THRESHOLD){
            limit[1] = i;
        }
        if (best_m < 0 && local_error<RANSAC_THRESHOLD){
            limit[0] = i;
            break;
        }
    }

    best_horizon_line->m = best_m;
    best_horizon_line->b = best_b;
    best_horizon_line->quality = best_quality;
    best_horizon_line->limits = limit;
}

void findObstacles_2H(int *obstacles, int *horizon, horizon_line_t *left_horizon, horizon_line_t *right_horizon){
    int horizon_intersect = (int) floor((left_horizon->b-right_horizon->b)/(right_horizon->m-left_horizon->m));
    findObstacles(obstacles, horizon, left_horizon, 0, horizon_intersect);
    findObstacles(obstacles, horizon, right_horizon, horizon_intersect+1, IMAGE_WIDTH-1)
}

void findObstacles_1H(int *obstacles, int *horizon, horizon_line_t *best_horizon){
    findObstacles(obstacles, horizon, best_horizon, 0, IMAGE_WIDTH-1);
}

void findObstacles(int *obstacles, int *horizon, horizon_line_t *horizon_line, int limit_left, int limit_right){
    int i;
    int diff;
    for (i=limit_left, i<= limit_right, i++){
        diff = (int) round(horizon[i] - horizon_line->m*i - horizon_line->b );
        if (diff > OBSTACLE_THRESHOLD){
            obstacles[i] = horizon[i];
        }
    }
}

v
struct image_t * horizonDetection(struct image_t *img)
{
    
    int x = 0;
    int y = 0;
    int i;int j;
    Mat track;
    int y_max = 0;
    int y_min = 0;
    int horizon[520];
    int best_horizon[520];
    Mat edge_image = image_edges(img);
    cv::imwrite("cnn.png", edge_image);
    while (y < img->w)
    {
        Dot p;
        p.x = x;
        p.y = y;
        
        track = findHorizonCandidate(img, &edge_image, &p);
            
        if (x == (img->h - 1))
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
            y = y_max + 1;
            p.y = y_max + 1;
            x = 0;
            p.x = 0;
            // if the segment is too short, scrap it
            if (y_max - y_min < 5)
            {
                for (i = 0; i < (y_max + 1 - y_min); i++)
                {
                    horizon[i] = 0;
                }
            }
        }
    }
    
    // calculate principal horizon
    horizon_line_t best_horizon_line;
    ransacHorizon(&horizon, &best_horizon_line);

    // check for secondary horizon
    horizon_line_t sec_horizon_line;
    if (best_horizon_line.m > 0){
        sec_horizon_line.limits[0] = best_horizon_line.limits[1];
    }
    else {
        sec_horizon_line.limits[1] = best_horizon_line.limits[0];
    }
    ransacHorizon(&horizon, &sec_horizon_line);

    // find obstacles using the horizon lines
    int obstacle[IMAGE_WIDTH] = {0};
    if (sec_horizon_line.quality > SECONDARY_HORIZON_THRESHOLD){
        // Continue with two horizon lines
        if (best_horizon_line.m > sec_horizon_line.m){
            findObstacles_2H(&obstacle, &best_horizon_line, &sec_horizon_line);
        }
        else {
            findObstacles_2H(&obstacle, &sec_horizon_line, &best_horizon_line);
        }
    }
    else {
        // Only use main horizon
        findObstacles_1H(&obstacle, &best_horizon_line);
    }
    
    // Mat img_w_track = cvLoadImage();
    // for (i = 0; i < img->h; i++)
    // {
    //     for (j = 0; j < img->w; j++)
    //     {
    //         if (track.data[i * img->w + j])
    //         {
    //             img_w_track.data[i * img->w + j] = ; //green
    //         }
    //         int value = int(obstacle[i]);
    //         if (value >= 0)
    //         {
    //             img_w_horizon.data[i * img->w + value] = ; //green
    //         }
    //         if (best_horizon[i] >= 0 and best_horizon[i] < img_w_horizon.shape[1]) //needs correction depending on the type of variable
    //         {                                                                      //red
    //             img_w_horizon[i][int(best_horizon[i])][0] = 0;
    //             img_w_horizon[i][int(best_horizon[i])][1] = 255;
    //             img_w_horizon[i][int(best_horizon[i])][2] = 0;
    //         }
    //         img_w_track[i][int(horizon[i])][0] = 0;
    //         img_w_track[i][int(horizon[i])][1] = 0;
    //         img_w_track[i][int(horizon[i])][2] = 255;
    //         //green
    //     }
    // }
    return NULL;
}

void HorizonDetectionInit() {
    cont_thres.lower_y = 16;  cont_thres.lower_u = 135; cont_thres.lower_v = 80;
    cont_thres.upper_y = 100; cont_thres.upper_u = 175; cont_thres.upper_v = 165;
}

void HorizonDetectionLoop() {

}