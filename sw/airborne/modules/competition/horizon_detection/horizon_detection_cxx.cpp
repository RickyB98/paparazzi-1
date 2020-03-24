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
#define RANSAC_ITERATIONS 10
#define RANSAC_THRESHOLD 2
#define RANSAC_TIMEOUT_LIM 5
#define SECONDARY_HORIZON_THRESHOLD 40
#define OBSTACLE_THRESHOLD 5

bool draw = false;

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
RNG rng(12345);

uint8_t cf_ymin = 100;
uint8_t cf_ymax = 150;
uint8_t cf_umin = 0;
uint8_t cf_umax = 100;
uint8_t cf_vmin = 110;
uint8_t cf_vmax = 170;


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
        (buf[1] >= cf_ymin) && (buf[1] <= cf_ymax) && (buf[0] >= cf_umin) && (buf[0] <= cf_umax) && (buf[2] >= cf_vmin) && (buf[2] <= cf_vmax))
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
    while (!onFloor && x < img->w - 1)
    {
        x++;
        onFloor = isFloor(img, y, x);
        track.data[y * img->w + x] = 255;
    }
    // move up to closest edge
    onEdge = (edge_image->data[y * edge_image->cols + x] > 0);
    while (onEdge == 0 && x < img->w - 1)
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
        cout <<"HORIZON"<< horizon[y]<< "    "<< y  <<endl;
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
        cout <<"HORIZON????????????????"<< horizon[y]<<"     " << y <<endl;
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
    float error[RANSAC_ITERATIONS] = {0.0f};
    float m[RANSAC_ITERATIONS] = {0.0f};
    float b[RANSAC_ITERATIONS] = {0.0f};

    int best_quality = 0;
    int best_error = RANSAC_THRESHOLD * (N + 1);
    float best_m = 0;
    float best_b = 0;

    uint8_t timeout_counter_s1 = 0; uint8_t timeout_counter_s2 = 0;
    bool timeout;


    int i,j;
    for (i = 0; i < RANSAC_ITERATIONS; i++)
    {
        // pick two different, non-zero points on the horizon, within [first,last]
        timeout = false;
        int s1, s2;
        //cout << "seraquesim?" << endl;

        timeout_counter_s1=0;
        do{
            s1=rand()%(last-first + 1) + first;
            //s1 = int(round(((last-first)*rand()/(RAND_MAX+1.0))));
            timeout_counter_s1++;
            cout << last << "ttt" << endl;
            cout << first << "fff" << endl;
            cout << s1 << "bbb" << endl;
            cout << horizon[s1] << "hhh" << endl;
        } while (horizon[s1]==0 && timeout_counter_s1<RANSAC_TIMEOUT_LIM);
        
        timeout_counter_s2=0;
        do{
            s2=rand()%(last-first + 1) + first;
            //s2 = int(round(((last-first)*rand()/(RAND_MAX+1.0))));
            timeout_counter_s2++;
            //cout << "seraquesim nononono" << endl;
            cout << s2 << endl;
            cout << horizon[s2] << "sssssssssss" << endl;
        } while ( (s1 == s2 || horizon[s2]==0 ) && timeout_counter_s2<RANSAC_TIMEOUT_LIM);

        if ((timeout_counter_s1 >= RANSAC_TIMEOUT_LIM)||(timeout_counter_s2 >= RANSAC_TIMEOUT_LIM)){continue;}

                cout << "aaaa" << endl;
         // calculate horizon based on s1 and s2
                cout << s1 << " " << s2 << endl;


        // why is this necessary? 
        if(s1!=s2){
        m[i] = (float) (horizon[s2] - horizon[s1]) / (float) (s2 - s1);
        b[i] = horizon[s1] - m[i] * s1;
        cout << m[i]<< "m" <<endl;
        cout << b[i] << "b" << endl;
        }
        else
        {
            timeout_counter_s2=0;
            do{
            s2=rand()%(last-first + 1) + first;
            //s2 = int(round(((last-first)*rand()/(RAND_MAX+1.0))));
            timeout_counter_s2++;
            //cout << "seraquesim sisisisi" << endl;
            //cout << s2 << endl;
            } while ( (s1 == s2 || horizon[s2]==0 ) && timeout_counter_s2<RANSAC_TIMEOUT_LIM);
        
                cout << s2 << endl;
            cout << horizon[s2] << "ppppppppp" << endl;
            m[i] = (float)(horizon[s2] - horizon[s1]) / (float)(s2 - s1);
            b[i] = horizon[s1] - m[i] * s1;
            break;
        }
        
        // calculate error and quality

        for (j = 0; j < N; j++)
        {
            float dx = abs(horizon[j] - m[i] * j - b[i]);

            if (dx < RANSAC_THRESHOLD)
            {
                error[i] += dx;
                quality[i]++;
                cout << quality[i]<<"   QUALITY     QUALITY"<< i << "       "<<endl;
            }
            else
            {
                error[i] += RANSAC_THRESHOLD;
                cout << quality[i]<<"  MISTAKE MISTAKE   "<< i <<endl;
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

        //cout << horizon[i] << "horizon" << endl;
        //cout << best_b << "b" <<endl;
        //cout << best_m << "m" << endl;
        //cout << local_error << "localerror" <<endl;
        if (best_m > 0 && local_error<RANSAC_THRESHOLD){
            limit[1] = i-2;
            //cout << "option 1"<<"" << limit[1]<< endl;
        }
        else if (best_m < 0 && local_error<RANSAC_THRESHOLD){
            limit[0] = i+2;
            //cout << "option 2" <<""<< limit[0]<< endl;
            break;
        }
        else{
            limit[0]=best_horizon_line->limits[0];
            limit[1]=best_horizon_line->limits[1];
            //cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" <<""<<limit[1]<<"" << limit[0]<< endl;

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
        if (diff > OBSTACLE_THRESHOLD){
            obstacles[i] = horizon[i];
        }
        else{continue;}
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
        uint16_t x = (int) round(horizon->m*y + horizon->b );
        if (x>=img->w){continue;}

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

        if ( obstacles[y] != 0) {
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



struct image_t * horizonDetection(struct image_t *img)
{
    
    int x = 0;
    int y = 0;
    int i;
    Mat track;
    int y_max = 0;
    int y_min = 0;
    int horizon[520];
    for ( i = 0; i < 520; i++)
    {
        horizon[i]=0;
    }
    
    Mat edge_image = image_edges(img);
    cv::imwrite("cnn.png", edge_image);
    cout<<img->h<<" "<< img->w <<endl;
    cout<<edge_image.cols<<" "<< edge_image.rows <<endl;
    while (y < img->h)
    {
        cout << "HEEELLLOOOOOOOO"<<endl;
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
            y = y_max + 1;
            p.y = y_max + 1;
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
    cout << "best horizon quality:" << best_horizon_line.quality<<endl;
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
    
    cout << "secondary horizon quality:"<< sec_horizon_line.quality << endl;

    ransacHorizon((int*)horizon, &sec_horizon_line);
    Mat picture(img->h, img->w, CV_8UC1);
        for (i=0;i<img->h;i++){
            for (int j = 0; j < img->w; ++j) {
                if (horizon[i]==j)
                    picture.data[i*img->w +j] = 255;
                else if (abs(j - best_horizon_line.m*i - best_horizon_line.b) < 1)
                    picture.data[i*img->w +j] = 128;
                else if (abs(j - sec_horizon_line.m*i-sec_horizon_line.b) < 1)
                    picture.data[i*img->w +j] = 100;
                else
                    picture.data[i*img->w +j] = 0;
            }
         }
         cv::imwrite("vala.png", picture);

    // find obstacles using the horizon lines
    int obstacle[IMAGE_WIDTH] = {0};
    if (sec_horizon_line.quality > SECONDARY_HORIZON_THRESHOLD){
        // Continue with two horizon lines
        if (best_horizon_line.m > sec_horizon_line.m){
            findObstacles_2H((int*) obstacle,(int*) horizon, &best_horizon_line, &sec_horizon_line);
            // if (draw){dra
            //     drawHorizon_2H(img, (int*) obstacle, &best_horizon_line, &sec_horizon_line);
            // }
        }
        else {
            findObstacles_2H((int*) obstacle,(int*) horizon, &sec_horizon_line, &best_horizon_line);
            // if (draw){
            //     drawHorizon_2H(img, (int*) obstacle, &sec_horizon_line, &best_horizon_line);
            // }
        }
    }
    else {
        // Only use main horizon
        findObstacles_1H((int*) obstacle,(int*) horizon, &best_horizon_line);
        // if (draw){
        //     drawHorizon_1H(img, (int*) obstacle, &best_horizon_line);
        // }
    }
    
    return NULL;
}

void HorizonDetectionInit() {
    cont_thres.lower_y = 16;  cont_thres.lower_u = 135; cont_thres.lower_v = 80;
    cont_thres.upper_y = 100; cont_thres.upper_u = 175; cont_thres.upper_v = 165;

    //cf_ymin = COMPETITION_CF_YMIN;
    //cf_ymax = COMPETITION_CF_YMAX;
    //cf_umin = COMPETITION_CF_UMIN;
    //cf_umax = COMPETITION_CF_UMAX;
    //cf_vmin = COMPETITION_CF_VMIN;
    //cf_vmax = COMPETITION_CF_VMAX;
}

void HorizonDetectionLoop() {

}
