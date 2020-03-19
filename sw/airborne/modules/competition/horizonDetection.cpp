#include <iostream>
#include "horizonDetection.hpp"

#include <opencv_contour.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
using namespace std;

#include <string>
#include <modules/computer_vision/cv.h>

struct contour_estimation cont_est;
struct contour_threshold cont_thres;

RNG rng(12345);

void image_edges(char *img, int width, int height)
{
    // Create a new image, using the original bebop image.
    Mat M(width, height, CV_8UC2, img); // original
    Mat image, edge_image, thresh_image;

    // convert UYVY in paparazzi to YUV in opencv
    cvtColor(M, M, CV_YUV2RGB_Y422);
    cvtColor(M, M, CV_RGB2YUV);

    // Threshold all values within the indicted YUV values.
    inRange(M, Scalar(cont_thres.lower_y, cont_thres.lower_u, cont_thres.lower_v), Scalar(cont_thres.upper_y, cont_thres.upper_u, cont_thres.upper_v), thresh_image);

    /// Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    edge_image = thresh_image;
    int edgeThresh = 35;
    Canny(edge_image, edge_image, edgeThresh, edgeThresh * 3);
    //findContours(edge_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    edge_image
}

typedef struct Dot
{
    float x;
    float y;
} Dot;

int isFloor(int pixel)
{
    if ((pixel[0] > 70 and pixel[0] < 100) &&
        (pixel[1] > 70 && pixel[1] < 100) &&
        (pixel[2] > 70 && pixel[2] < 100))
    {
        return true;
    }
    else
    {
        return false;
    }
}

int findHorizonCandidate(Mat *img, Mat *edge_image, Dot *p)
{
    x = p.x;
    y = p.y;
    onFloor = isFloor(img[y][x][:])
        track[y][x] = true;
    while
        not onFloor and x < img.shape[1] - 1
        {
            x++;
            onFloor = isFloor(img[y][x][:]);
            track[y][x] = true;
        }
    // move up to closest edge
    onEdge = (edges[y][x] > 0);
    while
        not onEdge and x < img.shape[1] - 1
        {
            x++;
            onEdge = (edges[y][x] > 0);
            track[y][x] = true;
        }
    p.x = x;
    p.y = y;
    return p;
}

int followHorizonLeft(Mat edge_image, Dot *p, int aux, int *horizon)
{

    int x;
    int y;
    int y_lim;
    x = p.x;
    y = p.y;

    while (y > y_lim && x > 0 && x < edge_image->size().width - 1)
    {
        y--;
        if (edge_image[y][x] > 0)
        { // edge continues right
            continue;
        }
        else if (edge_image[y][x - 1] > 0)
        { // edge continues bottom right
            x--;
        }
        else if (edge_image[y][x + 1] > 0)
        { //edge continues top right
            x--;
        }
        else
        { //increase step
            y--;
            if (edge_image[y][x] > 0)
            {
                horizon[y + 1] = x;
            }
            else if (edge_image[y][x - 1] > 0)
            {
                horizon[y + 1] = x;
                x--;
            }
            else if (edge_image[y][x + 1] > 0)
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

int followHorizonRight(Mat edge_image, Dot *p, int aux, int *horizon)
{

    int x;
    int y;
    int y_lim;
    x = p.x;
    y = p.y;

    while (y < edge_image->size().height - 1 && x > 0 && x < edge_image->size().width - 1)
    {
        y++;
        if (edge__image[y][x] > 0)
        { // edge continues right
            continue;
        }
        else if (edge__image[y][x - 1] > 0)
        { // edge continues bottom right
            x--;
        }
        else if (edge__image[y][x + 1] > 0)
        { // edge continues top right
            x++;
        }
        else
        { // increase step
            y++;
            if (edge__image[y][x] > 0)
            {
                horizon[y - 1] = x;
            }
            else if (edge__image[y][x - 1] > 0)
            {
                horizon[y - 1] = x;
                x--;
            }
            else if (edge__image[y][x + 1] > 0)
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

void horizonDetection(char *img, int width, int height, double *edge_image)
{

    int x = 0;
    int y = 0;
    int i;
    int y_max = 0;
    int y_min = 0;
    int horizon[520];
    while (y < width)
    {
        Dot p;
        p.x = x;
        p.y = y;

        Dot p1 = findHorizonCandidate(img, edge_image, p);

        if (x == (height - 1))
        {
            x = p1.x;
            y = p1.y;
            x = 0;
            y++;
            continue;
        }
        else
        {
            y = p.y;
            x = p.x;
            horizon[y] = p.x;
            //can limit y_lim to y_max to avoid overwriting past edges, however, it would be helpful to know which one is better
            //other idea: do snake horizon > ransacHorizon > second snake horizon only keeping lines close to the ransac Horizon

            y_min = followHorizonLeft(edge_image, p1, 0, horizon);
            y_max = followHorizonRight(edge_image, p1);
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
}