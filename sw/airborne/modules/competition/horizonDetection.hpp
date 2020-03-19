#include <stdio.h>
#include <opencv_contour.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
using namespace std;

extern void image_edges(char *img, int width, int height);
extern void horizonDetection(char *img,int width, int height, double *edge_image);
int followHorizonRight(Mat edge_image, struct *p, int aux, int *horizon);
int followHorizonLeft(Mat edge_image, struct *p, int aux, int *horizon);
int isFloor(int pixel);
int findHorizonCandidate(Mat *img,Mat *edge_image,Dot *p);