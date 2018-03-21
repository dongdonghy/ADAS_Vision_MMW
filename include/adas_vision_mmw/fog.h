#include "iostream"
//#include <algorithm>
#include "time.h"
#include "string.h"
//#include "io.h"
//#include <windows.h>

/****** OpenCV 2.3.1 *******/
#include "opencv2/opencv.hpp"

#define MAX_INT 20000000

using namespace std;
using namespace cv;

//Type of Min and Max value
typedef struct _MinMax
{
	double min;
	double max;
}MinMax;

Mat ReadImage();
void rerange();
void fill_x_y();
int find_table(int y);
void locate(int l1, int l2, double l3);
void getL(Mat img);

Vec<float, 3> Airlight(Mat img, Mat dark);
Mat TransmissionMat(Mat dark);
Mat DarkChannelPrior(Mat img);

void RefineTrans(Mat trans);


void printMat(char * name, Mat m);

//Mat fog(Mat input_image);
