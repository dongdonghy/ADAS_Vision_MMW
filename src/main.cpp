#include "ros/ros.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <list>
#include <iostream> 
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <string> 
#include "opencv2/opencv.hpp"
#include <vector>
#define _CRT_SECURE_NO_WARNINGS
#include "adas_vision_mmw/fog.h"
#include "adas_vision_mmw/guidedfilter.h"
#include <cv_bridge/cv_bridge.h>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <sys/time.h>
#include <string>
#include <curses.h>
#include <GL/glut.h>
using namespace std;
using namespace cv;

#define CAMwidth 640
#define CAMheight 480
#define MoveDistortBorder 10
#define LaneOverLookimageWidth 200
#define LaneCarWidth 20
#define WINDOWIMGE1 1
#define WINDOWIMGE2 1
#define WINDOWIMGE3 1
#define WINDOWIMGE4 1
#define WINDOWIMGE5 1
#define MaxDrawPoint 100
#define DRAW_LANE_BY_GPS_AND_ESR 1
#define DRAW_LANE_BY_GPS 1

#define RectShieldDistance 150
#define AimColorByDistance1 10
#define AimColorByDistance2 50 
#define AimColorByDistance3 175

#define RadiusMagnify 5


double LaneCarMinDistance =0.0;

vector<double> bounding_box;
//bounding_box.clear();
double result[60];
int number = 0;
int number2 = 0;
double result2[600];
long double SystemInfo[14];
vector<long double> LatitudeInfo(50);
vector<long double> LongitudeInfo(50);

vector<long double> ESRAimDistance1(50);
vector<long double> ESRAimAngle1(50);
vector<vector<long double> > ESRAimDistance(64, vector<long double>(20));
vector<vector<long double> > ESRAimAngle(64, vector<long double>(20));

int _PriorSize = 15;
float _topbright = 0.0005;
float _w = 0.95;
float t0 = 0.2;
int SizeH = 0;
int SizeW = 0;
int SizeH_W = 0;
Vec<float, 3> a;
float a2=0.93;
Mat trans_refine;
Mat dark_out1;

struct AimInfoStruct
{
    unsigned char aim_ID;
    double aim_distance;
    double aim_angel;
    bool aim_IsAlive;
    unsigned char remap_J;
};
AimInfoStruct AimInfoArray_front[64];
AimInfoStruct AimInfoArray_rear[64];

// output stream
template<typename T> string toString(const T& t){
    ostringstream oss;
    oss<<t;
    return oss.str().c_str();   
}

bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)  
{  
    //Number of key points  
    int N = key_point.size();  
  
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);  
    for (int i = 0; i < n + 1; i++)  
    {  
        for (int j = 0; j < n + 1; j++)  
        {  
            for (int k = 0; k < N; k++)  
            {  
                X.at<double>(i, j) = X.at<double>(i, j) +  
                    std::pow(key_point[k].x, i + j);  
            }  
        }  
    }  
   
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);  
    for (int i = 0; i < n + 1; i++)  
    {  
        for (int k = 0; k < N; k++)  
        {  
            Y.at<double>(i, 0) = Y.at<double>(i, 0) + std::pow(key_point[k].x, i) * key_point[k].y;  
        }  
    }  
  
    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);  
    cv::solve(X, Y, A, cv::DECOMP_LU);  
    return true;  
}  

void BoundingBoxCallback(const std_msgs::Float64MultiArray msg)
{
    number = msg.data.size()/6;
    cout << number << endl;
    for(int i = 0; i<number*6; i++)
        result[i] = msg.data[i];
}

void EsrCallback(const std_msgs::Float64MultiArray msg)
{
    number2 = msg.data[0];
    cout << number2 << endl;
    for(int i = 0; i<11; i++)
    {
        SystemInfo[i]=msg.data[i];
    }
    for(int j = 0; j<number2*8; j++)
        result2[j] = msg.data[j+11];
}


double ConvertDegreesToRadians(double degrees)
{
    return degrees * M_PI / 180;
}

double ConvertRadiansToDegrees(double radian)
{
    return radian * 180.0 / M_PI;
}


double HaverSin(double theta)
{
    double v = sin(theta / 2);
    return v * v;
}

// the radius of the earth
double EARTH_RADIUS = 6371000.0;

double Distance(long double lat1,long double lon1, long double lat2,long double lon2)
{
    //haversine equation
    lat1 = ConvertDegreesToRadians(lat1);
    lon1 = ConvertDegreesToRadians(lon1);
    lat2 = ConvertDegreesToRadians(lat2);
    lon2 = ConvertDegreesToRadians(lon2);

    double vLon = abs(lon1 - lon2);
    double vLat = abs(lat1 - lat2);

    //h is the great circle distance in radians
    double h = HaverSin(vLat) + cos(lat1) * cos(lat2) * HaverSin(vLon);
    if(h>1)
    {
        printf("angular transformation error %f\n  ",h);
    }
    double distance = 2 * EARTH_RADIUS * asin(sqrt(h));
    return distance;
}

long double GetGPSAngle(long double lat1,long double lon1, long double lat2,long double lon2)
{

    lat1 = ConvertDegreesToRadians(lat1);
    lon1 = ConvertDegreesToRadians(lon1);
    lat2 = ConvertDegreesToRadians(lat2);
    lon2 = ConvertDegreesToRadians(lon2);
	
    long double sin_c=0,cos_c=0,agnle=0;
    cos_c=cos(M_PI/2-lat1) * cos(M_PI/2 - lat2) +sin(M_PI/2 - lat1)*sin(M_PI/2 - lat2) * cos(lon1- lon2);
    if(abs(cos_c)>=1)
    {
        return 0;
        printf("out of memory\n");
    }

    sin_c=sqrt(1-cos_c * cos_c);
    agnle=asin(sin(M_PI/2-lat1)* sin(lon1- lon2)/sin_c);

    agnle=agnle* 180/M_PI;
    if(lat1 >= lat2 && lon1 >= lon2 )//first quadrant
    {
        agnle=agnle;
    }
    else if(lat1 >= lat2 && lon1 < lon2 ) //second quadrant
    {
        agnle=agnle+360;
    }
    else if(lat1 <= lat2 && lon1 <= lon2 ) //third quadrant
    {
        agnle=180-agnle;
    }
    else if(lat1 < lat2 && lon1 > lon2 ) //fouth quadrant
    {
        agnle=180-agnle;
    }
    return agnle;
}

Mat DarkChannelPrior(Mat img)
{
    Mat dark = Mat::zeros(img.rows, img.cols, CV_32FC1);

    for (int i = 0; i<img.rows; i++)
    {
        for (int j = 0; j<img.cols; j++)
        {
            dark.at<float>(i, j) = min(min(img.at<Vec3f>(i, j)[0], img.at<Vec3f>(i, j)[1]),
				  min(img.at<Vec3f>(i, j)[0], img.at<Vec3f>(i, j)[2]));
        }
    }
    //window-sized template operation, corresponding to the minimum filter
    erode(dark, dark_out1, Mat::ones(_PriorSize, _PriorSize, CV_32FC1));
    return dark_out1;
}

Mat DarkChannelPrior_(Mat img)
{
    float A = (a[0] + a[1] + a[2]) / 3.0;

    Mat dark = Mat::zeros(img.rows, img.cols, CV_32FC1);
    Mat dark_out = Mat::zeros(img.rows, img.cols, CV_32FC1);
    for (int i = 0; i<img.rows; i++)
    {
        for (int j = 0; j<img.cols; j++)
        {
            dark.at<float>(i, j) = min(min(img.at<Vec3f>(i, j)[0] / A, img.at<Vec3f>(i, j)[1] / A),
				   min(img.at<Vec3f>(i, j)[0] / A, img.at<Vec3f>(i, j)[2] / A));
        }
    }
    erode(dark, dark_out, Mat::ones(_PriorSize, _PriorSize, CV_32FC1));
    return dark_out;
}


//calculate A
Vec<float, 3> Airlight(Mat img, Mat dark)
{
    int SizeH_W_2=img.rows*img.cols;
    int n_bright = _topbright*SizeH_W_2;
    Mat dark_1 = dark.reshape(1, SizeH_W_2);
    vector<int> max_idx;
    float max_num = 0;
    Vec<float, 3> A(0, 0, 0);
    Mat RGBPixcels = Mat::ones(n_bright, 1, CV_32FC3);

    for (int i = 0; i<n_bright; i++)
    {
        max_num = 0;
        max_idx.push_back(max_num);
        for (float * p = (float *)dark_1.datastart; p != (float *)dark_1.dataend; p++)
        {
            if (*p>max_num)
            {
                max_num = *p;
                max_idx[i] = (p - (float *)dark_1.datastart);
            }
        }
        RGBPixcels.at<Vec3f>(i, 0) = ((Vec3f *)img.data)[max_idx[i]];
        ((float *)dark_1.data)[max_idx[i]] = 0;
    }
    for (int j = 0; j<n_bright; j++)
    {
        A[0] += RGBPixcels.at<Vec3f>(j, 0)[0];
        A[1] += RGBPixcels.at<Vec3f>(j, 0)[1];
        A[2] += RGBPixcels.at<Vec3f>(j, 0)[2];
    }
    A[0] /= n_bright;
    A[1] /= n_bright;
    A[2] /= n_bright;
    if (A[0]>a2)
    {
        A[0]=a2;
    }
    if (A[1]>a2)
    {
        A[1]=a2;
    }
    if (A[2]>a2)
    {
        A[2]=a2;
    }
    return A;
}


//Calculate Transmission Matrix
Mat TransmissionMat(Mat dark)
{
    float A = (a[0] + a[1] + a[2]) / 3.0f;
    for (int i = 0; i < dark.rows; i++)
    {
        for (int j = 0; j < dark.cols; j++)
        {
            float temp = (dark_out1.at<float>(i, j));
            float B = fabs(A - temp);
            if (B - 0.3137254901960784f < 0.0000000000001f)//K=80
            {
                dark.at<float>(i, j) = (1 - _w*dark.at<float>(i, j))*(0.3137254901960784f / (B));
            }
            else
            {
                dark.at<float>(i, j) = 1 - _w*dark.at<float>(i, j);
            }
            if (dark.at<float>(i, j) <= 0.2f)
            {
                dark.at<float>(i, j) = 0.5f;
            }
            if (dark.at<float>(i, j) >= 1)
            {
                dark.at<float>(i, j) = 1.0f;
            }
        }
    }
    return dark;
}

Mat TransmissionMat1(Mat dark)
{
    float A = (a[0] + a[1] + a[2]) / 3.0f;
    for (int i = 0; i < dark.rows; i++)
    {
        for (int j = 0; j < dark.cols; j++)
        {
            dark.at<float>(i, j) = (1 - _w*dark.at<float>(i, j));
        }
    }
    return dark;
}

//Calculate Haze Free Image
Mat hazefree(Mat img, Mat t, Vec<float, 3> a, float exposure = 0)
{
    float AAA = a[0];
    if (a[1] > AAA)
        AAA = a[1];
    if (a[2] > AAA)
        AAA = a[2];

    Mat freeimg = Mat::zeros(SizeH, SizeW, CV_32FC3);
    img.copyTo(freeimg);

    Vec<float, 3> * p = (Vec<float, 3> *)freeimg.datastart;
    float * q = (float *)t.datastart;

    for (; p<(Vec<float, 3> *)freeimg.dataend && q<(float *)t.dataend; p++, q++)
    {
        (*p)[0] = ((*p)[0] - AAA) / std::max(*q, t0) + AAA + exposure;
        (*p)[1] = ((*p)[1] - AAA) / std::max(*q, t0) + AAA + exposure;
        (*p)[2] = ((*p)[2] - AAA) / std::max(*q, t0) + AAA + exposure;
    }
    return freeimg;
}

void printMatInfo(char * name, Mat m)
{
    cout << name << ":" << endl;
    cout << "\t" << "cols=" << m.cols << endl;
    cout << "\t" << "rows=" << m.rows << endl;
    cout << "\t" << "channels=" << m.channels() << endl;
}

int main(int argc, char **argv)
{	
    int pointsCount=0;
    int Lane_point_count =20;
    int Lane_point_draw =20;
    long double GPS_DistancePoint[1000]={0};
    long double GPS_AnglePoint[1000]={0};	
    double maxspeed_test=0;
    long double Rotate_Angle_theta=0.0;
    long double Absolute_X=0.0;
    long double Absolute_Y=0.0;
    long double Relative_X=0.0;
    long double Relative_Y=0.0;
    int j=0,ID_k=0,ID_i=0;
    int ID_flag=0;
    Mat img_merge;
    Mat dark_channel;
    Mat trans;
    Mat imgCamera_R;
    Mat imgCamera;
    Mat free_img;
    char filename[100];
    int period = 10;
    int m = 0;

    //bird's view
    cv::Mat LaneOverLookimage = cv::Mat::zeros(CAMheight-MoveDistortBorder*2 , LaneOverLookimageWidth, CV_8UC3);  
    LaneOverLookimage.setTo(cv::Scalar(0, 0, 0));  
    std::vector<cv::Point> points;  
    cv::Mat LaneA;  
    cv::Point points_fitted_test; 
    std::vector<cv::Point> points_fitted;  
    std::vector<cv::Point> points_fitted_1;  
    std::vector<cv::Point> points_fitted_2;  
    std::vector<cv::Point> points_fitted_3;  
    std::vector<cv::Point> points_fitted_4;  
    std::vector<cv::Point> points_fitted_5;  
    std::vector<cv::Point> points_fitted_6;  
    double pointTemp=0;
    int lanePointX=0;

    //parameters of the lines
    int LaneX=540,LaneY=240;
    int CarPixel=20;
    int ArcX=540,ArcY=290,ArcRadioX=50,ArcRadioY=50;
    int laneFlag=0;
    double arcAngle1=0;
    double arcAngle2=0;
    double LatrateShield=0.1;
    double GPSDistance=0;

    long double t1=0,t2=0,t3=0,t4=0;
    t1=26+35/60.0+31.13/3600.0;
    t2=107+13/60.0+24.66/3600.0;
    t3=26+35/60.0+31.20/3600.0;
    t4=107+13/60.0+24.61/3600.0;
    GPSDistance=Distance(t1, t2, t3, t4);
    printf("GPSDistance: %f\n",GPSDistance);

    SystemInfo[10]=1;
    long double LaneR=0.0;
    
    //test
    long double GPSLat=0;
    long double GPSLat1=0;
    long double GPSLat2=0;
    GPSLat=((long double)((unsigned int)(0xff| (0xff<<8)|(0xff<<16) | (0xff<<24))))
            +((long double)((unsigned int)(0xff| (0xff<<8)|(0xff<<16) | (0xff<<24))))/100000000.0;
    printf("GPSLat:%.9Lf \n",GPSLat);
    GPSLat1=((long double)((unsigned int)(0xff| (0xff<<8)|(0xff<<16) | (0xff<<24))));
    printf("GPSLat1:%.9Lf \n",GPSLat1);
    GPSLat2=((long double)((unsigned int)(0xff| (0xff<<8)|(0xff<<16) | (0xff<<24))))/100000000.0;
    printf("GPSLat2:%.9Lf \n",GPSLat2);

    ros::init(argc, argv, "fog");
    ros::NodeHandle n;

    CvCapture* cvSequence = cvCreateCameraCapture(1);

    IplImage *imInput;

    ros::Subscriber sub = n.subscribe("bounding_box", 1000, BoundingBoxCallback);
    ros::Subscriber sub2 = n.subscribe("esr_data", 1000, EsrCallback);

    int i = 0;
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

    //camer parameters with 70 angle
    cameraMatrix.at<double>(0, 0) = 763.102f;
    cameraMatrix.at<double>(0, 1) = 0.0;
    cameraMatrix.at<double>(0, 2) = 334.835f;
    cameraMatrix.at<double>(1, 0) = 0.0;
    cameraMatrix.at<double>(1, 1) = 766.144f;
    cameraMatrix.at<double>(1, 2) = 209.684f;
    cameraMatrix.at<double>(2, 0) = 0.0;
    cameraMatrix.at<double>(2, 1) = 0.0;
    cameraMatrix.at<double>(2, 2) = 1.0;

    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = -0.366061f;
    distCoeffs.at<double>(1, 0) = -0.500899f;
    distCoeffs.at<double>(2, 0) = 0.00159151f;
    distCoeffs.at<double>(3, 0) = -0.000403139f;
    distCoeffs.at<double>(4, 0) = 2.44183f;

    Mat  map1, map2;
    Size imageSize;
    imInput = cvQueryFrame(cvSequence);
    imageSize = Mat(imInput).size();
    cout << imageSize << endl;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
    imageSize, CV_16SC2, map1, map2);
    Mat undistort, result;
    CvFont font;  
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.5f, 1.5f, 0, 2, CV_AA);
    ESRAimDistance1.clear();
    ESRAimAngle1.clear();
    ESRAimDistance[0].clear();
    ESRAimAngle[0].clear();
	
    LatitudeInfo.clear();
    LongitudeInfo.clear();

    while (char(waitKey(1)) != 'q')
    {
        clock_t start, finish;
        float duration=0;
        start=clock();
        ros::spinOnce();

        imInput = cvQueryFrame(cvSequence);
        #if WINDOWIMGE1
        cv::namedWindow(" ", CV_WINDOW_NORMAL);
        cvMoveWindow(" ",0,0);
        imshow(" ", Mat(imInput));
        #endif	
        Mat undistort2;

        //2-distortion
        remap(Mat(imInput), undistort2, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
        undistort=imInput;
        #if WINDOWIMGE2
        cv::namedWindow("2-undistort image", CV_WINDOW_NORMAL);
        cvMoveWindow("2-undistort image",500,0);
        imshow("2-undistort image", undistort);
        #endif

        SizeH = undistort.rows;
        SizeW = undistort.cols;
        SizeH_W = undistort.rows*undistort.cols;

        Mat img(SizeH, SizeW, CV_32FC3);
        undistort.convertTo(img, CV_32FC3);
        img = img / 255;
        Mat img_W=img(Rect(150,0,350,200));
        dark_channel = DarkChannelPrior(img_W);

        //calculate A
        m++;
        a = Airlight(img_W, dark_channel);
        Mat imh;
        Mat tran;	
        resize(img, imh, Size(), 0.2, 0.2, INTER_LINEAR);
        trans_refine = TransmissionMat(DarkChannelPrior_(imh));	
        Mat tran2 = guidedFilter(imh, trans_refine, 15, 0.0001);//filter
        resize(tran2, tran, Size(), 5.0, 5.0, INTER_LINEAR);
        result = hazefree(img, tran, a, 0.15);

        #if WINDOWIMGE3	
        cv::namedWindow("3-After Distort and Demist", CV_WINDOW_NORMAL);
        imshow("3-After Distort and Demist", result);
        #endif
        laneFlag=0;	
        LaneOverLookimage.setTo(cv::Scalar(100, 100, 100));  

        // first way to draw the line
        #if DRAW_LANE_BY_GPS
        if(SystemInfo[10]<0)
        {
            double Addangel=0;
            double addAngle_yh=0.2;
            int arc_moveY=150;
            for(Addangel =0; Addangel<90; Addangel+=0.2)
            {
                LaneR=200;
                ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2+LaneR, CAMheight- MoveDistortBorder*2-arc_moveY),
                        Size(250, 250),0,180+Addangel,180+Addangel+addAngle_yh,Scalar(255, 255, 255),1,4,0);	
			
                ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2+LaneR, CAMheight- MoveDistortBorder*2-arc_moveY),
                        Size(LaneR+LaneCarWidth*1.5, LaneR+LaneCarWidth*1.5),0,180+Addangel,180+Addangel+addAngle_yh,Scalar(255, 255, 255),1,4,0);	
			
                ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2+LaneR, CAMheight- MoveDistortBorder*2-arc_moveY),
                        Size(LaneR+LaneCarWidth*0.5, LaneR+LaneCarWidth*0.5),0,180+Addangel,180+Addangel+addAngle_yh,Scalar(255, 255, 255),1,4,0);
			
                ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2+LaneR, CAMheight- MoveDistortBorder*2-arc_moveY),
                        Size(LaneR-LaneCarWidth*0.5, LaneR-LaneCarWidth*0.5),0,180+Addangel,180+Addangel+addAngle_yh,Scalar(255, 255, 255),1,4,0);
			
                ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2+LaneR, CAMheight- MoveDistortBorder*2-arc_moveY),
                        Size(LaneR-LaneCarWidth*1.5, LaneR-LaneCarWidth*1.5),0,180+Addangel,180+Addangel+addAngle_yh,Scalar(255, 255, 255),1,4,0);
			
                ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2+LaneR, CAMheight- MoveDistortBorder*2-arc_moveY),
                        Size(LaneR-LaneCarWidth*2.5, LaneR-LaneCarWidth*2.5),0,180+Addangel,180+Addangel+addAngle_yh,Scalar(255, 255, 255),1,4,0);
             }
         }
         else if (SystemInfo[10]>0)
         {
             for(j =0; j<80; j++)
             {
                 LaneR=RadiusMagnify * SystemInfo[3]/SystemInfo[10];//for radius
                 ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2-LaneR-LaneCarWidth*2.5, CAMheight- MoveDistortBorder*2),
                                                 Size(LaneR, LaneR),0,280+j,280+j+1,Scalar(255, 255, 255),1,4,0);	
                 ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2-LaneR-LaneCarWidth*1.5, CAMheight- MoveDistortBorder*2),
                                                 Size(LaneR, LaneR),0,280+j,280+j+1,Scalar(255, 255, 255),1,4,0);	
                 ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2-LaneR-LaneCarWidth*0.5, CAMheight- MoveDistortBorder*2),
                                                 Size(LaneR, LaneR),0,280+j,280+j+1,Scalar(255, 255, 255),1,4,0);
                 ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2-LaneR+LaneCarWidth*0.5, CAMheight- MoveDistortBorder*2),
                                                 Size(LaneR, LaneR),0,280+j,280+j+1,Scalar(255, 255, 255),1,4,0);
                 ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2-LaneR+LaneCarWidth*1.5, CAMheight- MoveDistortBorder*2),
                                                 Size(LaneR, LaneR),0,280+j,280+j+1,Scalar(255, 255, 255),1,4,0);
                 ellipse(LaneOverLookimage,Point(LaneOverLookimageWidth/2-LaneR+LaneCarWidth*2.5, CAMheight- MoveDistortBorder*2),
                                                 Size(LaneR, LaneR),0,280+j,280+j+1,Scalar(255, 255, 255),1,4,0);
             }
         }
        else
        {
            int moveY=0,dashedline=5;
            for(j =0; j<460; j+=dashedline*1)
            {	
                line(LaneOverLookimage,Point(LaneOverLookimageWidth/2-LaneCarWidth*2.5,CAMheight-MoveDistortBorder*2-j-dashedline-moveY), 
                    Point(LaneOverLookimageWidth/2-LaneCarWidth*2.5,CAMheight- MoveDistortBorder*2-j-moveY), Scalar(255, 255, 255), 1);

                line(LaneOverLookimage,Point(LaneOverLookimageWidth/2-LaneCarWidth*1.5,CAMheight-MoveDistortBorder*2-j-dashedline-moveY), 
                    Point(LaneOverLookimageWidth/2-LaneCarWidth*1.5,CAMheight- MoveDistortBorder*2-j-moveY), Scalar(255, 255, 255), 1);

                line(LaneOverLookimage,Point(LaneOverLookimageWidth/2-LaneCarWidth*0.5,CAMheight-MoveDistortBorder*2-j-dashedline-moveY), 
                    Point(LaneOverLookimageWidth/2-LaneCarWidth*0.5,CAMheight- MoveDistortBorder*2-j-moveY), Scalar(255, 255, 255), 1);

                line(LaneOverLookimage,Point(LaneOverLookimageWidth/2+LaneCarWidth*0.5,CAMheight-MoveDistortBorder*2-j-dashedline-moveY), 
                    Point(LaneOverLookimageWidth/2+LaneCarWidth*0.5,CAMheight- MoveDistortBorder*2-j-moveY), Scalar(255, 255, 255), 1);

                line(LaneOverLookimage,Point(LaneOverLookimageWidth/2+LaneCarWidth*1.5,CAMheight-MoveDistortBorder*2-j-dashedline-moveY), 
                    Point(LaneOverLookimageWidth/2+LaneCarWidth*1.5,CAMheight- MoveDistortBorder*2-j-moveY), Scalar(255, 255, 255), 1);

                line(LaneOverLookimage,Point(LaneOverLookimageWidth/2+LaneCarWidth*2.5,CAMheight-MoveDistortBorder*2-j-dashedline-moveY), 
                    Point(LaneOverLookimageWidth/2+LaneCarWidth*2.5,CAMheight- MoveDistortBorder*2-j-moveY), Scalar(255, 255, 255), 1);
            }	
        }
        #endif

        #if DRAW_LANE_BY_GPS_AND_ESR
        //store data for 5 seconds
        if(LatitudeInfo.size()>=MaxDrawPoint)
        {
            LatitudeInfo.erase(LatitudeInfo.begin());//delete the first one
            LatitudeInfo.push_back(SystemInfo[1]);  	
        }
        else
        {
            LatitudeInfo.push_back(SystemInfo[1]); 
        }
        if(LongitudeInfo.size()>=MaxDrawPoint)
        {
            LongitudeInfo.erase(LongitudeInfo.begin());
            LongitudeInfo.push_back(SystemInfo[2]);  
        }
        else
        {
            LongitudeInfo.push_back(SystemInfo[2]);  
        }

        #endif
        LaneCarMinDistance=10000;
        SystemInfo[7]=SystemInfo[7]-6;
        if(SystemInfo[7]<0)
        {
            SystemInfo[7]+=360;
        }
        for(ID_k=0;ID_k<64;ID_k++)
        {
            AimInfoArray_rear[ID_k].aim_ID =AimInfoArray_front[ID_k].aim_ID;
            AimInfoArray_rear[ID_k].aim_IsAlive =AimInfoArray_front[ID_k].aim_IsAlive;
            AimInfoArray_rear[ID_k].remap_J =AimInfoArray_front[ID_k].remap_J;
            AimInfoArray_rear[ID_k].aim_distance =AimInfoArray_front[ID_k].aim_distance;
            AimInfoArray_rear[ID_k].aim_angel =AimInfoArray_front[ID_k].aim_angel;
        }

        for(ID_k=0;ID_k<64;ID_k++)
        {
            AimInfoArray_front[ID_k].aim_ID =0xff;
            AimInfoArray_front[ID_k].aim_IsAlive =false;
            AimInfoArray_front[ID_k].remap_J =0xff;
            AimInfoArray_front[ID_k].aim_distance =0;
            AimInfoArray_front[ID_k].aim_angel =0;
        }

        for(j =0; j<number2; j++)
        {
            if(result2[j*8+4] < AimColorByDistance1)			
            {
                rectangle(result, cvPoint((int)result2[j*8], (int)result2[j*8+1]), cvPoint((int)result2[j*8+2], 
                          (int)result2[j*8+3]), Scalar(0, 0, 255), 2, 8, 0);

                circle(LaneOverLookimage, cvPoint((int)(LaneOverLookimageWidth/2 + LaneCarWidth/4 * result2[j*8+4]*sin(result2[j*8+5]/180.0*M_PI)),
                          (int)(CAMheight- MoveDistortBorder*2 - 2 * result2[j*8+4]*cos(result2[j*8+5]/180.0*M_PI))), 7,Scalar(0,0,255),2,8,0);

            }
            else if(result2[j*8+4] < AimColorByDistance2)
            {
                rectangle(result, cvPoint((int)result2[j*8], (int)result2[j*8+1]), cvPoint((int)result2[j*8+2], 
                          (int)result2[j*8+3]), Scalar(255, 255, 0), 2, 8, 0);

                circle(LaneOverLookimage, cvPoint((int)(LaneOverLookimageWidth/2 + LaneCarWidth/4 * result2[j*8+4]*sin(result2[j*8+5]/180.0*M_PI)),
                          (int)(CAMheight- MoveDistortBorder*2 - 2 * result2[j*8+4]*cos(result2[j*8+5]/180.0*M_PI))), 7,Scalar(255,255,0),2,8,0);
	
            }	
            else
            {
                rectangle(result, cvPoint((int)result2[j*8], (int)result2[j*8+1]), cvPoint((int)result2[j*8+2], 
                          (int)result2[j*8+3]), Scalar(0, 255, 0), 2, 8, 0);
	
                circle(LaneOverLookimage, cvPoint((int)(LaneOverLookimageWidth/2 + LaneCarWidth/4 * result2[j*8+4]*sin(result2[j*8+5]/180.0*M_PI)),
                          (int)(CAMheight- MoveDistortBorder*2 - 2 * result2[j*8+4]*cos(result2[j*8+5]/180.0*M_PI))), 7,Scalar(0,255,0),2,8,0);
            }
	
            lanePointX=(int)(LaneOverLookimageWidth/2 + LaneCarWidth/4 * result2[j*8+4]*sin(result2[j*8+5]/180.0*M_PI));

            if(result2[j*8+4] < 10000)
            {
                putText(result,toString(result2[j*8+4]),Point((int)result2[j*8]+5, (int)result2[j*8+1]-5),
                        FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,0,0),1,1);
            }
			
            if(LaneCarMinDistance > result2[j*8+4] )//get the minium
            {
                LaneCarMinDistance=result2[j*8+4] * cos(result2[j*8+5]/180.0 * M_PI);				
            }
	
            AimInfoArray_front[(int)result2[j*8+7]].aim_IsAlive=true;	
            AimInfoArray_front[(int)result2[j*8+7]].aim_ID=(int)result2[j*8+7];
            AimInfoArray_front[(int)result2[j*8+7]].remap_J=j;
            AimInfoArray_front[(int)result2[j*8+7]].aim_distance=result2[j*8+4];
            AimInfoArray_front[(int)result2[j*8+7]].aim_angel=result2[j*8+5]+SystemInfo[7];
            printf("AimInfoArray_front[ID_k].aim_ID %d \n",AimInfoArray_front[(int)result2[j*8+7]].aim_ID);
        }

        // draw all
        for(ID_k=0;ID_k<64;ID_k++)
        {
            if(AimInfoArray_rear[ID_k].aim_IsAlive==true)
            {
                points_fitted.clear();
                points_fitted_1.clear();
                points_fitted_2.clear();
                points_fitted_3.clear();
                points_fitted_4.clear();
                points_fitted_5.clear();
                points_fitted_6.clear();

            if(AimInfoArray_front[ID_k].aim_IsAlive==true)	
            {
                if(ESRAimDistance[ID_k].size()>=100)
                {
                    ESRAimDistance[ID_k].erase(ESRAimDistance[ID_k].begin());//delete the first member
                    ESRAimAngle[ID_k].erase(ESRAimAngle[ID_k].begin());
                    ESRAimDistance[ID_k].push_back(AimInfoArray_front[ID_k].aim_distance);  
                    ESRAimAngle[ID_k].push_back(AimInfoArray_front[ID_k].aim_angel);  
                }
                else
                {
                    ESRAimDistance[ID_k].push_back(AimInfoArray_front[ID_k].aim_distance);  
                    ESRAimAngle[ID_k].push_back(AimInfoArray_front[ID_k].aim_angel);  
                }
            }
            else
            {
                ESRAimDistance[ID_k].clear();
                ESRAimAngle[ID_k].clear();
            }
            Lane_point_count=ESRAimDistance[ID_k].size();

            if(Lane_point_count>=MaxDrawPoint)
            {
                Lane_point_count=MaxDrawPoint;
            }
            Lane_point_draw=Lane_point_count/2>0?Lane_point_count/2:1;

            if(ESRAimDistance[ID_k].size()>=1 && ESRAimAngle[ID_k].size()>=1)
            {
                printf("11111SystemInfo[7] %Lf\n",SystemInfo[7]);
                points.clear();
                for(j=0;j<Lane_point_draw-2;j++)
                    {
                    GPS_DistancePoint[j] = Distance(LatitudeInfo[Lane_point_count/Lane_point_draw*(j+1)-1], 
                                           LongitudeInfo[Lane_point_count/Lane_point_draw*(j+1)-1], 
                                           LatitudeInfo[Lane_point_count-1], LongitudeInfo[Lane_point_count - 1]);

                    GPS_AnglePoint[j]=GetGPSAngle(LatitudeInfo[Lane_point_count/Lane_point_draw*(j+1)-1],
                                           LongitudeInfo[Lane_point_count/Lane_point_draw*(j+1)-1],
                                           LatitudeInfo[Lane_point_count - 1], LongitudeInfo[Lane_point_count - 1]);
                    }
                    GPS_DistancePoint[Lane_point_draw-1] = Distance(LatitudeInfo[Lane_point_count/Lane_point_draw*Lane_point_draw-1], 
                                  LongitudeInfo[Lane_point_count/Lane_point_draw*Lane_point_draw-1], 
                                  LatitudeInfo[Lane_point_count-1], LongitudeInfo[Lane_point_count - 1]);

                    GPS_AnglePoint[Lane_point_draw-1]=GetGPSAngle(LatitudeInfo[Lane_point_count/Lane_point_draw * (Lane_point_draw-1) -1], 
                    LongitudeInfo[Lane_point_count/Lane_point_draw * (Lane_point_draw-1) -1], 
                    LatitudeInfo[Lane_point_count - 1], LongitudeInfo[Lane_point_count - 1]);

                    printf("GPS_DistancePoint[0] %Lf  GPS_AnglePoint[0] %Lf \n",GPS_DistancePoint[0],GPS_AnglePoint[0]);

                    printf("SystemInfo[7] %Lf\n",SystemInfo[7]);
                    
                    Rotate_Angle_theta = SystemInfo[7]/180.0 * M_PI;
                    printf("Rotate_Angle_theta %Lf\n",Rotate_Angle_theta);
                    for(j=0;j<Lane_point_draw-2;j++)
                    {
                        Absolute_X= 5*(ESRAimDistance[ID_k][Lane_point_count/Lane_point_draw*(j+1)-1]
                        *sin(ESRAimAngle[ID_k][Lane_point_count/Lane_point_draw*(j+1)-1]/180.0 * M_PI)-GPS_DistancePoint[j]*sin(GPS_AnglePoint[j]/180.0 * M_PI));

                        Absolute_Y= 2*(ESRAimDistance[ID_k][Lane_point_count/Lane_point_draw*(j+1)-1]*
                                cos(ESRAimAngle[ID_k][Lane_point_count/Lane_point_draw*(j+1)-1]/180.0 * M_PI)
                                -GPS_DistancePoint[j]*cos(GPS_AnglePoint[j]/180.0 * M_PI));
                        Relative_X = Absolute_X * cos(Rotate_Angle_theta) + Absolute_Y * sin(Rotate_Angle_theta);
                        Relative_Y = Absolute_Y * cos(Rotate_Angle_theta) - Absolute_X * sin(Rotate_Angle_theta);

                        points.push_back(cv::Point((int)(Relative_X+LaneOverLookimageWidth/2),(int)CAMheight-MoveDistortBorder*2-Relative_Y));  
                    }

                    Absolute_X= 5*( ESRAimDistance[ID_k][Lane_point_count - 1]*sin(ESRAimAngle[ID_k][Lane_point_count - 1]/180.0 * M_PI));
                    Absolute_Y= 2 * ( ESRAimDistance[ID_k][Lane_point_count - 1]*cos(ESRAimAngle[ID_k][Lane_point_count - 1]/180.0 * M_PI));
                    Relative_X = Absolute_X * cos(Rotate_Angle_theta) + Absolute_Y * sin(Rotate_Angle_theta);
                    Relative_Y = Absolute_Y * cos(Rotate_Angle_theta) - Absolute_X * sin(Rotate_Angle_theta);
                    points.push_back(cv::Point((int)(Relative_X+LaneOverLookimageWidth/2),(int)CAMheight-MoveDistortBorder*2-Relative_Y));  

                    pointsCount=points.size()-1;
                    for (int i = 0; i < pointsCount+1; i++)  
                    {  
                       points_fitted_test=points[i];
                        if(points_fitted_test.y<500)
                        {
                            for(int yh_i=0;yh_i<360;yh_i+=60)
                            {
                                ellipse(LaneOverLookimage,points_fitted_test,Size(5, 5),0, 180+yh_i,180+yh_i+15,Scalar(0, 0, 255),1, 8, 0);	
                            }
                            printf("ID_k %d \n",ID_k);
                            printf("AimInfoArray_front[ID_k].aim_ID %d \n",AimInfoArray_front[ID_k].aim_ID);
                        }

                        pointTemp=points[i].x;
                        points[i].x=points[i].y;
                        points[i].y=pointTemp;
                        printf("points.X %d  points.Y %d \n",points[i].x,points[i].y);
                    } 

                    polynomial_curve_fit(points, 3, LaneA);  
                    printf("points[0].x %d points[pointsCount].x %d\n",points[0].x,points[pointsCount].x);
  
                    if(points[0].x < points[pointsCount].x)
                    {
                        for (int x = points[0].x-1; x < points[pointsCount].x; x++)  
                        {  
                            double y = LaneA.at<double>(0, 0) + LaneA.at<double>(1, 0) * x +  
                                   LaneA.at<double>(2, 0)*std::pow(x, 2) + LaneA.at<double>(3, 0)*std::pow(x, 3);  

                            points_fitted.push_back(cv::Point(y, x));	
                            points_fitted_1.push_back(cv::Point(y-60, x)); 
                            points_fitted_2.push_back(cv::Point(y-40, x)); 
                            points_fitted_3.push_back(cv::Point(y-20, x)); 
                            points_fitted_4.push_back(cv::Point(y+20, x)); 
                            points_fitted_5.push_back(cv::Point(y+40, x)); 

                        }  
                    }
                    else
                    {
                        for (int x =points[pointsCount].x-1; x < points[0].x; x++)  
                        {  
                            double y = LaneA.at<double>(0, 0) + LaneA.at<double>(1, 0) * x +  
                                       LaneA.at<double>(2, 0)*std::pow(x, 2) + LaneA.at<double>(3, 0)*std::pow(x, 3);  

                            points_fitted.push_back(cv::Point(y, x));	
                            points_fitted_1.push_back(cv::Point(y-60, x)); 
                            points_fitted_2.push_back(cv::Point(y-40, x)); 
                            points_fitted_3.push_back(cv::Point(y-20, x)); 
                            points_fitted_4.push_back(cv::Point(y+20, x)); 
                            points_fitted_5.push_back(cv::Point(y+40, x)); 
                        }  
                    }
   
                    if(points_fitted.size()>0 && points_fitted_1.size()>0 && points_fitted_2.size()>0 )
                    {
                        cv::polylines(LaneOverLookimage, points_fitted, false, cv::Scalar(255, 255, 0), 1, 8, 0);  
                    }		
                }
            }
            else
            {
                ESRAimDistance[ID_k].clear();  
                ESRAimAngle[ID_k].clear();  
            }
        }

        #if DRAW_LANE_BY_GPS_AND_ESR 
        circle(LaneOverLookimage, cvPoint(100,450), 7,Scalar(255,255,0),2,8,0);
        #endif	

        Rect rect(MoveDistortBorder, MoveDistortBorder, result.cols-MoveDistortBorder * 2 +1,result.rows-MoveDistortBorder* 2 + 1);
		Mat resultCut = result(rect);
        #if WINDOWIMGE4   	
        namedWindow("  ", 0);
        cvMoveWindow("  ",200,500);
        imshow("  ", resultCut);
        #endif

        // combine
        Size size(resultCut.cols + LaneOverLookimage.cols, MAX(resultCut.rows, LaneOverLookimage.rows));
        img_merge.create(size, CV_MAKETYPE(resultCut.depth(), 3));
        img_merge = Scalar::all(0);
        Mat outImg_left, outImg_right;

        // set the ROI
        outImg_left = img_merge(Rect(0, 0, resultCut.cols, resultCut.rows));
        outImg_right = img_merge(Rect(resultCut.cols, 0, LaneOverLookimage.cols, LaneOverLookimage.rows));

        // copy
        resultCut.copyTo(outImg_left);
        LaneOverLookimage.copyTo(outImg_right);

        #if WINDOWIMGE5
        namedWindow("   ", 0);
        cvMoveWindow("   ",800,500);
        imshow("   ", img_merge);
        #endif

        finish=clock();
        duration=(float)(finish-start)/CLOCKS_PER_SEC;
        cout << "time cost: " <<duration << endl;

        if(char(waitKey(1)) == 'q') 
            break;             
    }

    cvReleaseCapture(&cvSequence);
    cvReleaseImage(&imInput);
    return 0;
}

