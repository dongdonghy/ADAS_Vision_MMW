/*	
Functions: dealing the ESR's data to give the inforamtion of cars in lane 

input: 	camera image (640x480), sensors data include the ESR/IMU/SEC/GPS and etc.
output: the location and size in the camera image match with the sensors
*/


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
#include <math.h>

//camera paramter
#define CAMwidth 640  
#define CAMheight 480

//these parameters could change the effect of rectangles' Location and Size by dealing with the ESR's data 
#define ShieldSpeed 0.5
#define DistanceShield 175
#define MaxSizeRectWidth 130
#define MinSizeRectWidth 0
#define YVauleAdjust 30
#define MergeAngle 3
#define	MergeDistance 50

using namespace std;
//the usb port object 
serial::Serial ser;
//ros message object	    
std_msgs::String result;     

//these paramters is definded to receive the GPS's data
long double GPSlatitude=0;
long double GPSlatitude_Decimal=0;
long double GPSlongitude=0;
long double GPSlongitude_Decimal=0;
long double GPSSpeed=0;
long double GPSYaw=0;
double  GPSV=0;
int GPSlongitude_int=0;
int GPSlatitude_int=0;

//check the ESR's data and store it 
int data_flag=1;
unsigned char id=0;
unsigned char checkSum;
unsigned char candata[12];

//IMU struct which is detect the Angle change speed of our drving car
typedef struct IMU
{
    double Wx;
    double Wy;
    double Wz;
}IMU_JY901;
IMU_JY901 IMUJY901;

//SEC struct which is detect the Angle of our drving car
typedef struct SEC
{
    double angleX;
    double angleY;
    double angleZ;
}SEC_315;
SEC_315 SEC315[2];

//_CAN_DATA struct which is the storage of detectting data by ESR sensor
typedef struct _CAN_DATA
{
    bool onComing;
    bool grouping_change;
    double latrate;
    double angle;
    unsigned char status; 
    double distance;
    double range_accel;
    double AimWidth;
    bool roling_count;
    bool brige_object;
    double speed;
    unsigned char range_mode;
}CAN_DATA;
CAN_DATA CanData[64];

//_RECTANGLE struct which is match the detecetted cars in the camera image by sensors' data 
typedef struct _RECTANGLE
{
    double x;
    double y;
    double width;
    double height;
    bool IsAlive;
}RECTANGLE;
RECTANGLE Rect[64];

//A mechanism to check the sensors' data is correct in the serial transmit
enum StateMachine
{
    Header,      
    Indicator,
    Data,
    Check,
    End
};
enum StateMachine currentState = Header;

//Valuing the struct CanData/SEC315/IMUJY901/GPS from SensorData which has been checked
void OnMessageReceived(unsigned char RecCanData[12])
{
    id=RecCanData[1];

    //receive the ESR sensor data
    if(id<64) 
    {
        CanData[id].onComing= ((bool)(RecCanData[2] & 0x01));
        CanData[id].grouping_change= (bool)((RecCanData[2] & 0x02)>>1);
        CanData[id].latrate=((double)((char)(RecCanData[2] & 0xfc)))/4*0.025;
        CanData[id].angle=((double)((short)((((RecCanData[3] & 0x1f)<< 8) |( RecCanData[4] & 0xf8))<<3)))/64*0.1;
        CanData[id].status=(RecCanData[3] & 0xe0)/32;
        CanData[id].distance=((double)((((RecCanData[4] & 0x07)<< 8) |( RecCanData[5]))))*0.1;	
        CanData[id].range_accel=((double)((short)((((RecCanData[6] & 0x03)<< 8) |( RecCanData[7]))<<6)))/64*0.05;	
        CanData[id].AimWidth=((double)((double)((RecCanData[6] & 0x3c))))/4*0.5;
        CanData[id].roling_count= ((bool)(RecCanData[6] & 0x40)>>6);
        CanData[id].brige_object= ((bool)(RecCanData[6] & 0x80)>>7);
        CanData[id].speed=((double)((short)((((RecCanData[8] & 0x3f)<< 8) |( RecCanData[9]))<<2)))/4*0.01;
        CanData[id].range_mode=(RecCanData[8] & 0xc0)/64;
    }
    //receive the SEC315 sensor data
    else if(id==0x80||id==0x81) 
    {
        SEC315[id-0X80].angleX=((double)((short)(RecCanData[2] | (RecCanData[3]<<8))))/10.0;
        SEC315[id-0X80].angleY=((double)((short)(RecCanData[4] | (RecCanData[5]<<8))))/10.0;
        SEC315[id-0X80].angleZ=((double)((short)(RecCanData[6] | (RecCanData[7]<<8))))/10.0;	
    }
    //receive the IMUJY901 sensor data 
    else if(id==0x82)
    {
    IMUJY901.Wx=((double)((short)(RecCanData[2] | (RecCanData[3]<<8))))/32768.0*2000;
    IMUJY901.Wy=((double)((short)(RecCanData[4] | (RecCanData[5]<<8))))/32768.0*2000;
    IMUJY901.Wz=((double)((short)(RecCanData[6] | (RecCanData[7]<<8))))/32768.0*2000;
    }
    //receive the GPS speed data by the weak GPS 
    else if(id==0x83)
    {
        GPSV=((double)((RecCanData[2]| (RecCanData[3]<<8)|(RecCanData[4]<<16) | (RecCanData[5]<<24))))/3600.0;
    }
    //receive the GPS latitude data by the strong GPS 
    else if(id==0x84)
    {
        GPSlatitude_int=((long double)((unsigned int)(RecCanData[2]| (RecCanData[3]<<8)|(RecCanData[4]<<16) | (RecCanData[5]<<24))));
        GPSlatitude_Decimal=((long double)(GPSlatitude_int % 100))/60.0 + 
                            ((long double)((unsigned int)(RecCanData[6]| (RecCanData[7]<<8)|(RecCanData[8]<<16) | (RecCanData[9]<<24))))/100000000.0/60.0;
        GPSlatitude_int=GPSlatitude_int/100;
        GPSlatitude=GPSlatitude_int + GPSlatitude_Decimal;
    }
    //receive the GPS longitude data by the strong GPS
    else if(id==0x85) 
    {
        GPSlongitude_int=((long double)((unsigned int)(RecCanData[2]| (RecCanData[3]<<8)|(RecCanData[4]<<16) | (RecCanData[5]<<24))));		
        GPSlongitude_Decimal=((long double)(GPSlongitude_int % 100))/60.0 
                           + ((long double)((unsigned int)(RecCanData[6]| (RecCanData[7]<<8)|(RecCanData[8]<<16) | (RecCanData[9]<<24))))/100000000.0/60.0;	
        GPSlongitude_int=GPSlongitude_int/100;
        GPSlongitude=GPSlongitude_int + GPSlongitude_Decimal;
    }
    //receive the GPS Speed and Yaw data by the strong GPS
    else if(id==0x86) 
    {	
        GPSSpeed=((long double)((unsigned int)(RecCanData[2]| (RecCanData[3]<<8)|(RecCanData[4]<<16) | (RecCanData[5]<<24))))/1000.0*1.852/3.6;
        GPSYaw=((long double)((unsigned int)(RecCanData[6]| (RecCanData[7]<<8)|(RecCanData[8]<<16) | (RecCanData[9]<<24))))/1000.0;
    }
}

//DecodeData the sensor data 
void DecodeData(unsigned char data[],int dataLength)
{
    for (int i = 0; i < dataLength; i++)
    {	
        switch (currentState) 
        {
            case Header:
                if (data[i] == 0xef) 
                {
                    currentState = Indicator;
                    checkSum = 0;
	   				candata[0]=data[i];
                }
                break;
            case Indicator:
                if (data[i]<0xff)
                {
                    currentState = Data; 
	    			candata[1]=data[i];
                }
                else 
                {
                    currentState = Header;
                }
                break;
            case Data:	
		data_flag++;
		if(data_flag<10)
		{	
			candata[data_flag]=data[i];
		}
		if(data_flag==9)
		{
			currentState = Check;
			data_flag=1;
		}			
                break;
            case Check:
		checkSum=candata[1]+candata[2]+candata[3]+candata[4]+candata[5]+candata[6]+candata[7]+candata[8]+candata[9];
                if ( checkSum  == data[i])
                {
			candata[10]=data[i];
                	currentState = End;
                }
                else 
                {
                    currentState = Header; 
                }
                break;
            case End:
                if (data[i] == 0xff)		
                {
		    candata[11]=data[i];
                    OnMessageReceived(candata); 
		    for(int m=0;m<12;m++)
		    {
			candata[m]=0;
		    }
                }
                currentState = Header;
                break;
            default:
                currentState = Header;
                break;
        }
    }
}

int main(int argc, char **argv)
{
    //these variables are designed for loop program
    int i,j,layer1,layer2,aimFlg,DataCnt;
    //ponits' coordinate
    int PointA_X1, PointA_Y1, PointA_X2, PointA_Y2, PointB_X1, PointB_Y1, PointB_X2, PointB_Y2, POINT_X1, POINT_Y1, POINT_X2, POINT_Y2;

    //ros init the esr node
    ros::init(argc, argv, "esr");
    ros::NodeHandle nh;
    ros::Publisher esr = nh.advertise<std_msgs::Float64MultiArray>("esr_data", 1000);
    std_msgs::Float64MultiArray esr_data;
    esr_data.data.resize(395);
    ros::Rate loop_rate(20);
	
    //open the serial port by the device file in the computer and set the serial baudrate 
    try
    {
        ser.setPort("/dev/ttyUSB0");		
        ser.setBaudrate(230400);			
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    //the main loop the dealing the sensors' data 
    while(ros::ok())
    {	
        //get the serial port data and make sure its' validity
        if(ser.available())
        {
            result.data  = ser.read(ser.available());
            unsigned char serdata[result.data.size()];
            for( j=0; j< result.data.size(); j++)
            {
                serdata[j] = (unsigned uint8_t) result.data[j];
            }
                //DecodeData the sensor data 
                DecodeData(serdata,result.data.size());	
        }	
        //reset the rectangle array
        for (i = 0; i < 64; i++)
        {
            Rect[i].x = 0;
            Rect[i].width = 0;
            Rect[i].height = 0;
            Rect[i].y = 0;	
            Rect[i].IsAlive=false;
        }
        //1 get the Rect Array from the Dlphi ESR sensor
        for (i = 0; i < 64; i++)
        {
            if ((abs(CanData[i].angle)<23) && (CanData[i].grouping_change==false) && (CanData[i].status==4 || CanData[i].status==3) 
                 &&abs((CanData[i].distance) < DistanceShield) && (abs(CanData[i].speed )> ShieldSpeed)&& (CanData[i].speed !=81.91))	
            {
                Rect[i].x = 763.102 * tan(CanData[i].angle/180.0 * M_PI) + 344.835; 
                Rect[i].width = 513.102 *2 / (CanData[i].distance * cos(CanData[i].angle / 180.0 *M_PI));
                if(Rect[i].width > MaxSizeRectWidth)
                {
                    Rect[i].width = MaxSizeRectWidth;
                }
                else if(Rect[i].width < MinSizeRectWidth)
                {
                    Rect[i].width = MinSizeRectWidth;
                }  
                if(CanData[i].AimWidth > 1)
                {
                    Rect[i].height = Rect[i].width * 2/ 1.8;
                    Rect[i].width=Rect[i].width*CanData[i].AimWidth;
                    Rect[i].y = CAMheight/2- Rect[i].width-YVauleAdjust;	
                    Rect[i].x = Rect[i].x- Rect[i].width/2;
                }
                else
                {
                    Rect[i].height = Rect[i].width*2 /1.8;
                    Rect[i].y = CAMheight/2- Rect[i].height/2-YVauleAdjust;	
                    Rect[i].width=Rect[i].width*1.5;;
                    Rect[i].x = Rect[i].x- Rect[i].width/2;
                }    				
                Rect[i].IsAlive=true;
            }
            else
            {
                Rect[i].IsAlive=false;
            }
        }
        //2 Merging the Rect Array data
        for ( layer1 = 0; layer1 < 0; layer1++)
        {
            if (Rect[layer1].IsAlive == true)
            {
                for ( layer2 = layer1 + 1; layer2 < 64; layer2++)
                {
                    if (Rect[layer2].IsAlive==true)
                    {
                        if ((MergeAngle > abs(CanData[layer1].angle - CanData[layer2].angle))
                             &&  MergeDistance > abs(CanData[layer1].distance - CanData[layer2].distance))
                        {		                                    
                            PointA_X1 = Rect[layer1].x;
                            PointA_Y1 = Rect[layer1].y;
                            PointA_X2 = Rect[layer1].x + Rect[layer1].width;
                            PointA_Y2 = Rect[layer1].y + Rect[layer1].height;
                            PointB_X1 = Rect[layer2].x;
                            PointB_Y1 = Rect[layer2].y;
                            PointB_X2 = Rect[layer2].x + Rect[layer2].width;
                            PointB_Y2 = Rect[layer2].y + Rect[layer2].height;
                            if (PointA_X1 < PointB_X1)
                            {
                                POINT_X1 = PointA_X1;
                            }
                            else
                            {
                                POINT_X1 = PointB_X1;
                            }
                            if (PointA_Y1 < PointB_Y1)
                            {
                                POINT_Y1 = PointA_Y1;
                            }
                            else
                            {
                                POINT_Y1 = PointB_Y1;
                            }
                            if (PointA_X2 > PointB_X2)
                            {
                                POINT_X2 = PointA_X2;
                            }
                            else
                            {
                                POINT_X2 = PointB_X2;
                            }
                            if (PointA_Y2 > PointB_Y2)
                            {
                                POINT_Y2 = PointA_Y2;
                            }
                            else
                            {
                                POINT_Y2 = PointB_Y2;
                            }
                            if (abs(CanData[layer1].speed)<(CanData[layer2].speed))
                            {
                                CanData[layer1].speed = CanData[layer2].speed;
                            }
                            if (abs(CanData[layer1].distance) > (CanData[layer2].distance))
                            {
                                CanData[layer1].distance = CanData[layer2].distance;
                            }
                            if (abs(CanData[layer1].latrate) < (CanData[layer2].latrate))
                            {
                                CanData[layer1].latrate = CanData[layer2].latrate;
                            }
                            Rect[layer1].x = POINT_X1;
                            Rect[layer1].y = POINT_Y1;
                            Rect[layer1].width = POINT_X2 - POINT_X1;
                            Rect[layer1].height = POINT_Y2 - POINT_Y1;
                            Rect[layer2].IsAlive = false;
                        }
                    }
                }
            }
        }
        //3 Valuing the system parameters
        esr_data.data[1] = GPSlatitude;			
        esr_data.data[2] = GPSlongitude;
        esr_data.data[3] = GPSSpeed;
        esr_data.data[4] = GPSYaw;
        esr_data.data[5] = SEC315[0].angleX;
        esr_data.data[6] = SEC315[0].angleY;
        esr_data.data[7] = SEC315[0].angleZ;
        esr_data.data[8] = IMUJY901.Wx;
        esr_data.data[9] = IMUJY901.Wy;
        esr_data.data[10] = IMUJY901.Wz;
        DataCnt=10;
        //4 Valuing the transmissive data 
        for( aimFlg=0;aimFlg<64;aimFlg++)
        {
            if(Rect[aimFlg].IsAlive == true)
            {
                esr_data.data[DataCnt+1] =Rect[aimFlg].x;
                esr_data.data[DataCnt+2] =Rect[aimFlg].y;
                esr_data.data[DataCnt+3] =Rect[aimFlg].x+Rect[aimFlg].width;
                esr_data.data[DataCnt+4] =Rect[aimFlg].y+Rect[aimFlg].height;
                esr_data.data[DataCnt+5] =CanData[aimFlg].distance;		
                esr_data.data[DataCnt+6] =CanData[aimFlg].angle;
                esr_data.data[DataCnt+7] =CanData[aimFlg].onComing;
                esr_data.data[DataCnt+8] =aimFlg;
                DataCnt+=8;
            }
        }
        esr_data.data[0]=(double)(DataCnt-10)/8;//record the num of CAN data after dealing 
        esr_data.data.resize(DataCnt+1);
        esr.publish(esr_data);
        loop_rate.sleep();      	 	
    }
    return 0;
}

