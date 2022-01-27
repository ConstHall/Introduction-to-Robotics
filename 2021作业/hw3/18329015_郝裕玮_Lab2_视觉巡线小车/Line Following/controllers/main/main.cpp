#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <iostream> 

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include<string.h>

using namespace std;
using namespace webots;

int main() {
	Motor *motors[4];//电机和键盘都要用webots给的类型
	webots::Keyboard keyboard;
	char wheels_names[4][8]={"motor1","motor2","motor3","motor4"};//对应RotationMotor里的句柄

	Robot *robot=new Robot();//使用webots的机器人主体
	Camera *camera = robot->getCamera("camera");//获取相机，句柄名为camera
	camera->enable(1);//设置相机每1ms更新1次

	keyboard.enable(1);//运行键盘输入设置频率是1ms读取一次

    GPS* gps = robot->getGPS("gps");//获取GPS，句柄名为gps
	gps->enable(1);//设置GPS每1ms更新1次

	double speed[4];//此数组会在后面赋值给电机以速度
	double velocity=10;//初速度
    int i;
	double cur_speed=0;//瞬时速度
	double avg_speed=0;//平均速度
	double speed_sum=0;

    //初始化
    for(i=0;i<=3;i++){
		motors[i]=robot->getMotor(wheels_names[i]);//按照你在仿真器里面设置的名字获取句柄
		motors[i]->setPosition(std::numeric_limits<double>::infinity());
		motors[i]->setVelocity(0.0);//设置电机一开始处于停止状态
		speed[i]=3;//给予小车一个初速度   
    }

	double speed_forward[4]={velocity,velocity,velocity,velocity};//前进方向
	double speed_leftCircle[4]={velocity,-velocity,-velocity,velocity};//左自旋(即左转弯)
	double speed_rightCircle[4]={-velocity,velocity,velocity,-velocity};//右自旋(即右转弯)

	int timeStep=(int)robot->getBasicTimeStep();//获取你在webots设置一帧的时间
	int cnt=0;//统计帧数
	double error_sum=0;//记录瞬时的半径累加和
	double result=0;//记录误差
	double x0=0,y0=1.22;//手动大致测量圆心坐标
	double cur_x,cur_y;//记录GPS的实时坐标
	double pi;//当前的瞬时半径
    double cur_error;//当前的瞬时误差
    double r=1.185;//手动测量的真实半径

	while(robot->step(timeStep)!=-1){//仿真运行一帧
        cnt++;//统计帧数
    	const unsigned char *a=camera->getImage();//读取相机抓取的最后一张图像。图像被编码为三个字节的序列，分别代表像素的红、绿、蓝

		int length=camera->getWidth();//图像长度
		int width=camera->getHeight();//图像宽度
		cout<<"图像尺寸为："<<length<<'*'<<width<<endl;//输出图像尺寸

        int b1,b2,b3,b4;
        b1=length*3*width/2;//图像中间一行的最左边的像素点
        b2=length*3*width/2+(width/2+3)*3;//图像中间一行的中间靠左的某像素点
        b3=length*3*width/2+(width/2+5)*3;//图像中间一行的中间靠右的某像素点
        b4=length*3*width/2+(length-1)*3;//图像中间一行的最右边的某像素点
        //其中b1，b2代表图像中间的黑色轨迹的两侧(大致估计)

		cur_x=gps->getValues()[0];//获取实时x坐标
		cur_y=gps->getValues()[1];//获取实时y坐标

		pi=sqrt((cur_x-x0)*(cur_x-x0)+(cur_y-y0)*(cur_y-y0));//计算当前的瞬时半径
		cur_error=abs(pi-r);//实时误差=|pi-r|
		error_sum+=cur_error;//误差累加
		result=error_sum/cnt;//计算当前的平均误差

		cur_speed=gps->getSpeed();
		speed_sum+=cur_speed;
		avg_speed=speed_sum/cnt;

		cout<<"当前经历了"<<cnt<<"帧"<<endl;
		cout<<"当前实时半径为"<<pi<<endl;
		cout<<"当前瞬时速度为"<<cur_speed<<","<<"当前平均速度为"<<avg_speed<<endl;
		cout<<"当前实时误差为"<<cur_error<<","<<"当前平均误差为"<<result<<endl;

		//以rgb的r为标准，当颜色为黑时，r的值必定小于80(10-30左右)
        if(a[b1]<80&&a[b2]>80&&a[b3]>80&&a[b4]>80){//若只有最左边像素点为黑色，则小车需要右转弯使得轨道黑线向中间靠拢
			for(i=0;i<=3;i++){
                speed[i]=speed_rightCircle[i];//速度方向为右转
			}
        }
        else if(a[b1]>80&&a[b2]<80&&a[b3]<80&&a[b4]>80){//若中间两个像素判断点为黑色，则小车可选择继续直行
			for (i=0;i<=3;i++){
				speed[i]=speed_forward[i];
			}
        }
        else if(a[b1]>80&&a[b2]>80&&a[b3]>80&&a[b4]<80){//若只有最右边像素点为黑色，则小车需要左转弯使得轨道黑线向中间靠拢
			for (i=0;i<=3; i++){
                speed[i]=speed_leftCircle[i];
			}
        }
        else if(a[b1]>80&&a[b2]>80&&a[b3]>80&&a[b4]>80){//若四个判断像素点全为白色，则小车可选择继续直行
			for(i=0;i<=3;i++){
				speed[i]=speed_forward[i];
			}
        }

		//将速度赋值给电机
		for(i=0;i<=3;i++){
			motors[i]->setVelocity(speed[i]);
		}

	}
	return 0;
}

 


