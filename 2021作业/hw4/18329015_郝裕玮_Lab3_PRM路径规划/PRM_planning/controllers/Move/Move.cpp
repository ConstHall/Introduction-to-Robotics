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

	double speed[4];//此数组会在后面赋值给电机以速度
	double velocity=15;//初速度
    int i;

    //初始化
    for(i=0;i<=3;i++){
		motors[i]=robot->getMotor(wheels_names[i]);//按照你在仿真器里面设置的名字获取句柄
		motors[i]->setPosition(std::numeric_limits<double>::infinity());
		motors[i]->setVelocity(0.0);//设置电机一开始处于停止状态
		speed[i]=0;//给予小车一个初速度   
    }

	double speed_forward[4]={velocity,velocity,velocity,velocity};//前进方向
	double speed_leftCircle[4]={velocity,-velocity,-velocity,velocity};//左自旋(即左转弯)
	double speed_rightCircle[4]={-velocity,velocity,velocity,-velocity};//右自旋(即右转弯)

	int timeStep=(int)robot->getBasicTimeStep();//获取你在webots设置一帧的时间

	while(robot->step(timeStep)!=-1){//仿真运行一帧
    	const unsigned char *a=camera->getImage();//读取相机抓取的最后一张图像。图像被编码为三个字节的序列，分别代表像素的红、绿、蓝

		int length=camera->getWidth();//图像长度
		int width=camera->getHeight();//图像宽度

        int b1,b2,b3,b4;
        b1=length*3*width/2;//图像中间一行的最左边的像素点
        b2=length*3*width/2+(width/2+3)*3;//图像中间一行的中间靠左的某像素点
        b3=length*3*width/2+(width/2+5)*3;//图像中间一行的中间靠右的某像素点
        b4=length*3*width/2+(length-1)*3;//图像中间一行的最右边的某像素点
        //其中b1，b2代表图像中间的黑色轨迹的两侧(大致估计)

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

 


