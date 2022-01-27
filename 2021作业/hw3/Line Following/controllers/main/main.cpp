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
	Camera *camera = robot->getCamera("camera");
	camera->enable(1);
	keyboard.enable(1);//运行键盘输入设置频率是1ms读取一次
    GPS* gps;
	gps = robot->getGPS("gps");
	gps->enable(1);

	double speed[4];
	double velocity=10;
    int i;

    //初始化
    for(i=0;i<=3;i++){
		motors[i]=robot->getMotor(wheels_names[i]);//按照你在仿真器里面设置的名字获取句柄
		motors[i]->setPosition(std::numeric_limits<double>::infinity());
		motors[i]->setVelocity(0.0);//设置电机一开始处于停止状态
		speed[i]=0;   
    }

	double speed_forward[4]={velocity,velocity,velocity,velocity};
	double speed_leftCircle[4]={velocity,-velocity,-velocity,velocity};
	double speed_rightCircle[4]={-velocity,velocity,velocity,-velocity};



	int timeStep=(int)robot->getBasicTimeStep();//获取你在webots设置一帧的时间
	//cout<<timeStep<<endl;
	int cnt=0;
	double dr=0;
	double result=0;
	while(robot->step(timeStep)!=-1){//仿真运行一帧
    	const unsigned char *a=camera->getImage();//读取相机抓取的最后一张图像。图像被编码为三个字节的序列，分别代表像素的红、绿
		int length=camera->getWidth();
		int width=camera->getHeight();
		cout<<length<<' '<<width<<endl;
        int b1,b2,b3,b4;
        b1=64*3*32;
        b2=64*3*32+35*3;
        b3=64*3*32+40*3;
        b4=64*3*32+63*3;
        cout<<int(a[64*3*32])<<' '<<int(a[64*3*32+1])<<' '<<int(a[64*3*32+2])<<endl;
		cout<<int(a[64*3*32+35*3])<<' '<<int(a[64*3*32+35*3+1])<<' '<<int(a[64*3*32+35*3+2])<<endl;
        cout<<int(a[64*3*32+40*3])<<' '<<int(a[64*3*32+40*3+1])<<' '<<int(a[64*3*32+40*3+2])<<endl;
        cout<<int(a[64*3*32+63*3])<<' '<<int(a[64*3*32+63*3+1])<<' '<<int(a[64*3*32+63*3+2])<<endl;

		double x=0,y=1.22;
		cnt++;
		double dx,dy;
		dx=gps->getValues()[0];
		dy=gps->getValues()[1];
		double r;
		r=abs(sqrt((abs(dx-x))*(abs(dx-x))+(abs(dy-y))*(abs(dy-y)))-1.185);
		dr+=r;
		result=dr/cnt;
		cout<<cnt<<' '<<r<<' '<<result<<endl;
        if(a[b1]<80&&a[b2]>80&&a[b3]>80&&a[b4]>80){
			for(i=0;i<=3;i++){
                speed[i]=speed_rightCircle[i];
			}
        }
        else if(a[b1]>80&&a[b2]<80&&a[b3]<80&&a[b4]>80){
			for (i=0;i<=3;i++){
				speed[i]=speed_forward[i];
			}
        }
        else if(a[b1]>80&&a[b2]>80&&a[b3]>80&&a[b4]<80){
			for (i=0;i<=3; i++){
                speed[i]=speed_leftCircle[i];
			}
        }
        else if(a[b1]>80&&a[b2]>80&&a[b3]>80&&a[b4]>80){
			for(i=0;i<=3;i++){
				speed[i]=speed_forward[i];
			}
        }
		//让电机执行
		for(i=0;i<=3;i++){
			motors[i]->setVelocity(speed[i]);
		}
	}
    
	return 0;
}

 


