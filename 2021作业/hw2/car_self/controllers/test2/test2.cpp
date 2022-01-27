#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
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


	//W:向前  S:向后  A:向左  D:向右  1:右前  2:右后  3:左前  4:左后  Q:左自旋  E:右自旋
	double speed_forward[4]={velocity,velocity,velocity,velocity};
	double speed_backward[4]={-velocity,-velocity,-velocity,-velocity};
	double speed_leftward[4]={velocity,-velocity,velocity,-velocity};
	double speed_rightward[4]={-velocity,velocity,-velocity,velocity};
	double speed_leftCircle[4]={velocity,-velocity,-velocity,velocity};
	double speed_rightCircle[4]={-velocity,velocity,velocity,-velocity};
	double speed_rightForward[4]={0,velocity,0,velocity};
	double speed_rightBackward[4]={-velocity,0,-velocity,0};
	double speed_leftForward[4]={velocity,0,velocity,0};
	double speed_leftBackward[4]={0,-velocity,0,-velocity};



	int timeStep=(int)robot->getBasicTimeStep();//获取你在webots设置一帧的时间
	//cout<<timeStep<<endl;

	while(robot->step(timeStep)!=-1){//仿真运行一帧
    	const unsigned char *a=camera->getImage();//读取相机抓取的最后一张图像。图像被编码为三个字节的序列，分别代表像素的红、绿
		int length=camera->getWidth();
		int width=camera->getHeight();
		cout<<length<<cout<<width<<endl;
		//cout<<int(a[9])<<' '<<int(a[10])<<' '<<int(a[11])<<endl;

		//获取键盘输入，这样写可以获得同时按下的按键（最多支持7个）
		int keyValue1=keyboard.getKey();

		//根据按键决定电机怎么样转动
		if(keyValue1=='W'){
			for(i=0;i<=3;i++){
				speed[i]=speed_forward[i];
			}
		}
		else if(keyValue1=='S'){
			for(i=0;i<=3;i++){
				speed[i]=speed_backward[i];
			}
		}
		else if(keyValue1=='A'){
			for(i=0;i<=3;i++){
				speed[i]=speed_leftward[i];
			}
		}
		else if(keyValue1=='D'){
			for(i=0;i<=3;i++){
				speed[i]=speed_rightward[i];
			}
		}
		else if(keyValue1=='Q'){
			for (i=0;i<=3; i++){
				speed[i]=speed_leftCircle[i];
			}
		}
		else if(keyValue1=='E'){
			for (i=0;i<=3;i++){
				speed[i]=speed_rightCircle[i];
			}
		}
		else if(keyValue1=='1'){
			for (i=0;i<=3; i++){
				speed[i]=speed_rightForward[i];
			}
		}
		else if(keyValue1=='2'){
			for(i=0;i<=3;i++){
				speed[i]=speed_rightBackward[i];
			}
		}
		else if(keyValue1=='3'){
			for(i=0;i<=3;i++){
				speed[i]=speed_leftForward[i];
			}
		}
		else if(keyValue1=='4'){
			for(i=0;i<=3;i++){
				speed[i]=speed_leftBackward[i];
			}
		}
		else{
			for(i=0;i<=3;i++){
				speed[i]=0;
			}
		}

		//让电机执行
		for(i=0;i<=3;i++){
			motors[i]->setVelocity(speed[i]);
		}
	}
    
	return 0;
}
