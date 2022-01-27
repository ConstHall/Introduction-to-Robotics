
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <iostream> 

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>

using namespace std;
using namespace webots;



int main() {
	Motor *motors[4];//电机和键盘都要用webots给的类型
	webots::Keyboard keyboard;
	char wheels_names[4][8] = { "motor1","motor2","motor3","motor4" };//你在仿真器里面设置的名字

	Robot *robot = new Robot();//使用webots的机器人主体
	keyboard.enable(1);//运行键盘输入设置频率是1ms读取一次

	double speed1[4];
	double speed2[4];
	double velocity = 10;

	//初始化
	for (int i = 0; i < 4; i++)
	{
		motors[i] = robot->getMotor(wheels_names[i]);//按照你在仿真器里面设置的名字获取句柄
		motors[i]->setPosition(std::numeric_limits<double>::infinity());
		motors[i]->setVelocity(0.0);

		speed1[i] = 0;
		speed2[i] = 0;
	}




	//我列了一个小表格，当四个轮子按照下面这样转的时候，车子可以完成前后左右，转圈
	//斜着走是前后+左右  两个键同时按
	double speed_forward[4] = { velocity ,velocity ,velocity ,velocity };
	double speed_backward[4] = { -velocity ,-velocity ,-velocity ,-velocity };
	double speed_leftward[4] = { velocity ,-velocity ,velocity ,-velocity };
	double speed_rightward[4] = { -velocity ,velocity ,-velocity ,velocity };

	double speed_leftCircle[4] = { velocity ,-velocity ,-velocity ,velocity };
	double speed_rightCircle[4] = { -velocity ,velocity ,velocity ,-velocity };


	int timeStep = (int)robot->getBasicTimeStep();//获取你在webots设置一帧的时间
	cout << timeStep << endl;

	while (robot->step(timeStep) != -1) //仿真运行一帧
	{



		//获取键盘输入，这样写可以获得同时按下的按键（最多支持7个）
		int keyValue1 = keyboard.getKey();
		int keyValue2 = keyboard.getKey();
		cout << keyValue1 << ":" << keyValue2 << endl;

		//根据按键决定电机怎么样转动
		if (keyValue1 == 'W')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_forward[i];
			}
		}
		else if (keyValue1 == 'S')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_backward[i];
			}
		}
		else if (keyValue1 == 'A')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_leftward[i];
			}
		}
		else if (keyValue1 == 'D')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_rightward[i];
			}
		}
		else if (keyValue1 == 'Q')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_leftCircle[i];
			}
		}
		else if (keyValue1 == 'E')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_rightCircle[i];
			}
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = 0;
			}
		}




		if (keyValue2 == 'W')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_forward[i];
			}
		}
		else if (keyValue2 == 'S')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_backward[i];
			}
		}
		else if (keyValue2 == 'A')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_leftward[i];
			}
		}
		else if (keyValue2 == 'D')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_rightward[i];
			}
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = 0;
			}
		}


		//让电机执行
		for (int i = 0; i < 4; i++)
		{
			motors[i]->setVelocity(speed1[i] + speed2[i]);
		}

		//wb_motor_set_velocity(wheels[0],right_speed);
	}


	return 0;
}
