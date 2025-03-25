
#include"../_h/Console.h"
#include"../_h/Control.h"
#include"../_h/ObtainError.h"
#include<chrono>
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<thread>
#include<cstdlib>


#define cnsl ConsoleSettings

#define SLEEP std::this_thread::sleep_for(std::chrono::seconds(2));

constexpr auto pM_width = 40;
constexpr auto pM_height = 9;

//显示器
constexpr char pageMain[pM_height][pM_width] = {
	"========================\0",
	"请选择：\0",
	"0.关闭电机\0",
	"1.开启电机\0",
	"2.修改电机速度\0",
	"3.查看当前电机速度\0",
	"4.修改电机旋转方向\0",
	"5.查看电机旋转方向\0",
	"========================\0"
};

Motor::Motor()
{
	motor.rot = ROT_UNKNOWN;
	motor.speed = 0;
	motor.state = MOTOR_CLOSE;
}

Motor::~Motor()
{
	//自动

}

Motor::PARAM Motor::getMotorParam()
{
	return motor;
}

void Motor::setMotorParam(PARAM m)
{
	motor = m;
}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

MotorController::MotorController()
{
}

void MotorController::doCtrl()
{
	int input = -1;//输入
	bool check = false;
	printer();
	//ctrl循环
	while (input != 0)
	{
		cnsl::setPos();
		cnsl::clear();
		cnsl::setPos();
		check = false;
		cmotor = getMotorParam();
		std::cin >> input;//VS的安全函数其他编辑器没有,所以这里cin与printf混用了

		if (input == 0) break;
		switch (input)
		{
		case 1:check = turnOn(); break;
		case 2:check = setSpeed(); break;
		case 3:seeSpeed(); break;
		case 4:check = setRot(); break;
		case 5:seeRot(); break;
		default:
			printf("\n你可能不能这么选择\n");
			SLEEP;//阻塞线程
			break;
		}
		if (check)
			setMotorParam(cmotor);
	}
	printf("\n退出程序\n");
}

void MotorController::printer()
{
	//printf("\n");
	for (int i = 0; i < pM_height; i++)
		printf("%s\n", pageMain[i]);
}

//打开电机
bool MotorController::turnOn()
{
	if (checkState())
	{
		printf("\n电机已经打开,不用再次打开\n");
		SLEEP;
		return false;
	}
	else
		cmotor.state = MOTOR_OPEN;
	return true;
}

//设置旋转速度
bool MotorController::setSpeed()
{
	if (!checkState())
	{
		printf("\n你的电机未打开,你不能进行操作\n");
		SLEEP;
		return false;
	}
	if (cmotor.rot == ROT_UNKNOWN)
	{
		printf("\n未预设电机旋转方向\n");
		SLEEP;
		return false;
	}
	else
	{
		printf("\n请输入电机的转速（0-255）：\n");
		char sk[3000] = { '\0' };
		/*for (char a = '\0'; a == '\n';) {
			std::cin >> a;
		}*/
		std::cin >> sk;
		bool check = true;//用语检测输入行为是否合法
		long long spd = 0;
		for (int i = 0; sk[i] != '\0'; i++)
		{
			if (sk[i] >= '0' && sk[i] <= '9') {
				spd = (spd * 10 + sk[i] - '0');
			}
			else
			{
				check = false;
				break;
			}
		}
		if (check == false || spd > 255)
		{
			printf("\n你的输入含有非法行为\n");
			SLEEP;
			return false;
		}
		else
		{
			cmotor.speed = spd;
			return true;
		}
	}
}

//设置旋转方向
bool MotorController::setRot()
{
	if (!checkState())
	{
		printf("\n你的电机未打开,你不能进行操作\n");
		SLEEP;
		return false;
	}
	if (cmotor.speed == 0)
		printf("\n请设置电机的旋转方向: 0.顺时针(预设)		1.逆时针(预设)\n");
	else
		printf("\n请设置电机的旋转方向: 0.顺时针		1.逆时针\n");
	int a = -1;
	std::cin >> a;
	switch (a)
	{
	case 0:cmotor.rot = ROT_CLOCKWISE; break;
	case 1:cmotor.rot = ROT_COUNTER; break;
	default:
		printf("\n你可能不能这么选择\n");
		SLEEP;//阻塞线程
		return false;
		break;
	}
	return true;
}

//查看速度
void MotorController::seeSpeed()
{
	if (!checkState())
	{
		printf("\n你的电机未打开,你不能进行操作\n");
		SLEEP;
		return;
	}
	printf("\n转速:%d\n", cmotor.speed);
	SLEEP;
}

//查看旋转方向
void MotorController::seeRot()
{
	if (!checkState())
	{
		printf("\n你的电机未打开,你不能进行操作\n");
		SLEEP;
		return;
	}
	if (cmotor.rot == ROT_UNKNOWN)
	{
		printf("\n当前未设置电机的旋转方向\n");
	}
	else if (cmotor.rot == ROT_CLOCKWISE)
	{
		if (cmotor.speed == 0)
			printf("\n电机的旋转方向:顺时针(预设)\n");
		else
			printf("\n电机的旋转方向:顺时针\n");
	}
	else if (cmotor.rot == ROT_COUNTER)
	{
		if (cmotor.speed == 0)
			printf("\n电机的旋转方向:逆时针(预设)\n");
		else
			printf("\n电机的旋转方向:逆时针\n");
	}
	SLEEP;
}

//检测电机是否开启
//@return true是
bool MotorController::checkState()
{
	if (cmotor.state == MOTOR_CLOSE)
		return false;
	else
		return true;
}

