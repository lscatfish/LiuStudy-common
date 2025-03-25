
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

//��ʾ��
constexpr char pageMain[pM_height][pM_width] = {
	"========================\0",
	"��ѡ��\0",
	"0.�رյ��\0",
	"1.�������\0",
	"2.�޸ĵ���ٶ�\0",
	"3.�鿴��ǰ����ٶ�\0",
	"4.�޸ĵ����ת����\0",
	"5.�鿴�����ת����\0",
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
	//�Զ�

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
	int input = -1;//����
	bool check = false;
	printer();
	//ctrlѭ��
	while (input != 0)
	{
		cnsl::setPos();
		cnsl::clear();
		cnsl::setPos();
		check = false;
		cmotor = getMotorParam();
		std::cin >> input;//VS�İ�ȫ���������༭��û��,��������cin��printf������

		if (input == 0) break;
		switch (input)
		{
		case 1:check = turnOn(); break;
		case 2:check = setSpeed(); break;
		case 3:seeSpeed(); break;
		case 4:check = setRot(); break;
		case 5:seeRot(); break;
		default:
			printf("\n����ܲ�����ôѡ��\n");
			SLEEP;//�����߳�
			break;
		}
		if (check)
			setMotorParam(cmotor);
	}
	printf("\n�˳�����\n");
}

void MotorController::printer()
{
	//printf("\n");
	for (int i = 0; i < pM_height; i++)
		printf("%s\n", pageMain[i]);
}

//�򿪵��
bool MotorController::turnOn()
{
	if (checkState())
	{
		printf("\n����Ѿ���,�����ٴδ�\n");
		SLEEP;
		return false;
	}
	else
		cmotor.state = MOTOR_OPEN;
	return true;
}

//������ת�ٶ�
bool MotorController::setSpeed()
{
	if (!checkState())
	{
		printf("\n��ĵ��δ��,�㲻�ܽ��в���\n");
		SLEEP;
		return false;
	}
	if (cmotor.rot == ROT_UNKNOWN)
	{
		printf("\nδԤ������ת����\n");
		SLEEP;
		return false;
	}
	else
	{
		printf("\n����������ת�٣�0-255����\n");
		char sk[3000] = { '\0' };
		/*for (char a = '\0'; a == '\n';) {
			std::cin >> a;
		}*/
		std::cin >> sk;
		bool check = true;//������������Ϊ�Ƿ�Ϸ�
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
			printf("\n������뺬�зǷ���Ϊ\n");
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

//������ת����
bool MotorController::setRot()
{
	if (!checkState())
	{
		printf("\n��ĵ��δ��,�㲻�ܽ��в���\n");
		SLEEP;
		return false;
	}
	if (cmotor.speed == 0)
		printf("\n�����õ������ת����: 0.˳ʱ��(Ԥ��)		1.��ʱ��(Ԥ��)\n");
	else
		printf("\n�����õ������ת����: 0.˳ʱ��		1.��ʱ��\n");
	int a = -1;
	std::cin >> a;
	switch (a)
	{
	case 0:cmotor.rot = ROT_CLOCKWISE; break;
	case 1:cmotor.rot = ROT_COUNTER; break;
	default:
		printf("\n����ܲ�����ôѡ��\n");
		SLEEP;//�����߳�
		return false;
		break;
	}
	return true;
}

//�鿴�ٶ�
void MotorController::seeSpeed()
{
	if (!checkState())
	{
		printf("\n��ĵ��δ��,�㲻�ܽ��в���\n");
		SLEEP;
		return;
	}
	printf("\nת��:%d\n", cmotor.speed);
	SLEEP;
}

//�鿴��ת����
void MotorController::seeRot()
{
	if (!checkState())
	{
		printf("\n��ĵ��δ��,�㲻�ܽ��в���\n");
		SLEEP;
		return;
	}
	if (cmotor.rot == ROT_UNKNOWN)
	{
		printf("\n��ǰδ���õ������ת����\n");
	}
	else if (cmotor.rot == ROT_CLOCKWISE)
	{
		if (cmotor.speed == 0)
			printf("\n�������ת����:˳ʱ��(Ԥ��)\n");
		else
			printf("\n�������ת����:˳ʱ��\n");
	}
	else if (cmotor.rot == ROT_COUNTER)
	{
		if (cmotor.speed == 0)
			printf("\n�������ת����:��ʱ��(Ԥ��)\n");
		else
			printf("\n�������ת����:��ʱ��\n");
	}
	SLEEP;
}

//������Ƿ���
//@return true��
bool MotorController::checkState()
{
	if (cmotor.state == MOTOR_CLOSE)
		return false;
	else
		return true;
}

