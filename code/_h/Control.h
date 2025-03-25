#pragma once

#ifndef CONTROL_H
#define CONTROL_H


class Motor
{
public:
	Motor();
	~Motor();

	typedef enum {
		ROT_UNKNOWN = 0u,
		ROT_CLOCKWISE,//˳ʱ��
		ROT_COUNTER//��ʱ��
	}RotationDirection;//�����ת����

	typedef enum {
		MOTOR_CLOSE = 0u,//�ر�
		MOTOR_OPEN//��
	}MotorState;//���״̬

	typedef struct {
		MotorState state;//���״̬
		RotationDirection rot;//�����ת����
		int speed;//ת��
	}PARAM;//�������

	//���Žӿ�
	//��PARAM����������
	PARAM getMotorParam();
	void setMotorParam(PARAM);

private:
	PARAM motor;
};


class MotorController :public Motor {
public:
	MotorController();
	void doCtrl();//�������Ŀ��Ƴ���
private:
	PARAM cmotor;//motor�ĸ���

	//������win��API��֧��
	//void print_WIN();//����windows�Ĵ�ӡ��
	//void print_UBUN();//����Linux�Ĵ�ӡ��

	void printer();//��ӡ��

	bool turnOn();//�򿪵��
	bool setSpeed();//������ת�ٶ�
	bool setRot();//������ת����
	void seeSpeed();//�鿴�ٶ�
	void seeRot();//�鿴��ת����
	//������Ƿ���
	//@return true��
	bool checkState();
};




#endif // !CONTROL_H