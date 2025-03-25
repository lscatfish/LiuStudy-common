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
		ROT_CLOCKWISE,//顺时针
		ROT_COUNTER//逆时针
	}RotationDirection;//电机旋转方向

	typedef enum {
		MOTOR_CLOSE = 0u,//关闭
		MOTOR_OPEN//打开
	}MotorState;//电机状态

	typedef struct {
		MotorState state;//电机状态
		RotationDirection rot;//电机旋转方向
		int speed;//转速
	}PARAM;//电机参数

	//开放接口
	//以PARAM的数据类型
	PARAM getMotorParam();
	void setMotorParam(PARAM);

private:
	PARAM motor;
};


class MotorController :public Motor {
public:
	MotorController();
	void doCtrl();//进入对象的控制程序
private:
	PARAM cmotor;//motor的复制

	//放弃对win的API的支持
	//void print_WIN();//管理windows的打印器
	//void print_UBUN();//管理Linux的打印器

	void printer();//打印器

	bool turnOn();//打开电机
	bool setSpeed();//设置旋转速度
	bool setRot();//设置旋转方向
	void seeSpeed();//查看速度
	void seeRot();//查看旋转方向
	//检测电机是否开启
	//@return true是
	bool checkState();
};




#endif // !CONTROL_H