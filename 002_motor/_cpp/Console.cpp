
#include"../_h/Console.h"
#include<iostream>

namespace ConsoleSettings
{

	//ָ���������
	//x����(120)		y����(30)
	//Ĭ��ֵ
	void setPos()
	{
		printf("\033[10;0H");
	}

	void clear()
	{
		for (int j = 0; j < 5; j++)
			for (int i = 0; i < 50; i++)
				printf("  ");
	}

}




