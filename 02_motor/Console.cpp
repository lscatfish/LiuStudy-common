
#include "Console.h"

#include <iostream>

namespace ConsoleSettings {

//指定输出坐标
// x：宽(120)		y：高(30)
//默认值
void setPos() { printf("\033[10;0H"); }

void clear() {
    for (int j = 0; j < 5; j++)
        for (int i = 0; i < 50; i++) printf("  ");
}

}  // namespace ConsoleSettings
