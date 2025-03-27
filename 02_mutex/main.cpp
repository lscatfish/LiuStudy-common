#include <stdlib.h>

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

int rnumberH = 0;             //氢的个数
int rnumberC = 0;             //碳的个数
std::mutex mtxH;              //氢的互斥锁
std::mutex mtxC;              //碳的互斥锁
std::condition_variable cvH;  //氢的条件变量
std::condition_variable cvC;  //碳的条件变量

//输入函数
int input() {
    std::cout << "请输入：";
    char in[666] = {'\0'};
    std::cin >> in;

    int sumC = 0;  //碳的个数
    int sumH = 0;  //氢的个数
    //检查输入
    for (int i = 0; in[i] != '\0'; i++) {
        if (in[i] != 'H' && in[i] != 'C') {
            std::cout << "输入错误！为你退出程序" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            exit(12);
        } else if (in[i] == 'H')
            sumH++;
        else if (in[i] == 'C')
            sumC++;
    }
    if (sumC * 3 != sumH) {  //不能组成甲烷
        std::cout << "输入错误！为你退出程序。" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        exit(12);
    }
    return sumC;
}

//检查生成氢的个数
//@return bool氢的个数是否小于3
bool checkNumner_H() {
    if (rnumberH < 3)
        return true;
    else
        return false;
}

//检查生成碳的个数
//@return bool碳的个数是否小于1
bool checkNumner_C() {
    if (rnumberC < 1)
        return true;
    else
        return false;
}

//生成氢的函数
void releaseHydrogen(int num) {
    for (int i = 0; i < num; i++) {
        std::unique_lock<std::mutex> lck(mtxH);  //上锁
        cvH.wait(lck, checkNumner_H);            //等待氢的个数小于4
        std::cout << "H";
        rnumberH++;
    }
}

//生成碳的函数
void releaseCarbon(int num) {
    for (int i = 0; i < num; i++) {
        std::unique_lock<std::mutex> lck(mtxC);  //上锁
        cvC.wait(lck, checkNumner_C);            //等待碳的个数小于1
        std::cout << "C";
        rnumberC++;
    }
}

//控制生成甲烷的函数
void barrier(int GoalMethaneNumber) {
    std::thread th_H(releaseHydrogen, GoalMethaneNumber * 3);  //生成氢的线程
    std::thread th_C(releaseCarbon, GoalMethaneNumber);        //生成碳的线程

    for (int i = 0; i < GoalMethaneNumber; i++) {
        while (checkNumner_C() == true || checkNumner_H() == true) {
            std::this_thread::yield();  //让出CPU
                                        // std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::unique_lock<std::mutex> lckH(mtxH);
        std::unique_lock<std::mutex> lckC(mtxC);
        rnumberH = 0;
        rnumberC = 0;
        std::cout << std::endl;
        cvH.notify_all();
        cvC.notify_all();
    }
    th_C.join();
    th_H.join();
}

int main() {
    int GoalMethaneNumber = input();
    barrier(GoalMethaneNumber);
    return 0;
}