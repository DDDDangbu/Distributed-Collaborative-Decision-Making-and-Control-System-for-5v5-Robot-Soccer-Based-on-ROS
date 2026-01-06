#ifndef _NUBOT_BEHAVIOUR_H
#define _NUBOT_BEHAVIOUR_H

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string.h>
#include "common.hpp"

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6

using namespace std;

namespace nubot{

/** Behaviour主要是底层的控制函数，控制机器人按照预定的轨迹运动*/

class Behaviour
{

public:
    Behaviour();
    ~ Behaviour();

    void fuzzyPIDcontrol(float &deltakp, float &deltaki,float &deltakd, float err,float err1);

    /// 设置目标点，最大速度，平移动作，目标速度
    void setTarget(DPoint target, float maxvel, char move_action, DPoint target_vel=DPoint(0,0));
    /// 设置目标角度，最大角速度，旋转动作 (rotate_mode代表旋转方式，0：以最小角度选择，1：顺时针旋转，-1：逆时针旋转）
    void setOrienation(float target_ori, float maxw, char rotate_action, int rotate_mode=0,double tar_w =0.0);
    void clear();

public:
    char move_action_;                   //平移动作
    char rotate_action_;                 //旋转动作
    DPoint2s target_;                    //目标点位置
    DPoint2s target_vel_;                //目标点速度
    float maxvel_;                       //最大速度限制
    float target_ori_;                   //目标点角度
    float target_w_;                     //target omega
    float maxw_;                         //最大角速度限制
    int   rotate_mode_;                  //旋转方式
 };
}

#endif // _NUBOT_BEHAVIOUR_H
