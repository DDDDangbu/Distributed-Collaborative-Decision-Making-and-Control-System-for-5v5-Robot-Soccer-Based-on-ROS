#ifndef _NUBOT_ACTIVEROLE_H
#define _NUBOT_ACTIVEROLE_H

#include <nubot_control/plan.h>
#include <nubot_common/ObstaclesInfo3d.h>
#include <boost/foreach.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/filesystem.hpp>
#include <boost/asio.hpp>

#define DRIBBLE_TIME 30
namespace nubot {

class ActiveRole
{
public:
      ActiveRole();
     ~ActiveRole();
void process();
bool checkPass();
void clearActiveState();
void selectCurrentState();
void selectCurrentAction(unsigned char state);
bool evaluateKick();
bool evaluatePass1();
bool evaluatePass2();
bool passisBetter1();
bool passisBetter2();
void caculatePassEnergy(double & energy, int & label);
void caculateDribblingEnergy(double & avoid_enegy,bool isNullFrontRobot);
void selectDribblingOrPassing(bool isNullFrontRobot);
void findBall();
void turn4Shoot();
void turn2Pass();
void NewAvoidObs();
void activeCatchBall();
void triggerShoot(double delta_force=0.0);
void stuckProcess();
void kickball4Coop(DPoint target);
void setAssistPt(DPoint &assist);
double calculateKickforce(DPoint target, int kick_mode);
double calculateShootingWilling(DPoint robot_pose);
bool evaluatePass();
bool passisBetter();
//! Kinect检测障碍物
void Obsnearest(nubot_common::ObstaclesInfo3d &obs, DPoint &gltarget);

public:
    World_Model_Info * world_model_;
    nubot_common::ObstaclesInfo3d Obstacles3d;
    Plan  * m_plan_;
    bool  stuckflg_;             /** 机器人当前是否堵转*/
    int   stucktime_;            /** 机器人堵转周期记录*/
    bool  NeedEvaluat ;          /** 射门的ROS服务，于底层控制节点通信*/
    int   dynamic_shoot_count_;  /** 动态射门的*/
    int   quick_shoot_count_;
    int   pass_lock_;
    unsigned char currentstate_;
    bool   kick_enable_;         /** 准备踢球*/
    bool   dribble_enable_;      /** 带球状态*/
    bool   isturn_;
    float  kick_force_;          /** 踢球的力量*/

private:
    DPoint  assist_pt_;                 /** 从assistrole类得到的 **/
    FieldInformation field_;

    double P1;                 /** set coefficient for shoot ploynomial*/
    double P2;
    double P3;
    double P4;
    double P5;
    double P6;
    double P7;

    double Pass_in_3m;
    double Pass_3m_5m;
    double Pass_far_5m;
    double Gain_power_pass;
    double Gain_power_shoot;
};

}


#endif // _NUBOT_ACTIVEROLE_H
