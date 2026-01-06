#ifndef __NUBOT_STRATEGY_ROLE_ASSIGNMENT_H_
#define __NUBOT_STRATEGY_ROLE_ASSIGNMENT_H_

#include <core.hpp>
#include <algorithm>
#include <nubot_control/plan.h>
#include <nubot_control/world_model_info.h>
#include <ros/ros.h>
namespace nubot{

/** 效能置计算的一些参数*/
const double ACTIVE_K = -300.0/MAXDIS_FIELD;//-200.0/MAXDIS_FIELD;                  /** 主攻参数：与足球距离的计算参数*/
const double ACTIVE_B = 200;
const double ACTIVE_THETA_K = -20.0/SINGLEPI_CONSTANT;        /** 主攻参数：与足球夹角的计算参数*/
const double ACTIVE_THETA_B=20;
const double ACTIVE_BALL_VEL = 0.1;       // 主攻效能值中足球速度系数
const double ACTIVE_MAX      = 281;       // 主攻效能值最大值

const double PASSIVE_K = -200.0/MAXDIS_FIELD;                 /** 防守参数：与防守点距离的计算参数*/
const double PASSIVE_B = 200;
const double PASSIVE_THETA_K = -10/SINGLEPI_CONSTANT;         /** 防守参数：与防守点夹角的计算参数*/
const double PASSIVE_THETA_B = 20;

const double ASSISTANT_K = -220.0/MAXDIS_FIELD;
const double ASSISTANT_B = 220;


const double ASSISTANTLEFT_K = -230.0/MAXDIS_FIELD;
const double ASSISTANTLEFT_B = 230;

const double ASSISTANTRIGHT_K = -230.0/MAXDIS_FIELD;
const double ASSISTANTRIGHT_B = 230;

const double PUREACTIVE_C=200;
const double PUREACTIVE_K=-200.0/MAXDIS_FIELD;
const double PUREACTIVE_THETA_C=20;
const double PUREACTIVE_THETA_K=-20/SINGLEPI_CONSTANT;

const double DISOFDEFENSE1 = FIELD_LENGTH /4.0;
const double block_dist = 50;       // distance threshold of opp blocking ball

class RoleAssignment
{

public:
    RoleAssignment();
    RoleAssignment(World_Model_Info & _world_model);
    ~RoleAssignment();
    World_Model_Info * world_model_;
    float RoleUtilityMatrix_[OUR_TEAM-1][ROLENUM];
    ros::Time lastRoleChangeTime_;
    ros::Time currentRoleTime_;
    /** 记录上一次的主攻ID and Utility **/
    int last_activeID;
    float last_activeUtility;

public:
    void calculateRoleUtility();
    void adjustRole();
    void selectRole();
    void fixRole();
    bool isoppBlockWay(DPoint robot_pos, DPoint ball_pos);     // does opponent block my way?
    int  process();
    /** 融合球的位置,取离球最近的机器人得到的足球信息.修改自staticpass.cpp中的fuseball_() */
    void setPts(DPoint &assist, DPoint &midfield, DPoint &passive);

    Plan *m_plan_;

private:
    /** 以下的三个DPoint是分别从activerole, assistrole和midfieldrole类得到的 **/
    DPoint  assist_pt_;
    DPoint  midfield_pt_;
    DPoint  passive_pt_;
};

}

#endif//!__NUBOT_STRATEGY_ROLE_ASSIGNMENT_H_
