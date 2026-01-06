#ifndef _NUBOT_WORLD_MODEL_INFO_H
#define _NUBOT_WORLD_MODEL_INFO_H

#include "world_model/teammatesinfo.h"
#include "nubot_control/dribblestate.hpp"
#include "Fieldinformation.h"

#define CENTRALIZE_POSITION
#define DISTANCE_OPP2BALL    36
#define TIME_DRIBBLE         10
#define TIME_LOSE            40

namespace nubot
{
const int PASSSTATE_TIME_DELAY = 200;
enum{ KickBySelf = 0, KickByOpp =1, NoKick=2};
const double KickBySelfTime = 10;

class PassState
{
public:
    bool is_passing_;
    int  time_lock_;
    int  pass_id_;
    int  catch_id_;
    DPoint pass_pt_;
    DPoint catch_pt_;
    bool is_dynamic_pass_;
    bool is_static_pass_;
    PassState():is_passing_(false){
        reset();
    }
    void set(const int & pass_id, const int & catch_id,
             const DPoint & pass_pt, const DPoint & catch_pt,
             const bool & _dynamic_pass,const bool &_static_pass)
    {
        is_passing_   = true;
        is_dynamic_pass_ = _dynamic_pass;
        is_static_pass_  = _static_pass;
        pass_id_      = pass_id;
        catch_id_     = catch_id;
        pass_pt_      = pass_pt;
        catch_pt_     = catch_pt;
        time_lock_    = PASSSTATE_TIME_DELAY;
    }
    void
    reset(){
        is_passing_      = false;
        is_dynamic_pass_ = false;
        is_static_pass_  = false;
        pass_id_ = -1;
        catch_id_ =-1;
        time_lock_ = 0;
    }
    void update()//ensure this update() is called in every callback         // WHAT?
    {
        if(time_lock_ > 0 )
            time_lock_--;
        else
            reset();
    }
};

class World_Model_Info
{
public:
    std::vector<nubot::Robot>      RobotInfo_;
    std::vector<nubot::BallObject> BallInfo_;
    std::vector<nubot::DPoint>     Obstacles_;
    std::vector<nubot::DPoint>     Opponents_;
    PassCommands      pass_cmds_;                // 接收到的队友机器人发球的传球命令
    MessageFromCoach  CoachInfo_;                // 接收到的COACH信息
    DribbleState      DribbleState_;             // 判断带球的状态；
    FieldInformation  field_info_;               // 场地信息

    int BallInfoState_;                          // 当前看到足球的状态自己、队友或者没有看到足球
    DPoint lastBallPosition_;                    // 上一帧足球所在的位置
    int  NoSeeBallNums_;                         // 连续几帧没有看到足球
    int  AgentID_;                               // 机器人自身的ID
    int  CurActiveRobotNums_;                    // 当前活跃的机器人数目
    int  PreActiveRobotNums_;                    // 上一帧活跃的机器人数目
    int  CurActiveNotGoalieNums_;                // 当前除去守门员活跃的机器人书目
    int  PreActiveNotGoalieNums_;                // 上一帧除去守门员活跃的机器人书目
    bool RegainBall_;                            // 3.7开始球由对方控制，重新得到球后需要完成一次传球；
    bool IsMoveForStartCommand_;
    bool IsOurDribble_;
    bool IsOppHoldball_;                         // 是否对方拥有球权
    bool lastDribble;                            // 判断是否为本方球权

    /// 传球有关的状态
    PassState pass_state_;        // 接球或者是踢球，主要用于锁定状态，接球通过通信锁定
    DPoint catch_pt_;             // 接球机器人位置
    DPoint pass_pt_;              // 传球机器人位置
    int    catch_ID_;             // 接球机器人ID
    int    pass_ID_;              // 传球机器人ID
    double kick_force_;           // 踢球的力量
    bool   is_dynamic_pass_;      // 准备动态传球
    bool   is_static_pass_;       // 准备静态传球
    bool   is_passed_out_;        // 足球已经踢出
    int    KickSelection_;        // 对方发球还是我方发球
    double static_pass_time_;
    double static_ball_dis_;
    BallObject     fuseBallInfo_; // 由fuseBall()得到的融合后的足球信息
    vector<DPoint> fuseOpps_;     // 融合后的对手位置

public:

    World_Model_Info();
    /// 计算场地上活跃的机器人数目
    void caculateActiveRobots();
    /// 函数的功能给是判断机器人是否带上足球
    void checkDribble(const bool & ball_holding);
    /// 检测机器人是否能够移动，即发球时候的状态
    void isMoveForStartCommand();
    /// 入口函数，首先调用，更新世界信息
    void update(const bool & ball_holding);
    /// 清空传球信息 */
    void clearPassState(bool isclearId);
    /// 距离球最近的机器人看到的足球位置为最后的位置, 改自staticpass.cpp
    bool fuseBall();
    /// 障碍物的初步处理,改自staticpass.cpp
    bool fuseOpp();
    /// 针对新规则，增加判断球权是否发生转移，即下一次触球算不算重新得到球
    void checkOppHoldBall();
};

}

#endif /** _NUBOT_WORLD_MODEL_INFO_H*/
