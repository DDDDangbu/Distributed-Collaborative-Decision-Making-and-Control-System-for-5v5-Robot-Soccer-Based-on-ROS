#ifndef _NUBOT_CONTROL_H
#define _NUBOT_CONTROL_H

#include <nubot_common/ActionCmd.h>
#include <nubot_common/VelCmd.h>
#include <nubot_common/BallIsHolding.h>
#include <nubot_common/WorldModelInfo.h>
#include <nubot_common/BallInfo3d.h>
#include <nubot_common/StrategyInfo.h>
#include <nubot_common/TargetInfo.h>
#include <nubot_common/Relocation.h>

#include <nubot_control/nubotcontrolConfig.h>
#include <nubot_control/world_model_info.h>
//#include <nubot_control/DebugInfo.h>
#include <nubot_control/strategy.hpp>
#include <nubot_control/common.hpp>
#include <nubot_control/plan.h>
#include <nubot_control/staticpass.h>
#include <nubot_control/test.h>

#include <dynamic_reconfigure/server.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <ros/ros.h>
#include <Fieldinformation.h>
#include "std_msgs/Bool.h"        /////////@hbx

using namespace std;
namespace nubot{

class NuBotControl
{

public:
    ros::Subscriber  goalie_ball3d_subL_;
    ros::Subscriber  goalie_ball3d_subR_;
    ros::Subscriber  worldmodelinfo_sub_;
    ros::Subscriber  odoinfo_sub_;
    ros::Subscriber  obstaclesinfo_sub_;
    ros::Subscriber  obsinfo3d_sub_;
    ros::Subscriber  ballHolding_sub_;
    ros::Subscriber  isOpposite_;   ///////@hbx

    ros::Publisher   action_cmd_pub_;
    ros::Publisher   strategy_info_pub_;
    ros::Publisher   isRelocation_pub_; //@lx

    ros::Timer       control_timer_;
    boost::shared_ptr<ros::NodeHandle> nh_;

    /// 实时更新的变量
    char                match_mode_;
    char                pre_match_mode_;
    DPoint              robot_pos_;
    DPoint              ball_pos_;
    DPoint              ball_vel_;    
    Angle               robot_ori_;
    bool                shootflg_;
    int                 handle_enable_;
    bool                ballisHolding_;
    float               strength_;
    int                 shootPos_;
    bool                isPenalty_;
    bool                isObstacle_;
    bool                isNewpassing_;
    int                 shootcnt_;
    ros::Time           start_time_;
    int                 maxvel_;
    int                 maxw_;
    bool                _isOpposite ;///////@hbx

    /// 最开始的时候足球的位置（与实时足球位置相减用于记录足球的移动距离，判断带球是否3米）
    DPoint              ballInitPos_;

public:
    /// 世界模型信息
    World_Model_Info world_model_info_;
    /// 策略信息
    Strategy        *m_strategy_;
    /// 单个机器人动作
    Plan             m_plan_;
    /// 静态发球站位
    StaticPass       m_staticpass_;
    /// 测试函数
    NubotTest       *test_;
    /// 场地参数
    FieldInformation field_;

    dynamic_reconfigure::Server<nubot_control::nubotcontrolConfig> reconfigureServer_;

public:
    NuBotControl(int argc, char **argv);
    ~NuBotControl();

    //! 更新全局信息
    void update_world_model_info(const nubot_common::WorldModelInfo & _world_msg);
    /** 球的三维信息,用于守门员角色*/
    void goalieBall3dCallbackL(const nubot_common::BallInfo3d  &_BallInfo_3d);
    void goalieBall3dCallbackR(const nubot_common::BallInfo3d  &_BallInfo_3d);
    /** Kinect 回调*/
    void obstaclesInfo3dCallback(const nubot_common::ObstaclesInfo3d  &_ObstaclesInfo_3d);
    /** 带球状态回调函数 */
    void ballHoldingCallback(const nubot_common::BallIsHolding & _ballisholding);
    /** 主要的控制框架位于这里*/
    void loopControl(const ros::TimerEvent& event);
    /** 正常比赛的部分，包括传接球 **/
    void normalGame();
    /** 比赛开始前的站位 **/
    void positioning();
    /** 自动回位 **/
    void parking();
    /** 点球部分 **/
    void penaltyStart();
    /** 发布策略消息 **/
    void pubStrategyInfo();
    /** 判断是否带球 **/
    void handleBall();
    /** 判断是传球还是踢球 **/
    void kickBall();
    /** 测试 **/
    void test();
    //! 中场与助攻在机器人动态传球时会出现穿过传球线的现象，在此矫正传球时候，中场与助攻的跑位点，防止传球失败
    void coorrect_target(bool & isDynamicStart, bool & isDynamicOut,  bool & IsDribble ,
                         const DPoint & robot_pos, DPoint & target);
    //! 判断点是否在多边形内部
    bool pnpoly(const std::vector<DPoint> & pts, const DPoint & test_pt);
    //! 发布速度消息
    void setEthercatCommond();
    void configure(const nubot_control::nubotcontrolConfig & config, uint32_t level);
    //! 接收反向命令////@hbx
    void Opposite(const std_msgs::Bool& isOpposite_msg);
};

}

#endif /* _NUBOT_CONTROL_H */
