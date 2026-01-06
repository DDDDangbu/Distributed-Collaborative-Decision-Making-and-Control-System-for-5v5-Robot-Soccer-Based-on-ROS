#include "nubot/nubot_control/passstrategy.h"

using namespace nubot;

PassStrategy::PassStrategy()
{
}

void
PassStrategy::process()
{
    DPoint robot_pos= world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    DPoint ball_pos = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    static int  delay = 0;
    static bool is_pass = false;
    target_orientation =  ball_pos.angle(robot_pos).radian_;
    if(world_model_->pass_cmds_.isvalid  && world_model_->pass_cmds_.catchrobot_id == world_model_->AgentID_)
    {
       if(world_model_->pass_cmds_.is_pass) //! 这时候处于跑位状态，跑到接球点
       {
           m_plan_->m_behaviour_.move2Position(2.75,1.5,world_model_->pass_cmds_.catch_pt,MAXVEL,robot_pos,robot_ori);
           m_plan_->m_behaviour_.rotate2AbsOrienation(1.785,1.2,target_orientation,10);
       }
       if(world_model_->pass_cmds_.is_passout) //! 足球已经传出，表示接球状态
          is_pass = true;
     }
    if(is_pass)
        delay++;
    if(delay > 30)
    {
         m_plan_->catchBall();
    }
}
