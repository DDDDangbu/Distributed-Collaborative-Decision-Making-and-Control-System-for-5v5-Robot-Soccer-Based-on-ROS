#include "nubot_control/strategy.hpp"

using namespace nubot;
Strategy::Strategy(World_Model_Info & _world_model, Plan & _plan){
    world_model_ = & _world_model;
    m_plan_ = & _plan;
    RoleAssignment_.world_model_ = world_model_;
    RoleAssignment_.m_plan_=m_plan_;

    ActiveRole_.m_plan_ = m_plan_;
    ActiveRole_.world_model_ = world_model_;

    AssistRole_.world_model_ = world_model_;
    AssistRole_.plan_ = m_plan_;

    PassiveRole_.world_model_ = world_model_;
    PassiveRole_.plan_ = m_plan_;

    MidfieldRole_.world_model_ = world_model_;
    MidfieldRole_.plan_ = m_plan_;
    selected_role_ = NOROLE;
    selected_action_ = Positioned;

    assist_pt_ = DPoint(0.0, 0.0);
    midfield_pt_ = DPoint(0.0, 0.0);
    passive_pt_ = DPoint(0.0, 0.0);

}
Strategy::Strategy(){
    RoleAssignment_.world_model_ = world_model_;
    selected_role_ = NOROLE;

    assist_pt_ = DPoint(0.0, 0.0);
    midfield_pt_ = DPoint(0.0, 0.0);
    passive_pt_ = DPoint(0.0, 0.0);
}
Strategy::~Strategy(){
}
bool
Strategy::passStrategy()
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    DPoint ball_pos  = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    Angle target_orientation =  ball_pos.angle(robot_pos);
    if(world_model_->pass_cmds_.isvalid  && world_model_->pass_cmds_.catchrobot_id == world_model_->AgentID_)
    {
        if(world_model_->pass_cmds_.is_dynamic_pass && !world_model_->pass_state_.is_passing_) //! 这时候处于跑位状态，跑到接球点
        {
            Angle delta_ang = target_orientation- robot_ori;
            if(robot_pos.distance(world_model_->pass_cmds_.catch_pt)>50 || fabs(delta_ang.radian_)>Angle(30,false).radian_)
            {
                m_plan_->move2Positionwithobs(world_model_->pass_cmds_.catch_pt,200,true);
                m_plan_->m_behaviour_.setOrienation(target_orientation.radian_,10,Catch_Positioned);
            }
            else if(robot_pos.distance(world_model_->pass_cmds_.catch_pt)<50 || fabs(delta_ang.radian_)>Angle(8,false).radian_ )
            {
                m_plan_->m_behaviour_.setTarget(world_model_->pass_cmds_.catch_pt,20,Catch_Positioned);
                m_plan_->m_behaviour_.setOrienation(target_orientation.radian_,10,Catch_Positioned);
            }
            selected_action_ = Catch_Positioned;
            ActiveRole_.currentstate_  = No_Action;
            return true;
        }
    }
    return false;
}

void Strategy::process()
{
    selectRole();
    selectAction();
}

void Strategy::calPassPositions()
{
    /// 助攻站位点计算
    AssistRole_.assistCalculate();
    /// 将助攻站位点传递到主攻和中场的类中
    ActiveRole_.setAssistPt(AssistRole_.assist_pt_);
    /// 中场站位点计算
    MidfieldRole_.midCalculate();
    /// 防守站位点计算
    PassiveRole_.passiveCalculate();
//    ROS_INFO("passive x: %f y: %f ",PassiveRole_.passive_pt_.x_,PassiveRole_.passive_pt_.y_);
    /// 将助攻、中场、防守的站位点传递到角色分配的类中
    RoleAssignment_.setPts(AssistRole_.assist_pt_, MidfieldRole_.midfield_pt_, PassiveRole_.passive_pt_);
}
 void
 Strategy::selectRole()
 {
     DPoint ball_pos = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
     selected_role_ = RoleAssignment_.process();//!选择角色，考虑角色发起状态，当处于发球状态时属于主攻；
     /// 我方控球状态，直接跑位
     if(world_model_->CurActiveNotGoalieNums_ == 2 && selected_role_ !=ACTIVE)
     {
         if(world_model_->IsOurDribble_&&ball_pos.x_ >0)// || world_model_->pass_state_.is_passing_||world_model_->is_dynamic_pass_))
             selected_role_ = ASSISTANT;
         else
             selected_role_=PASSIVE;
     }
     if(world_model_->CurActiveNotGoalieNums_ == 1)
          selected_role_ =ACTIVE;
     if(world_model_->AgentID_==1)
         selected_role_ = GOALIE;
     if(selected_role_!=ACTIVE)
         ActiveRole_.clearActiveState();
 }
 void
 Strategy::selectAction()
 {
     /** 分配机器人角色，得到角色信息，防守、进攻、主攻等等*/
     DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
     Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
     if(passStrategy()) //! 直接进入传球跑位决策；
           return;
     switch (selected_role_) {
     case GOALIE:
         selected_action_ = Positioned;
         goalie_strategy_.ballTrack();
         if(world_model_->CoachInfo_.MatchMode !=STOPROBOT)
         {
           goalie_strategy_.strategy();
           if( robot_pos.distance(goalie_strategy_.dest_point_)>10 )
             m_plan_->m_behaviour_.setTarget(goalie_strategy_.dest_point_,420,Positioned);//modify 100 to 500,maxvel threshold
           if( fabs(Angle(robot_ori-goalie_strategy_.dest_angle_).radian_) > deg(5) )
             m_plan_->m_behaviour_.setOrienation(goalie_strategy_.dest_angle_,10,Positioned);
         }
        break;
     case ACTIVE:
         ActiveRole_.process();
         selected_action_ = ActiveRole_.currentstate_;
         break;
     case PASSIVE:
         selected_action_ = Positioned;
         PassiveRole_.process();
         break;
     case MIDFIELD:
         selected_action_ = Positioned;
         MidfieldRole_.process();
         break;
     case ASSISTANT:
         selected_action_ = Positioned;
         AssistRole_.process();
         break;
     case ACIDPASSIVE:
         break;
     case GAZER:
         break;
     case BLOCK:
         break;
     case NOROLE:
         break;
     default:
         break;
     }
 }
