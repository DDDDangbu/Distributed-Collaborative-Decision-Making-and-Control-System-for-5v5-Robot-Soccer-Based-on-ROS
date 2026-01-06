#include "nubot_control/activerole.h"

using namespace nubot;

ActiveRole::ActiveRole()
{
    stuckflg_ = false;
    pass_lock_ = 0;
    stucktime_ = 0;
    isturn_=true;
    NeedEvaluat = false;
    dynamic_shoot_count_ = 0;
    quick_shoot_count_ = 0;
    currentstate_ = CanNotSeeBall;
    kick_enable_ =false;
    assist_pt_=field_.oppGoal_[GOAL_MIDDLE];
}
ActiveRole:: ~ActiveRole()
{
}

/// \brief  找到最近的障碍物，本函数用于在静态球发球阶段，利用Kinect找到面朝最近的障碍物，即为要传球的队友
void
ActiveRole::Obsnearest(nubot_common::ObstaclesInfo3d &obs, DPoint &gltarget)
{
    float pos_xmin = 600;
    for(vector<nubot_common::Point3d>::iterator it = obs.pos.begin(); it != obs.pos.end(); it++)
    {
        DPoint target_tmp;
        PPoint real_pt;
        DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
        Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();

        target_tmp.x_ = (*it).x;
        target_tmp.y_ = (*it).y;
        PPoint target_poly = DPoint(target_tmp.x_,target_tmp.y_);
        DPoint target_globle = robot_pos + DPoint(target_poly.rotate(robot_ori));

        if(!field_.isInInterField(target_globle))
            continue;
        real_pt = PPoint(DPoint(target_tmp.x_ ,target_tmp.y_));

        float pos_xmin_tmp =  real_pt.radius_;
        if( pos_xmin_tmp <= pos_xmin)
        {
            pos_xmin = pos_xmin_tmp;
            gltarget = target_globle;
        }
    }
}

void ActiveRole::setAssistPt(DPoint &assist)
{
    assist_pt_ = assist;
}

//传球
void
ActiveRole::kickball4Coop(DPoint target)
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();

    static bool towardacive_coop=false;
    static int  kickrightface_cnt = 0;
    Angle  theta = target.angle(robot_pos);
    Angle  thetaerr = theta - robot_ori;

    if(Obstacles3d.pos_known_3d && fabs(thetaerr.radian_)<deg(5))
    {
        Obsnearest(Obstacles3d, target);
        theta = target.angle(robot_pos);
        thetaerr = theta - robot_ori;
    }

    if(world_model_->DribbleState_.is_dribble_ && towardacive_coop )
    {
        towardacive_coop=false;
        kick_enable_ = true;
        if(world_model_->CurActiveNotGoalieNums_ ==1)
            kick_force_ = 16.2;
        else
            kick_force_ = calculateKickforce(target,PASS);
        kickrightface_cnt = 0;
    }
    else
    {
        if (!world_model_->DribbleState_.is_dribble_ )
        {
            m_plan_->catchBallForCoop();
        }
        else
        {
            if (fabs(thetaerr.radian_)<deg(5))
            {
                kickrightface_cnt++;
                if (kickrightface_cnt>=3)
                {
                    kickrightface_cnt=0;
                    towardacive_coop=true;
                }
                else
                    towardacive_coop=false;
            }
            else
            {
                kickrightface_cnt=0;
                towardacive_coop=false;
            }
            if (!towardacive_coop)
            {

                m_plan_->m_behaviour_.setTarget(robot_pos,MAXVEL,KickCoop_turn);
                m_plan_->m_behaviour_.setOrienation(theta.radian_,MAXW,KickCoop_turn);
            }
            else
                m_plan_->m_behaviour_.clear();
        }
    }
}

//主攻流程函数
void
ActiveRole::process()
{
    DPoint ball_pos  = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    if(world_model_->CoachInfo_.MatchMode ==STOPROBOT)
    {
        clearActiveState();
        m_plan_->m_behaviour_.clear();
        currentstate_ = No_Action;
        return;
    }
    if(world_model_->CoachInfo_.MatchMode == STARTROBOT)
    {
        /// 静态传球前半段，当前主攻为发球机器人
        if(world_model_->is_static_pass_)//静态传球
        {
            DPoint target =field_.oppGoal_[GOAL_MIDDLE];
            //存在接球目标，传球机器人非接球目标，非守门员活跃机器人数量大于1
            if(world_model_->catch_ID_ > 0 && world_model_->catch_ID_ < 6 && world_model_->catch_ID_!=world_model_->pass_ID_ && world_model_->CurActiveNotGoalieNums_>1)
                target = world_model_->catch_pt_;//接球机器人位置
            kickball4Coop(target);//向target传球
            currentstate_ = StaticPass;
            return;
        }
        /// 静态传球后半段，主攻已经切换到接球机器人
        double ball_moving_dis = world_model_->pass_state_.pass_pt_.distance(ball_pos);
        if(world_model_->pass_state_.catch_id_ == world_model_->AgentID_ && world_model_->pass_state_.is_passing_ && ball_moving_dis < 50 )
        {
            m_plan_->m_behaviour_.clear();
            currentstate_ = Catch_Positioned;
        }
        else
        {
            selectCurrentState();
            selectCurrentAction(currentstate_);
        }
    }
}

void
ActiveRole::stuckProcess()
{
    if(world_model_->DribbleState_.is_dribble_)
    {
        if(stucktime_ < 35)
            m_plan_->m_behaviour_.setTarget(DPoint(0,0),300,Stucked);
        else
            m_plan_->m_behaviour_.setTarget(DPoint(0,0),-100,Stucked);
    }
    else
        activeCatchBall();
    stuckflg_ = true;
}

void
ActiveRole::clearActiveState()
{
    kick_enable_ = false;
    stucktime_ = 0;
    NeedEvaluat = false;
    dynamic_shoot_count_ = 0;
    quick_shoot_count_ = 0;
    stuckflg_ = false;
    pass_lock_ = 0;
    m_plan_->m_behaviour_.clear();
    currentstate_ = No_Action;
}

//估算当前位置是否适合射门

bool
ActiveRole::evaluateKick()
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    double shooting_willing = calculateShootingWilling(robot_pos);
    /// 若在己方场地，不能射门
    if(robot_pos.x_<-LOCATIONERROR)
        return false;

    /// 若当前朝向指向球门以外，不能射门
    float  leftOppgoalCorner  =  field_.oppGoal_[GOAL_UPPER].angle(robot_pos).radian_;
    float  rightOppgoalCorner =  field_.oppGoal_[GOAL_LOWER].angle(robot_pos).radian_;

    if((robot_ori.radian_>leftOppgoalCorner + 3/180 *M_PI)||robot_ori.radian_<rightOppgoalCorner - 3/180 *M_PI) //loose the limitation of shooting
        return false;

    /// 若机器人正前方一定区域内存在障碍物，不能射门
    if(!m_plan_->isNullInTrap(world_model_->Opponents_,robot_pos,robot_ori,30,40,0,150)) //can shoot area
        return false;

    if(shooting_willing < 100)
        return false;

    return true;
}

//估算当前位置是否适合传球
bool
ActiveRole::evaluatePass()
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    DPoint assist_pos   = m_plan_->getWhichRolePos(ASSISTANT); //get position of assistant and midfield
    DPoint midfield_pos = m_plan_->getWhichRolePos(MIDFIELD);
    double assist_willing = calculateShootingWilling(assist_pos); //if assist or midfield can shoot
    double midfield_willing = calculateShootingWilling(midfield_pos);
    if(world_model_->CurActiveNotGoalieNums_==1)
        return false;
    else if(world_model_->CurActiveNotGoalieNums_==2)
    {
        DPoint _assist=m_plan_->getWhichRolePos(ASSISTANT);
        if(_assist.x_!=10000)
        {
            LineSegment Line = LineSegment(_assist,robot_pos);
            double line_length = Line.distance();
            Angle  robot2assist=_assist.angle(robot_pos);
            if(fabs((robot2assist-robot_ori).radian_)<1 && line_length>200)
            {
                if(world_model_->Opponents_.size()==0)
                {
                    world_model_->catch_pt_ = _assist;
                    world_model_->catch_ID_ = m_plan_->getWhichRoleID(ASSISTANT);
                    return true;
                }
                else
                    for(int i=0 ; i<world_model_->Opponents_.size() ; i++)
                    {
                        if(Line.distance(world_model_->Opponents_[i],true) < robot_pos.distance(world_model_->Opponents_[i])*0.3)
                            break;
                        else if(i==world_model_->Opponents_.size()-1)
                        {
                            world_model_->catch_pt_ = _assist;
                            world_model_->catch_ID_ = m_plan_->getWhichRoleID(ASSISTANT);
                            return true;
                        }
                    }
            }
        }
    }
    /// 所有机器人都在场上
    else
    {
        DPoint _assist=m_plan_->getWhichRolePos(ASSISTANT);
        DPoint _middle=m_plan_->getWhichRolePos(MIDFIELD);
        if(_assist.x_!=10000)
        {
            LineSegment Line = LineSegment(_assist,robot_pos);
            double line_length = Line.distance();// get the distance between assist and robot
            Angle  robot2assist=_assist.angle(robot_pos);
            if(fabs((robot2assist-robot_ori).radian_)<3 && line_length>200)
            {
                if(world_model_->Opponents_.size()==0)
                {
                    world_model_->catch_pt_ = _assist;
                    world_model_->catch_ID_ = m_plan_->getWhichRoleID(ASSISTANT);
                    return true;
                }
                else
                    for(int i=0 ; i<world_model_->Opponents_.size() ; i++)
                    {
                        if(Line.distance(world_model_->Opponents_[i],true) < robot_pos.distance(world_model_->Opponents_[i])*0.4) //whether pass path is be occupied by obstacles?
                            break;
                        else if(i==world_model_->Opponents_.size()-1)
                        {
                            world_model_->catch_pt_ = _assist;
                            world_model_->catch_ID_ = m_plan_->getWhichRoleID(ASSISTANT);
                            return true;
                        }
                    }
            }
        }
        if(_middle.x_!=10000)
        {
            LineSegment Line = LineSegment(_middle,robot_pos);
            double line_length = Line.distance();
            Angle  robot2middle=_middle.angle(robot_pos);
            if(fabs((robot2middle-robot_ori).radian_)<3 && line_length>200)
            {
                if(world_model_->Opponents_.size()==0)
                {
                    world_model_->catch_pt_ = _assist;
                    world_model_->catch_ID_ = m_plan_->getWhichRoleID(ASSISTANT);
                    return true;
                }
                else
                    for(int i=0 ; i<world_model_->Opponents_.size() ; i++)
                    {
                        if(Line.distance(world_model_->Opponents_[i],true) < robot_pos.distance(world_model_->Opponents_[i])*0.4)
                            break;
                        else if(i==world_model_->Opponents_.size()-1  && midfield_willing > assist_willing) //when midfield is more suitable to shoot, pass ball to midfield
                        {
                            world_model_->catch_pt_ = _middle;
                            world_model_->catch_ID_ = m_plan_->getWhichRoleID(MIDFIELD);
                            return true;
                        }
                    }
            }
        }
    }
    return false;
}

bool
ActiveRole::passisBetter()
{
    DPoint robot_pos    = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori    = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    DPoint assist_pos   = m_plan_->getWhichRolePos(ASSISTANT);
    DPoint midfield_pos = m_plan_->getWhichRolePos(MIDFIELD);
    Angle  robot2goal   = field_.oppGoal_[GOAL_MIDDLE].angle(robot_pos); //distance between robot and mid of goal line
    Angle  assist2goal  = field_.oppGoal_[GOAL_MIDDLE].angle(assist_pos);
    Angle  midfield2goal= field_.oppGoal_[GOAL_MIDDLE].angle(midfield_pos);
    bool   isNullrobot  = m_plan_->isNullInTrap(world_model_->Opponents_,robot_pos,robot2goal,50,80,0,150); //size and location of this rectangle
    bool   isNullassist = m_plan_->isNullInTrap(world_model_->Opponents_,assist_pos,assist2goal,50,80,0,150);
    bool  isNullmidfield= m_plan_->isNullInTrap(world_model_->Opponents_,midfield_pos,midfield2goal,50,80,0,150);
    float  dribble_dis  = robot_pos.distance(world_model_->DribbleState_.satrt_point_);
    double assist_willing = calculateShootingWilling(assist_pos);
    double midfield_willing = calculateShootingWilling(midfield_pos);
    double shooting_willing = calculateShootingWilling(robot_pos);

    ROS_INFO("shooting_willing:[%f]",shooting_willing);
    ROS_INFO("assist_willing:[%f]",assist_willing);
    ROS_INFO("midfield_willing:[%f]",midfield_willing);

    /// 若场上就1台机器人，则不能传球
    if(world_model_->CurActiveNotGoalieNums_==1)
        return false;
    /// 若在我方半场接球且距离中线1m，或是重新获球，传球肯定是最优选择
    if(world_model_->DribbleState_.satrt_point_.x_<-300||!world_model_->RegainBall_)
        return true;
    else if(!isNullrobot&&isNullassist&&assist_pos.x_>-100&&assist_pos.x_!=10000)
        return true;
    else if(world_model_->CurActiveNotGoalieNums_==4&&!isNullrobot
            &&isNullmidfield&&midfield_pos.x_>-100&&midfield_pos.x_!=10000)
        return true;
    else if(assist_willing >shooting_willing || midfield_willing>shooting_willing)
        return true;

    else
        return false;
}

/// \brief 判断是否带球3m
bool
ActiveRole::checkPass()
{
    bool rtvl =  false;
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    if(world_model_->DribbleState_.is_dribble_ )
    {
        if(robot_pos.distance(world_model_->DribbleState_.satrt_point_) > 250)
            rtvl = true;
    }
    return rtvl;
}

/// \brief  计算传球势能值
/// \return 最大的传球势能值以及传球点
void
ActiveRole::caculatePassEnergy(double & energy, int & label)
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    double max_pass_energe = -10000;
    int    selected_index = -1;
    std::vector<double>  our_energe;
    std::vector<double>  opp_energe;
    std::vector<int>     our_label;
    std::vector<int>     opp_label;
    energy  = 0;
    for(int i = 1; i < OUR_TEAM ; i++)
    {
        if(world_model_->RobotInfo_[i].isValid() && i!=world_model_->AgentID_-1)
        {
            /// 传球距离因素
            LineSegment Line = LineSegment(world_model_->RobotInfo_[i].getLocation(),robot_pos);
            double line_length = Line.distance();
            energy = (line_length > 750||line_length < 350) ? 0.1: (0.4-0.0015*fabs(line_length - 550));

            /// 障碍物因素（包括对方机器人以及我方机器人)
            int near_obs = 0;
            for(int j = 0 ; j < world_model_->Opponents_.size() ; j++)
            {
                if(Line.distance(world_model_->Opponents_[j],true) < 100 )
                    near_obs++;
            }
            for(int j = 0 ; j < OUR_TEAM; j++) //judge whether our other robots is obstacles
            {
                if(j != i && j!=world_model_->AgentID_-1 && world_model_->RobotInfo_[j].isValid())
                {
                    if(Line.distance(world_model_->RobotInfo_[j].getLocation(),true)<50)
                        near_obs++;
                }
            }

            if(near_obs>3)  near_obs = 3;
            energy += 0.15 * (3-near_obs)+0.05;

            /// 角度因素
            Angle delta_ang = world_model_->RobotInfo_[i].getHead() - robot_ori;
            energy += 0.1/SINGLEPI_CONSTANT * fabs(delta_ang.radian_)+0.1;

            /// 当重新获得球权时，必须传球，所以适当增大传球势能值
            if(!world_model_->RegainBall_)
            {
                energy+=0.4;
                if(energy>1) energy=1;
            }

            if(world_model_->RobotInfo_[i].getLocation().x_ > 50)
            {
                if(robot_pos.x_<-500)
                {
                    energy+=0.2;
                    if(energy>1) energy=1;
                }

                opp_energe.push_back(energy);
                opp_label.push_back(i+1);
            }
            else
            {
                if((world_model_->RobotInfo_[i].getLocation().x_ -robot_pos.x_)>150)
                {
                    our_energe.push_back(energy);
                    our_label.push_back(i+1);
                }
            }
        }
    }
    if(opp_label.size() > 0)  //! 选择合适的机器人作为接球机器人
    {
        for(int j = 0; j < opp_label.size();j++)
        {
            if(opp_energe[j]>max_pass_energe)
            {
                max_pass_energe = opp_energe[j];
                selected_index = opp_label[j];
            }
        }
    }
    else if(our_label.size()>0)
    {
        for(int j = 0; j < our_label.size();j++)
        {
            if(our_energe[j] > max_pass_energe)
            {
                max_pass_energe = our_energe[j];
                selected_index  = our_label[j];
            }
        }
    }
    energy = max_pass_energe;
    label  = selected_index;
}
void
ActiveRole::caculateDribblingEnergy(double & avoid_enegy, bool isNullFrontRobot)
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    int nums_obs = world_model_->Obstacles_.size();
    int obsNearRobot = 0;
    avoid_enegy = 0;
    for(int j = 0 ; j < nums_obs ; j++)
    {
        if(world_model_->Obstacles_[j].distance(robot_pos) < 150)
            obsNearRobot++;
    }
    /// 若周围无一障碍物，选择带球
    if(obsNearRobot==0)
        avoid_enegy=1;
    else
    {
        /// 若前方存在障碍物，选择带球
        if(!isNullFrontRobot)
            avoid_enegy = 1;
        else
        {
            if(obsNearRobot <= 2)
                avoid_enegy = 0.6;
            else
                avoid_enegy = 0.2;
        }

    }
}

/// \brief 若目前机器人无法完成射门，则接下来的动作将在传球与带球避障中选择
void
ActiveRole::selectDribblingOrPassing(bool isNullFrontRobot)
{
    /// 选择基于传球势能值以及避障势能值
    double pass_energy = 0;
    double avoid_enegy = 0;
    double possibility = 0.51;
    int selected_index = -1;

    caculatePassEnergy(pass_energy, selected_index);
    caculateDribblingEnergy(avoid_enegy, isNullFrontRobot);
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();

    if(pass_lock_ > 0)
        currentstate_  = TurnToPass;
    /// 强制性的直接带球过中心后，因为可以直接射门
    else if(robot_pos.x_ > -150 && world_model_->RegainBall_ && avoid_enegy > 0.5 && evaluateKick())
        //currentstate_  = AvoidObs;
        currentstate_ = TurnForShoot;
    else if(pass_energy*possibility>avoid_enegy*(1-possibility))
        currentstate_  = TurnToPass;
    else
        currentstate_  = AvoidObs;
    /// pass_lock_ 传球点的位置保持或者传球点上一次在己方，这一次在对方，需要调整；
    if(currentstate_  == TurnToPass)
    {
#if 1
        if(selected_index != -1)
        {
            if( world_model_->catch_ID_ ==-1 || pass_lock_==0)
            {
                world_model_->catch_pt_ = world_model_->RobotInfo_[selected_index-1].getLocation();
                world_model_->catch_ID_ = selected_index;
                pass_lock_ = 0;
            }
        }
        else
        {
            world_model_->catch_ID_ = -1;
            world_model_->catch_pt_ = field_.oppGoal_[GOAL_MIDDLE];
        }
#else
        int passive_id = -1;
        DPoint passive_pos;
        for(int i=0; i<5;i++)
        {
            if(world_model_->RobotInfo_[i+1].isValid() && world_model_->RobotInfo_[i+1].getCurrentRole() == PASSIVE)
            {
                passive_id = i+1;
                passive_pos = world_model_->RobotInfo_[passive_id-1].getLocation();
            }
        }
        if(passive_id != -1)
        {
            if( world_model_->catch_ID_ ==-1 || pass_lock_==0)
            {
                world_model_->catch_pt_ = passive_pos;
                world_model_->catch_ID_ = passive_id;
                pass_lock_ = 0;
            }
        }
        else
        {
            world_model_->catch_ID_ = -1;
            world_model_->catch_pt_ = field_.oppGoal_[GOAL_MIDDLE];
        }
        ROS_INFO("pass_pos:[%f %f], pas_id:[%d]", world_model_->catch_pt_.x_, world_model_->catch_pt_.y_, world_model_->catch_ID_);
#endif
    }
}

/// \brief 根据当前的机器人位置、足球的状态选择机器人状态
void
ActiveRole::selectCurrentState()
{
    static int dribble_delay=0;
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    bool   is_robot_stuck = world_model_->RobotInfo_[world_model_->AgentID_-1].isStuck();
    bool   isNullFrontRobot = m_plan_->isNullInTrap(world_model_->Obstacles_,robot_pos,robot_ori,50,75,-25,150);

    /** 判断机器人周围是否存在着障碍物，或者是机器人是否堵转，如果为true，则状态赋值*/
    if(is_robot_stuck && !isNullFrontRobot)
    {
        stucktime_++;
        if(stucktime_ > 150 && stucktime_ < 300)
            currentstate_ = TurnToPass;
        else if (stucktime_ > 300)
            currentstate_ = TurnForShoot;
        else
            currentstate_ = Stucked;
    }
    //! 足球没看到球NOTSEEBALL，机器人状态是CanNotSeeBall
    else if(world_model_->BallInfoState_ == NOTSEEBALL && !world_model_->DribbleState_.is_dribble_)
        currentstate_ = CanNotSeeBall;
    else //! 如果看到球，判断是否带上球
    {
        if(!world_model_->DribbleState_.is_dribble_)
        {
            currentstate_ = CatchBall;
            dribble_delay=0;
        }   //! 如果看到球，判断是需要传球
        else if(world_model_->DribbleState_.is_dribble_)
        {
            //! 持球后的基础状态是避障，但会随时评估是否能够射门及传球
            currentstate_ = AvoidObs;
            //! 若场地上只有一个非守门员机器人，或者非重新获得球权并且在对方半场，可以考虑射门 //zdx_note conider is opp_field and evaluatekick() is true, then shoot
            //if((world_model_->CurActiveNotGoalieNums_==1|| !world_model_->RegainBall_)&&!world_model_->field_info_.isOurField(robot_pos)&&evaluateKick())
            if(world_model_->RegainBall_&&!world_model_->field_info_.isOurField(robot_pos)&&evaluateKick())
            //currentstate_ = AtShootSituation;
                currentstate_ = TurnForShoot;
                //currentstate_ = AvoidObs;
            //! 如果机器人处于不能射门状态，考虑是否应该传球
            else if(passisBetter()&&evaluatePass())
                currentstate_ = TurnToPass;

            if(passisBetter())
                ROS_INFO("passbetter");

            if(!evaluatePass())
                ROS_INFO("not evaluate");
        }
    }

    if(currentstate_ != Stucked)
    {
        stucktime_ = 0;
        stuckflg_ = false;
    }
    if(currentstate_ != TurnToPass)
        pass_lock_ = 0;
}

/// \brief 根据当前的机器人状态选择动作
void
ActiveRole::selectCurrentAction(unsigned char state)
{
    unsigned char MyState = state;

    switch(MyState)
    {
    case Stucked:
        stuckProcess();
        break;
    case CatchBall:
        activeCatchBall();
        break;
    case AvoidObs:
        NewAvoidObs();
        break;
    case AtShootSituation:
        world_model_->catch_pt_ = world_model_->field_info_.oppGoal_[GOAL_MIDDLE];
        isturn_=true;
        triggerShoot();
        break;
    case TurnForShoot:
        turn4Shoot();
        break;
    case TurnToPass:
        isturn_=true;
        turn2Pass();
        break;
    case CanNotSeeBall:
        findBall();
        break;
    default:
        currentstate_ =  No_Action;
        break;
    }

}

/// \brief  转身射门动作，目前该函数在正常比赛中已近弃用，保留在此供点球调用-daiwei 5.29
void ActiveRole::turn4Shoot()
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();

    Angle  theta2target_str = field_.oppGoal_[GOAL_MIDDLE].angle(robot_pos); //this angle can be modified
    //zdx_modify:caculate of angle to shoot
    Angle  theta2target_left = field_.oppGoal_[GOAL_UPPER].angle(robot_pos);
    Angle  theta2target_right = field_.oppGoal_[GOAL_LOWER].angle(robot_pos);
    Angle  theta2target = theta2target_left - theta2target_right;
    Angle  biasAng = Angle(1,false);
    //Angle  leftdelta  = theta2target + biasAng;
    //Angle  rightdelta = theta2target - biasAng;

    Angle  leftdelta  = theta2target_left + biasAng;
    Angle  rightdelta = theta2target_right - biasAng;


    if( robot_ori.radian_ < leftdelta.radian_ && robot_ori.radian_ > rightdelta.radian_
            /*&& fabs( (robot_ori-theta2target).radian_ ) < deg(3)*/)
    {
            currentstate_ = AtShootSituation;
            selectCurrentAction(AtShootSituation);
            m_plan_->m_behaviour_.move_action_=AtShootSituation;
            m_plan_->m_behaviour_.rotate_action_=AtShootSituation;
    }
    else
    {
        if(m_plan_->rotateMode(world_model_->Obstacles_,robot_pos,robot_ori,theta2target))
            m_plan_->m_behaviour_.setOrienation(theta2target_str.radian_,4,TurnForShoot_Robot,1);
        else
            m_plan_->m_behaviour_.setOrienation(theta2target_str.radian_,4,TurnForShoot_Robot,-1);
    }
}

/// \brief 传球动作
void ActiveRole::turn2Pass()
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    Angle  theta2pass= world_model_->catch_pt_.angle(robot_pos);
    Angle  d_theta2pass = theta2pass - robot_ori;
    bool   is_catch_location = false;

    if(pass_lock_==0)
        pass_lock_=1;
    if(world_model_->catch_ID_ >0 && world_model_->catch_ID_< 6)
        if( world_model_->catch_pt_.distance(world_model_->RobotInfo_[world_model_->catch_ID_-1].getLocation()) < 50)
            is_catch_location =true;

    if(fabs(d_theta2pass.radian_) > deg(5) || !is_catch_location)
    {
        if(fabs(d_theta2pass.radian_) > deg(5) )
        {
            if(m_plan_->rotateMode(world_model_->Opponents_,robot_pos,robot_ori,theta2pass))
                m_plan_->m_behaviour_.setOrienation(theta2pass.radian_,6,TurnToPass_Robot,1);
            else
                m_plan_->m_behaviour_.setOrienation(theta2pass.radian_,6,TurnToPass_Robot,-1);
        }
        pass_lock_ = 1;
    }
    /// 当朝向对准接球机器人
    else
    {
        if(pass_lock_==3)
        {
          kick_enable_= true;
          kick_force_ = calculateKickforce(world_model_->catch_pt_,PASS);
        }
        pass_lock_ ++;
    }
}

/// \brief 未发现足球时的找球动作
void
ActiveRole::findBall()
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    DPoint target;
    static bool ourFieldSearch = true;
    if(world_model_->NoSeeBallNums_ < 10)
    {
        if(world_model_->field_info_.isOurField(world_model_->lastBallPosition_))
            ourFieldSearch = true;
        else
            ourFieldSearch = false;
    }
    if(world_model_->NoSeeBallNums_ < 10)
        return;
    if(ourFieldSearch)
    {
        target =  DPoint(-850,0);  //650
        if(robot_pos.distance(target)< 50)
            ourFieldSearch = true;
    }
    else
    {
        target =  DPoint(850,0);
        if(robot_pos.distance(target)< 50)
            ourFieldSearch = false;
    }
    Angle theta = target.angle(robot_pos);
    if(world_model_->CurActiveNotGoalieNums_>1)
    {
        m_plan_->move2Positionwithobs(target,300);
        m_plan_->m_behaviour_.setOrienation(theta.radian_,MAXW,CanNotSeeBall);
    }
    ROS_INFO("find ball1");
}

/// \brief 持球后的带球动作
void
ActiveRole::NewAvoidObs()
{
    /// 首先计算带球运动的目标点
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    DPoint target    = m_plan_->getAvailablePosIn3m(world_model_->DribbleState_.satrt_point_);

    /// 防止机器人避障直接进去对方小禁区的补丁
    if(field_.isOppPenalty(target))
    {
        target.y_=((FIELD_XLINE2-LOCATIONERROR)-robot_pos.x_)*(target.y_-robot_pos.y_)/(target.x_-robot_pos.x_)+robot_pos.y_;
        target.x_=FIELD_XLINE2-LOCATIONERROR;
    }

    /// 指向target的朝向
    Angle target_orientation = target.angle(robot_pos);
    /// 到达target时的机器人朝向
    Angle target_ori=target_orientation;

    /// 避障动作
    bool isNullFront = m_plan_->isNullInTrap(world_model_->Opponents_,robot_pos,0,50,75,-25,150);
    static bool turnmode=false;
    static bool isBackObs=false;
    if(!isNullFront||!isturn_)
    {
        isturn_=false;
        /// 前方有障碍物时，暂时背对障碍物
        float tmp_ori=M_PI;

        if(fabs(angularnorm(robot_ori.radian_-tmp_ori))>0.5&&!isBackObs)
        {
            m_plan_->m_behaviour_.setOrienation(tmp_ori,MAXW,AvoidObs);
            turnmode=m_plan_->rotateMode(world_model_->Opponents_,robot_pos,robot_ori,target_orientation);
        }
        else
        {
            /// 机器人已几乎转过来
            if(fabs(robot_ori.radian_)<1)
            {
                isBackObs=false;
                isturn_=true;
            }

            float vel=350-robot_pos.distance(world_model_->DribbleState_.satrt_point_);
            if(vel<100) vel=100;
            m_plan_->m_behaviour_.setTarget(target,vel,MoveWithBall);
            if(turnmode)
                m_plan_->m_behaviour_.setOrienation(target_orientation.radian_,MAXW,AvoidObs,1);
            else
                m_plan_->m_behaviour_.setOrienation(target_orientation.radian_,MAXW,AvoidObs,-1);
            isBackObs=true;
        }

    }

    /// 晃开了障碍物，或者本不存在障碍物
    else
    {
        isBackObs=false;
        /// 如果是重新获得球权，则必须传球，指向助攻或防守位置，若不是重新获得球权则指向球门中点
        if((!world_model_->RegainBall_||world_model_->DribbleState_.satrt_point_.x_<-200)&&world_model_->CurActiveNotGoalieNums_>1)
            target_ori=assist_pt_.angle(target).radian_;
        else
            target_ori=field_.oppGoal_[GOAL_MIDDLE].angle(target).radian_;

        if((world_model_->CurActiveNotGoalieNums_==1||world_model_->RegainBall_)&&!world_model_->field_info_.isOurField(robot_pos))
        {
            target=robot_pos;
            target_ori=field_.oppGoal_[GOAL_MIDDLE].angle(target).radian_;
            m_plan_->m_behaviour_.setTarget(target,MAXVEL,/*AvoidObs*/TurnForShoot);
            m_plan_->m_behaviour_.setOrienation(target_ori.radian_,MAXW,/*AvoidObs*/TurnForShoot);
        }
        else if((!world_model_->RegainBall_||world_model_->DribbleState_.satrt_point_.x_<-200)&&world_model_->CurActiveNotGoalieNums_>1)
        {
            if(world_model_->DribbleState_.satrt_point_.x_<-200&&m_plan_->isNullInTrap(world_model_->Obstacles_,robot_pos,0,50,75,0,300) && world_model_->CurActiveNotGoalieNums_==2
                    &&world_model_->DribbleState_.is_dribble_)
            {
                target=target;
                target_ori=field_.oppGoal_[GOAL_MIDDLE].angle(target).radian_;
                m_plan_->m_behaviour_.setTarget(target,MAXVEL,AvoidObs);
                m_plan_->m_behaviour_.setOrienation(target_ori.radian_,MAXW,AvoidObs);
            }
            else
            {
                target=robot_pos;
                target_ori=assist_pt_.angle(target).radian_;
                m_plan_->m_behaviour_.setTarget(target,MAXVEL,AvoidObs);
                m_plan_->m_behaviour_.setOrienation(target_ori.radian_,MAXW,AvoidObs);
            }
        }
        /// 因为机器人带球，运动受到约束，此时的运动将分为三个阶段
        else
        {
            if(target.x_<50)
                target.x_=50;
            target_orientation = target.angle(robot_pos);
            if(fabs((robot_ori-target_orientation).degree())>30&&target.distance(robot_pos)>30)
            {
                if(m_plan_->rotateMode(world_model_->Opponents_,robot_pos,robot_ori,target_orientation))
                    m_plan_->m_behaviour_.setOrienation(target_orientation.radian_,MAXW,/*AvoidObs*/TurnForShoot_Robot,1);
                else
                    m_plan_->m_behaviour_.setOrienation(target_orientation.radian_,MAXW,/*AvoidObs*/TurnForShoot_Robot,-1);
                m_plan_->m_behaviour_.setTarget(robot_pos,MAXVEL,TurnForShoot_Robot);
            }
            else if(target.distance(robot_pos)>30)
            {
                m_plan_->move2Positionwithobs(target,MAXVEL);
            }
            else
            {
                m_plan_->m_behaviour_.setTarget(target,MAXVEL,/*AvoidObs*/TurnForShoot_Robot);
                m_plan_->m_behaviour_.setOrienation(target_ori.radian_,MAXW,/*AvoidObs*/TurnForShoot_Robot);
            }
        }
    }
}

/// \brief 抓球动作
void
ActiveRole::activeCatchBall()
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    DPoint ball_pos = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    double dis2b = robot_pos.distance(ball_pos);
    if(world_model_->BallInfoState_ == NOTSEEBALL)
    {
        m_plan_->m_behaviour_.clear();
        return;
    }
    if(field_.isInInterField(ball_pos))
    {
        if(field_.isOppGoal(ball_pos))
        {
            double thetaofr2b = ball_pos.angle(robot_pos).radian_;
            m_plan_->m_behaviour_.setTarget(ball_pos,100,CatchBall_slow);
            m_plan_->m_behaviour_.setOrienation(thetaofr2b,3,CatchBall_slow);
        }
        else if(!field_.isOurPenalty(ball_pos)) //if ball is in our penalty area, then catch the ball quickly
            m_plan_->catchBall();
        else
        {
            if(dis2b < 200)
                m_plan_->catchBallSlowly(); //max speed is 1.5m/s
            else
                m_plan_->catchBall();
        }
    }
    else
        m_plan_->m_behaviour_.clear();
}

/// \brief 踢球动作
/// \param 可微调的力量参数
void
ActiveRole::triggerShoot(double delta_force)
{
    kick_enable_ = true;
    kick_force_ = calculateKickforce(field_.oppGoal_[GOAL_MIDDLE],SHOOT);
}

/// \brief  根据踢球模式（平传or挑射）确定踢球力量
/// \param  目标点位置，踢球模式
/// \return 踢球力量
double
ActiveRole::calculateKickforce(DPoint target, int kick_mode)
{
    DPoint robot_pos = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    DPoint target_pos = target;
    double dis = robot_pos.distance(target_pos)/100.0;
    double strength = 0.0;
#ifdef SIMULATION
    if(kick_mode==PASS)
    {
        if(dis<=1.5)
            strength = 4;
        else if(dis>1.5&&dis<=3.0)
            strength = 6;
        else if(dis>3.0&&dis<5.0)
            strength = 8;
        else
            strength = 9;
    }

    else
    {
        //*************************nubot4******************************//
        DPoint target_point = DPoint(FIELD_XLINE1,-100);
        double target_height = 70;
        DPoint ball_pos = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
        double dis_ball2target = target_point.distance(ball_pos);
        if(4 * dis_ball2target  > 3 * target_height)
        {
            //strength = 5 * dis_ball2target/ sqrt((24*dis_ball2target -16 * target_height)/(9.8*100));
            //strength = 5* dis_ball2target /100*sqrt(9.8*100/(24*dis_ball2target-18*target_height)); //gc_equation
            strength = 100* dis_ball2target * sqrt(9.8 / (96 * dis_ball2target - 128 * target_height)); //zdx_equation
            ROS_INFO("strength:[%f]", strength);
        }
        else
        {
            strength = dis_ball2target / 0.5  * 1.25;
            std::cout<<"Attention, cannot fly to target_height"<<std::endl;
        }
        strength =  strength / 100; /// cm 2 m.because in simulation the unit is m and in our code the unit is cm

//        std::cout<<"strength for shoot "<<strength<<std::endl;
    }
#else
    if(kick_mode==PASS)
    {
        if(dis<=4.0)
            strength = Pass_in_3m;
        else if(dis>4.0&&dis<6.0)
            strength = Pass_3m_5m;
        else
            strength = Pass_far_5m;
        strength += Gain_power_pass;
        std::cout<<"strength for pass "<<strength<<std::endl;
    }
    else
    {
        static double prev_strength = 16.0;
        static double prev_dis = 5.0;
        if(dis <= 5.0)
        {
           strength = 20.0;
           prev_dis = dis;
        }
        else if (dis <= 8.5 && dis >= 5.5)
        {
            strength = P1*pow(dis,6)+P2*pow(dis,5)+
                   P3*pow(dis,4)+P4*pow(dis,3)+
                   P5*pow(dis,2)+P6*dis+P7;
            prev_strength = strength;
            prev_dis = dis;
        }
        else if (dis > 5.0 && dis < 5.5 && prev_dis >= 5.5)
        {
            strength = prev_strength - 0.6;
        }
        else if (dis > 5.0 && dis < 5.5 && prev_dis <= 5.0)
        {
            strength = prev_strength;
        }
        else if (dis > 8.5 && dis <= 11)
            strength = 0.8 * dis +14.4;
        else
            strength = 23.2;

        strength += Gain_power_shoot;
        std::cout<<"strength for shoot "<<strength<<std::endl;
    }
#endif
    return strength;
}

//计算机器人的射门意愿，由是否有障碍物、与球门的距离、与球门的夹角共同决定 - zhong zhengyu
double
ActiveRole::calculateShootingWilling(DPoint robot_pos)
{
    double shooting_willing = 0;
    double willing = 0;
    Angle  robot2goal   = field_.oppGoal_[GOAL_MIDDLE].angle(robot_pos);
    bool   isNullrobot  = m_plan_->isNullInTrap(world_model_->Opponents_,robot_pos,robot2goal,50,80,0,150);
    float  leftOppgoalCorner  =  field_.oppGoal_[GOAL_UPPER].angle(robot_pos).radian_;
    float  rightOppgoalCorner =  field_.oppGoal_[GOAL_LOWER].angle(robot_pos).radian_;
    float leftOppgoaldst = field_.oppGoal_[GOAL_UPPER].distance(robot_pos);
    float rightOppgoaldst = field_.oppGoal_[GOAL_LOWER].distance(robot_pos);

    if(isNullrobot)
    {
        //计算机器人与球门的距离，以该距离处理得到willing
        willing = (2700 - (leftOppgoaldst + rightOppgoaldst))/100;
        shooting_willing = willing;

        //计算机器人与球门的夹角，以该夹角处理得到willing
        willing = (leftOppgoalCorner - rightOppgoalCorner)*180/3.1415926535 * 8;
        shooting_willing += willing;

        return shooting_willing;
    }
    else
        return 0.0;

}
