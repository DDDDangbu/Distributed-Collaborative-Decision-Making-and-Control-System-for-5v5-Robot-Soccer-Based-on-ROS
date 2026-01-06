#include "nubot_control/world_model_info.h"
using namespace nubot;

World_Model_Info::World_Model_Info()
{
    RobotInfo_.resize(OUR_TEAM);
    BallInfo_.resize(OUR_TEAM);
    AgentID_ = 0;
    CurActiveRobotNums_ = 0;
    PreActiveRobotNums_ = 0;
    PreActiveNotGoalieNums_ = 0;
    CurActiveNotGoalieNums_ = 0;
    BallInfoState_ = NOTSEEBALL;
    RegainBall_ = false;
    CoachInfo_.MatchMode = STOPROBOT;
    CoachInfo_.MatchType = STOPROBOT;
    IsMoveForStartCommand_ =false;
    lastBallPosition_ = DPoint(-650,0);
    NoSeeBallNums_ = 0;
    KickSelection_ = NoKick;
    static_pass_time_ = 0;
    static_ball_dis_  = 0;
    clearPassState(true);
    pass_state_.reset();
    fuseOpps_.reserve(10);
}

void World_Model_Info::update(const bool & ball_holding)
{
    fuseBall();
    fuseOpp();
    /// 关于传接球（！待修改，不明白什么意思）
    pass_state_.update();
    if(BallInfoState_ != NOTSEEBALL)
    {
        lastBallPosition_ = BallInfo_[AgentID_-1].getGlobalLocation();
        NoSeeBallNums_ = 0;
    }
    else
        NoSeeBallNums_++;

    caculateActiveRobots();
    checkDribble(ball_holding);
    isMoveForStartCommand();
}

/// \brief 计算场地上活跃的机器人数目
void World_Model_Info::caculateActiveRobots()
{
    PreActiveRobotNums_     = CurActiveRobotNums_;
    PreActiveNotGoalieNums_ = CurActiveNotGoalieNums_;
    CurActiveNotGoalieNums_ = CurActiveRobotNums_ = 0;
    for(std::size_t i = 1 ; i < OUR_TEAM ; i++)
        CurActiveNotGoalieNums_ += RobotInfo_[i].isValid();
    CurActiveRobotNums_ = CurActiveNotGoalieNums_ + RobotInfo_[0].isValid();
}

/// \brief 函数的功能给是根据机器人的带球状态判断机器人是否是重新获得球权
/// \param ball_holding为从nubot_hwcontroller返回的服务状态
void World_Model_Info::checkDribble(const bool & ball_holding)
{
    static int DribbleCount = 0;
    /// 传球计时，这个时候虽然没有机器人带球，但是任然掌握球权?
    static int PassCount = 0;
    bool   pre_dribble   = DribbleState_.is_dribble_;
    DPoint robot_pos     = RobotInfo_[AgentID_-1].getLocation();
    DPoint ball2robot    = DPoint(BallInfo_[AgentID_-1].getRealLocation());

    #ifdef SIMULATION
    bool dribblecheck = ball_holding;
    #else
    /// 以前的带球是双重判断，即视觉加电位计，取消前向视觉后，不再使用视觉判断，直接取底层传回的带球状态
    bool dribblecheck = ball_holding;
    #endif

    DribbleState_.update(dribblecheck,robot_pos,ball2robot);
    /// 只有在自己得到本方队员的传球时，才能有效射门
     if(DribbleState_.is_dribble_ && !pre_dribble)
     {
         if(lastDribble)
             RegainBall_ = true;
         else
             RegainBall_ = false;
     }
     if(DribbleState_.is_dribble_)
         DribbleCount = 0;
     else if(!DribbleState_.is_dribble_)
         DribbleCount++;
     if(DribbleCount > 10)
         RegainBall_ = false;

     /// 更新dribble_状态；
     #ifdef SIMULATION
     RobotInfo_[AgentID_-1].setDribbleState(ball_holding);
     #else
     RobotInfo_[AgentID_-1].setDribbleState(DribbleState_.is_dribble_);
     #endif

     /// 判断对方是否拥有球权，通过记录对方的带球时间来实现
     checkOppHoldBall();

     /// 判断我方是否拥有球权
     IsOurDribble_ = false;
     for(int i = 0; i <OUR_TEAM;i++)
     {
         if(RobotInfo_[i].isValid() && RobotInfo_[i].getDribbleState() && i!=AgentID_-1)
         {
             IsOurDribble_ =true;
             lastDribble=true;
             PassCount=0;
         }
     }
/// zzq 190622
     if(CoachInfo_.MatchMode!=STARTROBOT)
     {
         RegainBall_ = false;
         lastDribble = false;
     }
}


/// 检测机器人是否能够移动，即发球时候的状态
void World_Model_Info::isMoveForStartCommand()
{
    /// 对于点球来说，end-game penalty不需要用到此函数; in-game penalty需要
    static bool is_start = false;
    static int wait_time = 0;
    static ros::Time start_time = ros::Time::now();
    static DPoint ball_pos = BallInfo_[AgentID_-1].getGlobalLocation();

    /// 表示是否重新点击开始命令,
    if(CoachInfo_.MatchMode!=STARTROBOT)
        is_start = false;
    /// STOPROBOT命令之后不能移动。
    if(CoachInfo_.MatchMode==STOPROBOT)
        IsMoveForStartCommand_ = false;

    if(CoachInfo_.MatchMode==STARTROBOT)
    {
        if(CoachInfo_.MatchType==OUR_KICKOFF    ||  CoachInfo_.MatchType==OUR_THROWIN       ||
           CoachInfo_.MatchType==OUR_GOALKICK   ||  CoachInfo_.MatchType==OUR_CORNERKICK    ||
           CoachInfo_.MatchType==OUR_FREEKICK   ||  CoachInfo_.MatchType==DROPBALL || CoachInfo_.MatchType == STOPROBOT)
            IsMoveForStartCommand_  = true;
        else if(CoachInfo_.MatchType==OPP_KICKOFF   || CoachInfo_.MatchType==OPP_THROWIN    ||
                CoachInfo_.MatchType==OPP_GOALKICK  || CoachInfo_.MatchType==OPP_CORNERKICK ||
                CoachInfo_.MatchType==OPP_FREEKICK  || CoachInfo_.MatchType==OUR_PENALTY    ||
                CoachInfo_.MatchType==OPP_PENALTY   )
        {
            IsMoveForStartCommand_  = false;
            if(!is_start)
            {
                is_start=true;
                start_time = ros::Time::now();
                ball_pos =  BallInfo_[AgentID_-1].getGlobalLocation();
            }

            ros::Duration duration = ros::Time::now() - start_time;
            if(CoachInfo_.MatchType==OUR_PENALTY    || CoachInfo_.MatchType==OPP_PENALTY )
                wait_time = PENALTY_WAIT_SECS;
            else
                wait_time = WAIT_SECS;

            if(duration.toSec()> wait_time)
                IsMoveForStartCommand_  = true;
            else if(BallInfoState_!=NOTSEEBALL)
            {
                if(BallInfo_[AgentID_-1].getGlobalLocation().distance(ball_pos) > 75)
                    IsMoveForStartCommand_  = true;
            }
        }
    }
}

void World_Model_Info::clearPassState(bool isclearId)
{
    if(isclearId)
    {
      catch_ID_= -1;
      pass_ID_ = -1;
     }
    is_static_pass_  = false;
    is_dynamic_pass_ = false;
    is_passed_out_   = false;
}

/// \brief 距离球最近的机器人看到的足球位置为最后的位置
bool World_Model_Info::fuseBall()
{
    float distance_min = 2000.0;
    float distance = distance_min;
    int ballNumber_=-1;

    for(int i=0;i<OUR_TEAM;i++)
        if(BallInfo_[i].isLocationKnown() && RobotInfo_[i].isValid())
        {
            distance=BallInfo_[i].getGlobalLocation().distance(RobotInfo_[i].getLocation());
            if(distance<distance_min)
            {
                distance_min=distance;
                ballNumber_=i;
            }
        }
    if(ballNumber_==-1)
        return false;                        //没有机器人看到球
    else
    {
        fuseBallInfo_=BallInfo_[ballNumber_];
        return true;
    }
}

/// \brief 融合结果为距离足球最近的机器人看到的对方机器人位置
bool World_Model_Info::fuseOpp()
{
    bool is_valid_opp = true;
    DPoint fuseBallPos = fuseBallInfo_.getGlobalLocation();

    fuseOpps_.clear();
    for(DPoint pos : Opponents_)
    {
        is_valid_opp = true;
        if((!field_info_.isOppGoal(pos)) && !(pos.x_==0.0 && pos.y_==0.0)
                && !(pos.distance(fuseBallPos)<150)
                && field_info_.isInInterField2(pos, -150, -100))
        {
            for(DPoint pt : fuseOpps_)
            {
                if(pos.distance(pt) < COMBINE_OPP_DIS)
                {
                    is_valid_opp = false;
//                    ROS_INFO("DISTACE LESS THAN 50");
                }
            }

            if(is_valid_opp)
            {
                //ROS_INFO("world_model_info: opp:%f %f",pos.x_, pos.y_);
                fuseOpps_.push_back(pos);
            }
        }
    }
    if(fuseOpps_.size()>0)
        return true;
    else
        return false;
}

/// \brief 判断对方是否获得球权
void World_Model_Info::checkOppHoldBall()
{
    static int HoldCount=0;
    for(int i=0;i<fuseOpps_.size();i++)
    {
        if(fuseBallInfo_.getGlobalLocation().distance(fuseOpps_[i])<DISTANCE_OPP2BALL)
        {
            HoldCount++;
            break;
        }
        else if(i==fuseOpps_.size()-1)
            HoldCount--;
    }
    if(HoldCount>TIME_DRIBBLE||
            CoachInfo_.MatchMode==OPP_KICKOFF   || CoachInfo_.MatchMode==OPP_THROWIN    ||
            CoachInfo_.MatchMode==OPP_GOALKICK  || CoachInfo_.MatchMode==OPP_CORNERKICK ||
            CoachInfo_.MatchMode==OPP_FREEKICK  || CoachInfo_.MatchMode==OUR_PENALTY    ||
            CoachInfo_.MatchMode==OPP_PENALTY   || CoachInfo_.MatchMode==DROPBALL)
    {
        IsOppHoldball_=true;
        lastDribble=false;
        HoldCount=0;
    }
    else if(HoldCount<-TIME_LOSE)
    {
        IsOppHoldball_=false;
        HoldCount=0;
    }
}
