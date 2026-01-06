#include <nubot_control/nubot_control.h>
using namespace std;
using namespace nubot;

NuBotControl::NuBotControl(int argc, char **argv): match_mode_(0), pre_match_mode_(0),
    shootflg_(false), isPenalty_(false), isObstacle_(false), isNewpassing_(true),
    shootcnt_(0),shootPos_(SHOOT), ballInitPos_(DPoint(-1000000,-1000000))
{
    const char * environment;
    ROS_INFO("initialize control process");

#ifdef SIMULATION
    std::string robot_name = argv[1];
    std::string num = robot_name.substr(robot_name.size()-1);
    //std::string robot_prefix = robot_name.substr(0,robot_name.size()-1);
    environment = num.c_str();
    ROS_FATAL("robot_name:%s",robot_name.c_str());
    nh_ = boost::make_shared<ros::NodeHandle>(robot_name);
#else
    nh_ = boost::make_shared<ros::NodeHandle>();
    /// 读取机器人标号，并赋值。在.bashrc中输入export AGENT=1，2，3，4，等等；
    if((environment = getenv("AGENT"))==NULL)
    {
        ROS_ERROR("this agent number is not read by robot");
        return ;
    }
#endif
    /// 向hwcontroller发布topic（单机器人动作信息）
    action_cmd_pub_     = nh_->advertise<nubot_common::ActionCmd>("nubotcontrol/actioncmd",1);
    /// 向world_model发布topic（多机器人策略信息）
    strategy_info_pub_  = nh_->advertise<nubot_common::StrategyInfo>("nubotcontrol/strategy",10);

    /// 向omnivision发送是否重定位的topic   2018 @lx
    isRelocation_pub_   = nh_->advertise<nubot_common::Relocation>("nubotcontrol/isrelocation", 1);
    /// 订阅来自world_model发布的topic（其他机器人共享的信息）
    worldmodelinfo_sub_ = nh_->subscribe("worldmodel/worldmodelinfo", 1, &NuBotControl::update_world_model_info, this);
    /// 守门员独有
    goalie_ball3d_subL_ = nh_->subscribe("goalie/ballinfo3dL",1, &NuBotControl::goalieBall3dCallbackL, this);
    goalie_ball3d_subR_ = nh_->subscribe("goalie/ballinfo3dR",1, &NuBotControl::goalieBall3dCallbackR, this);
    /// 订阅来自Kinect发布的topic（障碍物信息）
    obsinfo3d_sub_      = nh_->subscribe("nubot/kinect/obstaclesinfo3d",1,&NuBotControl::obstaclesInfo3dCallback, this);
#ifdef SIMULATION
    ballHolding_sub_    = nh_->subscribe("ballisholding/BallIsHolding",1, &NuBotControl::ballHoldingCallback, this);
#else
    /// 订阅来自hwcontroler发布的信息（带球状态信息）
    ballHolding_sub_    = nh_->subscribe("/nubotdriver/ballisholding",1, &NuBotControl::ballHoldingCallback, this);
#endif
    /// 控制周期
    control_timer_      = nh_->createTimer(ros::Duration(0.015),&NuBotControl::loopControl,this);
    reconfigureServer_.setCallback(boost::bind(&NuBotControl::configure, this, _1, _2));

    /// 初始化一些重要的变量，这里将多个类（plan，subtargets，staticpass，strategy）的world_model都指向了nubot_control开辟的内存空间
    world_model_info_.AgentID_ = atoi(environment);
    world_model_info_.CoachInfo_.MatchMode = STOPROBOT;
    m_plan_.world_model_ =  & world_model_info_;
    m_plan_.m_subtargets_.world_model_ =  & world_model_info_;
    m_staticpass_.world_model_= & world_model_info_;
    m_strategy_ = new Strategy(world_model_info_,m_plan_);
    test_=new NubotTest(world_model_info_,m_plan_,m_strategy_->ActiveRole_);
    ballisHolding_=false;


    //////@hbx
    _isOpposite= false;
    ///subscribe the message about opposite
    isOpposite_          = nh_->subscribe("/omnivision/isOpposite",1,&NuBotControl::Opposite,this);

}

NuBotControl::~NuBotControl()
{
    /// 机器人停止运动
    m_plan_.m_behaviour_.move_action_ = No_Action;
    m_plan_.m_behaviour_.rotate_action_ = No_Action;
    setEthercatCommond();
    delete m_strategy_;
    delete test_;
}

/// \brief 动态参服务器
void NuBotControl::configure(const nubot_control::nubotcontrolConfig & config, uint32_t level)
{/*
   kp_ = config.kp;
   kalpha_ = config.kalpha;
   kbeta_  = config.kbeta;

   m_strategy_.m_plan_.kp =  kp_;
   m_strategy_.m_plan_.kalpha =  kalpha_;
   m_strategy_.m_plan_.kbeta  =   kbeta_;*/
}

/// \brief 订阅到世界模型信息后，更新
void NuBotControl::update_world_model_info(const nubot_common::WorldModelInfo & _world_msg)
{
    /// 更新自身与队友的信息，自身的策略信息记住最好不要更新，因为本身策略是从此节点发布的
    for(std::size_t i = 0 ; i < OUR_TEAM ; i++)
    {
        world_model_info_.RobotInfo_[i].setID(_world_msg.robotinfo[i].AgentID);
        /// 机器人的状态信息
        world_model_info_.RobotInfo_[i].setLocation(DPoint(_world_msg.robotinfo[i].pos.x,_world_msg.robotinfo[i].pos.y));
        world_model_info_.RobotInfo_[i].setHead(Angle(_world_msg.robotinfo[i].heading.theta));
        world_model_info_.RobotInfo_[i].setVelocity(DPoint(_world_msg.robotinfo[i].vtrans.x,_world_msg.robotinfo[i].vtrans.y));
        world_model_info_.RobotInfo_[i].setStuck(_world_msg.robotinfo[i].isstuck);
        world_model_info_.RobotInfo_[i].setKick(_world_msg.robotinfo[i].iskick);
        world_model_info_.RobotInfo_[i].setValid(_world_msg.robotinfo[i].isvalid);
        world_model_info_.RobotInfo_[i].setW(_world_msg.robotinfo[i].vrot);

        /// 信息是来源于队友，则要更新机器人策略信息
        if(world_model_info_.AgentID_ != i+1)
        {
            /// 静态传接球的站位选择信息
            world_model_info_.RobotInfo_[i].setTargetNum(1,_world_msg.robotinfo[i].targetNum1);
            world_model_info_.RobotInfo_[i].setTargetNum(2,_world_msg.robotinfo[i].targetNum2);
            world_model_info_.RobotInfo_[i].setTargetNum(3,_world_msg.robotinfo[i].targetNum3);
            world_model_info_.RobotInfo_[i].setTargetNum(4,_world_msg.robotinfo[i].targetNum4);
            world_model_info_.RobotInfo_[i].setpassNum(_world_msg.robotinfo[i].staticpassNum);
            world_model_info_.RobotInfo_[i].setcatchNum(_world_msg.robotinfo[i].staticcatchNum);

            world_model_info_.RobotInfo_[i].setDribbleState(_world_msg.robotinfo[i].isdribble);
            world_model_info_.RobotInfo_[i].setRolePreserveTime(_world_msg.robotinfo[i].role_time);
            world_model_info_.RobotInfo_[i].setCurrentRole(_world_msg.robotinfo[i].current_role);
            world_model_info_.RobotInfo_[i].setTarget(DPoint(_world_msg.robotinfo[i].target.x,_world_msg.robotinfo[i].target.y));
        }
    }
    /// 更新障碍物信息
    world_model_info_.Obstacles_.clear();
    for(nubot_common::Point2d point : _world_msg.obstacleinfo.pos )
        world_model_info_.Obstacles_.push_back(DPoint(point.x,point.y));
    world_model_info_.Opponents_.clear();
    for(nubot_common::Point2d point : _world_msg.oppinfo.pos )
        world_model_info_.Opponents_.push_back(DPoint(point.x,point.y));

    /// 更新足球物信息
    for(std::size_t i = 0 ; i < OUR_TEAM ; i++)
    {
        world_model_info_.BallInfo_[i].setGlobalLocation(DPoint(_world_msg.ballinfo[i].pos.x ,_world_msg.ballinfo[i].pos.y));
        world_model_info_.BallInfo_[i].setRealLocation(PPoint(Angle(_world_msg.ballinfo[i].real_pos.angle),_world_msg.ballinfo[i].real_pos.radius));
        world_model_info_.BallInfo_[i].setVelocity(DPoint(_world_msg.ballinfo[i].velocity.x,_world_msg.ballinfo[i].velocity.y));
        world_model_info_.BallInfo_[i].setVelocityKnown(_world_msg.ballinfo[i].velocity_known);
        world_model_info_.BallInfo_[i].setLocationKnown(_world_msg.ballinfo[i].pos_known);
        world_model_info_.BallInfo_[i].setValid(_world_msg.ballinfo[i].pos_known);
    }
    world_model_info_.BallInfoState_ = _world_msg.ballinfo[world_model_info_.AgentID_-1].ballinfostate;

    /// 更新的COACH信息
    world_model_info_.CoachInfo_.MatchMode =_world_msg.coachinfo.MatchMode;
    world_model_info_.CoachInfo_.MatchType =_world_msg.coachinfo.MatchType;

    world_model_info_.CoachInfo_.TestMode=_world_msg.coachinfo.TestMode;
    world_model_info_.CoachInfo_.pointA.x_=_world_msg.coachinfo.pointA.x;
    world_model_info_.CoachInfo_.pointA.y_=_world_msg.coachinfo.pointA.y;
    world_model_info_.CoachInfo_.pointB.x_=_world_msg.coachinfo.pointB.x;
    world_model_info_.CoachInfo_.pointB.y_=_world_msg.coachinfo.pointB.y;
    world_model_info_.CoachInfo_.angleA=_world_msg.coachinfo.angleA;
    world_model_info_.CoachInfo_.angleB=_world_msg.coachinfo.angleB;
    world_model_info_.CoachInfo_.id_A=_world_msg.coachinfo.idA;
    world_model_info_.CoachInfo_.id_B=_world_msg.coachinfo.idB;
    world_model_info_.CoachInfo_.kick_force=_world_msg.coachinfo.kickforce;

    /// 2018 @lx  每次当coach发送stop命令的时候，开启全局定位，发送start命令的时候机器人开始自定位。用这种机制完成重定位
    nubot_common::Relocation isRelocation;
    if(world_model_info_.CoachInfo_.MatchMode == STOPROBOT){
        isRelocation.isRelocation = true;

    }
    if(world_model_info_.CoachInfo_.MatchMode == STARTROBOT){
        isRelocation.isRelocation = false;

    }

    //        isRelocation_pub_.publish(isRelocation);
    /// 2018 @lx

    /// 为了减少通信量，coach中的很多值是复用的，比如最大速度和角速度是可以通过angleA和angleB来动态调整的
    if(_world_msg.coachinfo.MatchMode!=TEST)
    {
        maxvel_=_world_msg.coachinfo.angleA;
        maxw_=_world_msg.coachinfo.angleB;
    }
    else
    {
        maxvel_ = 200;
        maxw_ = 5;
    }

    /// 更新传球信息（pass_cmds_为从所有机器人得到的传球信息的融合）！传球机制待修改
    world_model_info_.pass_cmds_.catchrobot_id   = _world_msg.pass_cmd.catch_id;
    world_model_info_.pass_cmds_.passrobot_id    = _world_msg.pass_cmd.pass_id;
    world_model_info_.pass_cmds_.isvalid         = _world_msg.pass_cmd.is_valid;
    world_model_info_.pass_cmds_.is_dynamic_pass = _world_msg.pass_cmd.is_dynamic_pass;
    world_model_info_.pass_cmds_.is_static_pass  = _world_msg.pass_cmd.is_static_pass;
    world_model_info_.pass_cmds_.is_passout      = _world_msg.pass_cmd.is_passout;
    world_model_info_.pass_cmds_.pass_pt         = DPoint(_world_msg.pass_cmd.pass_pt.x,_world_msg.pass_cmd.pass_pt.y);
    world_model_info_.pass_cmds_.catch_pt        = DPoint(_world_msg.pass_cmd.catch_pt.x,_world_msg.pass_cmd.catch_pt.y);

    /// 表示正在传球过程中，因此其他机器人必须根据该状态确定角色，pass_state_为机器人自己根据自身感知计算求得的传球状态
    if(world_model_info_.pass_cmds_.isvalid && world_model_info_.pass_cmds_.is_passout && !world_model_info_.pass_state_.is_passing_)
    {
        world_model_info_.pass_state_.set(world_model_info_.pass_cmds_.passrobot_id,
                                          world_model_info_.pass_cmds_.catchrobot_id,
                                          world_model_info_.pass_cmds_.pass_pt,
                                          world_model_info_.pass_cmds_.catch_pt,
                                          world_model_info_.pass_cmds_.is_dynamic_pass,
                                          world_model_info_.pass_cmds_.is_static_pass);
        std::cout<<" pass_done_com: "<<std::endl;
        /// 球已经传出之后，所有的准备传球信息清空
        world_model_info_.clearPassState(true);
    }

    /// 这个先如此改，之后将所有数据用world_model_进行传递
    m_strategy_->goalie_strategy_.robot_info_    = _world_msg.robotinfo[world_model_info_.AgentID_-1];
    m_strategy_->goalie_strategy_.ball_info_2d_  = _world_msg.ballinfo[world_model_info_.AgentID_-1];
}

/// \brief 球的三维信息,用于守门员角色
void NuBotControl::goalieBall3dCallbackL(const nubot_common::BallInfo3d  &_BallInfo_3d)
{
    m_strategy_->goalie_strategy_.setBallInfo3dRel( _BallInfo_3d );
}
void NuBotControl::goalieBall3dCallbackR(const nubot_common::BallInfo3d  &_BallInfo_3d)
{
    m_strategy_->goalie_strategy_.setBallInfo3dRel( _BallInfo_3d );
}

/// \brief Kinect消息回调
void NuBotControl::obstaclesInfo3dCallback(const nubot_common::ObstaclesInfo3d  &_ObstaclesInfo_3d)
{
    m_strategy_->ActiveRole_.Obstacles3d.pos = _ObstaclesInfo_3d.pos;
    m_strategy_->ActiveRole_.Obstacles3d.pos_known_3d = _ObstaclesInfo_3d.pos_known_3d;
}

/// \brief 带球状态回调函数
void NuBotControl::ballHoldingCallback(const nubot_common::BallIsHolding & _ballisholding)
{
    ballisHolding_= _ballisholding.BallIsHolding;
}

/// \brief 主要的控制框架位于这里，非常重要的部分
void NuBotControl::loopControl(const ros::TimerEvent& event)
{
    /// 当前比赛模式
    match_mode_ = world_model_info_.CoachInfo_.MatchMode;
    /// 上一个比赛模式（记录上一个非start或stop命令是为了确定start后的不同动作选择）
    pre_match_mode_ = world_model_info_.CoachInfo_.MatchType;
    /// 转存该机器人的部分状态信息
    robot_pos_  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getLocation();
    robot_ori_  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getHead();
    ball_pos_   = world_model_info_.BallInfo_[world_model_info_.AgentID_-1].getGlobalLocation();
    ball_vel_   = world_model_info_.BallInfo_[world_model_info_.AgentID_-1].getVelocity();
    /// 更新当前的机器人的带球状态信息
    world_model_info_.update(ballisHolding_);
    /// 计算各个角色的动态站位点
    m_strategy_->calPassPositions();
    /// 更新plan里面的机器人位置等信息
    m_plan_.update();

    /// 初始化足球位置
    if(world_model_info_.BallInfoState_ != NOTSEEBALL  && ballInitPos_.x_ == -1000000)
        ballInitPos_ = world_model_info_.BallInfo_[world_model_info_.AgentID_-1].getGlobalLocation();

    /// 这是正式比赛用的程序
    if(match_mode_!=TEST)
    {
        ///////@hbx
        if(match_mode_ == STOPROBOT || _isOpposite )
        {
            /// 一系列参数的置位
            world_model_info_.static_pass_time_ = 100;
            world_model_info_.static_ball_dis_  = 10000;
            world_model_info_.is_static_pass_ = false;
            world_model_info_.is_dynamic_pass_= false;
            m_strategy_->ActiveRole_.isturn_=true;
            isPenalty_=false;
            isNewpassing_=true;
            isObstacle_=false;
            shootflg_ =false;
            strength_ =0;
            shootcnt_ =0;

            start_time_ = ros::Time::now();
            ballInitPos_ = DPoint(-1000000,-1000000);
            world_model_info_.clearPassState(true);
            m_strategy_->selected_action_ = No_Action;
            m_strategy_->RoleAssignment_.last_activeID=0;
            m_strategy_->RoleAssignment_.last_activeUtility=0;
            m_strategy_->ActiveRole_.clearActiveState();
            m_plan_.m_behaviour_.clear();
        }
        /// 机器人在开始之前的跑位. 开始静态传接球的目标点计算
        else if(match_mode_ > STOPROBOT && match_mode_ <= DROPBALL)
            positioning();
        else if(match_mode_==PARKINGROBOT)
            parking();
        /// 机器人正式比赛了，进入start之后的机器人状态
        else
        {
            /// 以下为点球部分
            if(isPenalty_)
                penaltyStart();
            else
            {
#ifndef END_GAME_PENALTY
                /// 正常的比赛流程
                normalGame();
#endif
            }
        }

        /// 抓球部分
        handleBall();
        /// 传球或者射门
        kickBall();
        /// 静态传球以及动态传球，都应该在这里进行赋值，以保证接球机器人能够切换到主攻
        pubStrategyInfo();
        /// 发送速度指令
        setEthercatCommond();
    }
}

void NuBotControl::normalGame()
{
    /// 如果对方发球，WAIT_SECS秒之内不能动
    if(!world_model_info_.IsMoveForStartCommand_)
    {
        //std::cout<<"stop!: "<< m_strategy_->selected_role_<<std::endl;
        world_model_info_.clearPassState(true);
        m_strategy_->selectRole();
        m_plan_.m_behaviour_.clear();
        m_strategy_->selected_action_ = No_Action;
        m_strategy_->ActiveRole_.currentstate_ = No_Action;
        world_model_info_.static_ball_dis_ = 0;
    }
    /// 己方发球或者是WAIT_SECS秒之后，是可以动的，但是需要调整角色.
    else
    {
        ros::Duration duration = ros::Time::now() - start_time_;
        world_model_info_.static_pass_time_ = duration.toSec();
        if(world_model_info_.BallInfoState_ != NOTSEEBALL )
        {
            double distance = ballInitPos_.distance(world_model_info_.BallInfo_[world_model_info_.AgentID_-1].getGlobalLocation());
            if(distance > world_model_info_.static_ball_dis_)
                world_model_info_.static_ball_dis_ = distance;
        }
        /// 正常的踢出了足球之后的状态，踢球机器人动作
        bool is_passing = world_model_info_.pass_state_.is_passing_;
        bool is_me_pass = world_model_info_.pass_state_.pass_id_ == world_model_info_.AgentID_;
        bool is_someone_catch = world_model_info_.pass_state_.catch_id_ != -1 ;

        /// 球已经传出
        if(is_passing && is_me_pass && is_someone_catch)
        {
            std::cout<<"passedout!: "<< m_strategy_->selected_role_<<std::endl;
            m_strategy_->selectRole();
            m_plan_.m_behaviour_.clear();

            /// 判断传球机器人是否阻挡了接球机器人的射门角度，每次传球只计算一次
            if(isNewpassing_)
            {
                std::vector<DPoint> pts;                        
                pts.reserve(3);
                pts.push_back(ball_pos_);
                pts.push_back(field_.oppGoal_[GOAL_LOWER]);
                pts.push_back(field_.oppGoal_[GOAL_UPPER]);
                isObstacle_=pnpoly(pts,world_model_info_.pass_pt_);
            }
            isNewpassing_=false;
            if(!shootflg_)
            {
                /// 如果传球机器人阻挡了接球机器人的射门角度，有撤开的动作
                if(isObstacle_)
                {
                    Angle thetaofr2b = ball_pos_.angle(robot_pos_);
                    DPoint target;
                    ROS_INFO("pass: %d  %f",world_model_info_.AgentID_,world_model_info_.pass_state_.pass_pt_.y_);
                    DPoint vector_pass = world_model_info_.pass_state_.catch_pt_ - world_model_info_.pass_state_.pass_pt_;
                    if(vector_pass.angle().radian_ < 0)
                        target = DPoint(robot_pos_.x_,world_model_info_.pass_state_.pass_pt_.y_+100);
                    else
                        target = DPoint(robot_pos_.x_,world_model_info_.pass_state_.pass_pt_.y_-100);
                    if(fabs(target.y_) > 500 * WIDTH_RATIO)
                        target.y_ = 500.0*WIDTH_RATIO*target.y_ /fabs(target.y_);//500
                    if(fabs(target.y_) < 450 *WIDTH_RATIO)
                        target.y_ = 450.0*WIDTH_RATIO*target.y_ /fabs(target.y_);//450
                    m_plan_.move2Positionwithobs(target,MAXVEL,false);
                    m_plan_.m_behaviour_.setOrienation(thetaofr2b.radian_,MAXW,AvoidObs);
                }
                else
                    m_strategy_->selectAction();
            }
        }
        /// 传球结束，清除置位
        else if((world_model_info_.static_pass_time_ > WAIT_SECS || world_model_info_.static_ball_dis_ >100) && world_model_info_.is_static_pass_)
        {
            std::cout<<"passdone!: "<< m_strategy_->selected_role_<<std::endl;  //daiwei 3.13
            world_model_info_.clearPassState(false);
        }
        /// 还在发球过程中,或者是正式比赛过程中，则机器人需要考虑传球时候的角色分配和动作
        else
        {
            m_strategy_->selectRole();
            bool isdynamic_start =  world_model_info_.pass_cmds_.is_dynamic_pass &&
                    world_model_info_.pass_cmds_.catchrobot_id!=world_model_info_.AgentID_&&
                    world_model_info_.pass_cmds_.passrobot_id!=world_model_info_.AgentID_;
            bool isdynamic_out   =  (world_model_info_.pass_state_.is_static_pass_ || world_model_info_.pass_state_.is_dynamic_pass_) &&
                    world_model_info_.pass_state_.is_passing_ &&
                    world_model_info_.pass_state_.catch_id_!=world_model_info_.AgentID_ &&
                    world_model_info_.pass_state_.pass_id_!=world_model_info_.AgentID_;
            bool isOurDrbble = world_model_info_.IsOurDribble_ && ball_pos_.x_ >0;
            bool noActiveAndDefence = m_strategy_->selected_role_!=ACTIVE && m_strategy_->selected_role_!=PASSIVE;

            if(m_strategy_->selected_role_!=ACTIVE  && world_model_info_.is_static_pass_)
            {

                Angle ang = ball_pos_.angle(robot_pos_);
                m_plan_.m_behaviour_.setOrienation(ang.radian_,MAXW,Positioned_Static);
                // m_plan_.m_behaviour_.clear();
                m_strategy_->selected_action_ = Positioned_Static;
            }
            else if(noActiveAndDefence && world_model_info_.CurActiveNotGoalieNums_>2)
            {
                //                std::cout<<"dynamic!: "<< m_strategy_->selected_role_<<std::endl;
                if(m_strategy_->selected_role_ == MIDFIELD)
                    coorrect_target(isdynamic_start,isdynamic_out,isOurDrbble,robot_pos_,m_strategy_->MidfieldRole_.midfield_pt_);
                if(m_strategy_->selected_role_ ==ASSISTANT)
                    coorrect_target(isdynamic_start,isdynamic_out,isOurDrbble,robot_pos_, m_strategy_->AssistRole_.assist_pt_);
                m_strategy_->selectAction();
            }
            else
            {
                //std::cout<<"normalgame!: "<< m_strategy_->selected_role_<<std::endl;
                m_strategy_->selectAction();
            }
        }
    }
}

void NuBotControl::positioning()
{
    //    std::cout<<"Posintion!"<<std::endl;
    /// 开始静态站位的计算，并清空可能的运动参数
    m_staticpass_.staticReady_();
    m_plan_.m_behaviour_.clear();
    /// 站位标志置位
    for(int i=0;i<OUR_TEAM;i++)
        m_staticpass_.isAllocation_[i]=false;

    Angle  target_orientation = ball_pos_.angle(robot_pos_);
    DPoint target = m_staticpass_.target_;

    start_time_ = ros::Time::now();        //daiwei 3.12
    ballInitPos_ = world_model_info_.BallInfo_[world_model_info_.AgentID_-1].getGlobalLocation();
    m_strategy_->ActiveRole_.currentstate_  = No_Action;
    m_strategy_->selected_action_ = Positioned;
    world_model_info_.static_pass_time_ = 0;
    world_model_info_.static_ball_dis_  = 0;

    if(target.distance(robot_pos_)>LOCATIONERROR/2.0)
        m_plan_.move2Positionwithobs(target,target.distance(robot_pos_)*MAXVEL/500,true);
    if(fabs((robot_ori_-target_orientation).degree())>1)
        m_plan_.m_behaviour_.setOrienation(target_orientation.radian_,fabs((robot_ori_-target_orientation).radian_)*MAXW/(2*M_PI),Positioned_Static);

    /// 是我方开球
    if(match_mode_==OUR_KICKOFF || match_mode_==OUR_THROWIN || match_mode_==OUR_FREEKICK || match_mode_==OUR_GOALKICK || match_mode_==OUR_CORNERKICK)
    {
        world_model_info_.is_static_pass_ = true;
        world_model_info_.catch_ID_ = m_staticpass_.m_nCatchNumber_;
        world_model_info_.pass_ID_  = m_staticpass_.m_nPassNumber_;
        if(world_model_info_.catch_ID_ > 0 && world_model_info_.catch_ID_ <6)
            world_model_info_.catch_pt_ =  world_model_info_.RobotInfo_[world_model_info_.catch_ID_-1].getLocation();
        if(world_model_info_.pass_ID_ > 0 && world_model_info_.pass_ID_ <6)
            world_model_info_.pass_pt_ =  world_model_info_.RobotInfo_[world_model_info_.pass_ID_-1].getLocation();
        world_model_info_.is_passed_out_ = false;
        world_model_info_.KickSelection_ = KickBySelf;
    }
    else if(match_mode_==OPP_KICKOFF || match_mode_==OPP_THROWIN || match_mode_==OPP_FREEKICK || match_mode_==OPP_GOALKICK || match_mode_==OPP_CORNERKICK)
    {
        m_staticpass_.m_nCatchNumber_=-1;
        m_staticpass_.m_nPassNumber_=-1;
        world_model_info_.clearPassState(true);
        world_model_info_.KickSelection_ = KickByOpp;
    }
    else if(match_mode_==DROPBALL)
    {
        world_model_info_.static_pass_time_ = 100;
        world_model_info_.static_ball_dis_  = 10000;
        ballInitPos_ = DPoint(-1000000,-1000000);
        shootflg_ = false;
        shootcnt_ = 0;
    }
    else if(match_mode_==OUR_PENALTY||match_mode_==OPP_PENALTY)
        isPenalty_=true;
}

void NuBotControl::parking()
{
    static double parking_y=0;
    /// 从launch文件获得y值
    ros::param::get("~parking_y",parking_y);
    cout<<"PARKINGROBOT"<<endl;
    DPoint parking_target;
    float tar_ori=SINGLEPI_CONSTANT/2.0;
    parking_target.x_=120*world_model_info_.AgentID_;
    /// 守门员站在离球门最近的地方
    if(world_model_info_.AgentID_==1)
        parking_target.x_=700;
    parking_target.y_=parking_y;
    /// 停到目标点10cm附近就不用动了，只需调整朝向
    if(parking_target.distance(robot_pos_)>10)
        m_plan_.move2Positionwithobs(parking_target,100,true);
    if(fabs(angularnorm(robot_ori_.radian_-tar_ori))>10.0*SINGLEPI_CONSTANT/180.0)
        m_plan_.m_behaviour_.setOrienation(tar_ori,2,Positioned);
}

void NuBotControl::penaltyStart()
{
    if(pre_match_mode_ == OUR_PENALTY)
    {
        std::cout<<"OUR_PENALTY(in-game)!"<<std::endl;
        m_strategy_->process();
        if(m_strategy_->selected_role_ == ACTIVE)
        {
            char delta_force = 0;
            if(test_->TestShoot(world_model_info_.AgentID_ ,field_.oppGoal_[GOAL_MIDDLE], delta_force))
                isPenalty_ = false;
        }
        else
            m_plan_.m_behaviour_.clear();

#ifndef END_GAME_PENALTY
        /// 初始化，重新计算IsMoveForStartCommand_的值
        world_model_info_.IsMoveForStartCommand_ = false;
        world_model_info_.isMoveForStartCommand();
        /// 足球移动了或者时间超过10s则退出点球状态
        if(world_model_info_.IsMoveForStartCommand_)
            isPenalty_ = false;
#endif
    }
    else if(pre_match_mode_ ==OPP_PENALTY)
    {
        std::cout<<"OPP_PENALTY(in-game)!"<<std::endl;
        if(world_model_info_.AgentID_ ==1)
            m_strategy_->process();
        else
            m_plan_.m_behaviour_.clear();

#ifndef END_GAME_PENALTY
        world_model_info_.IsMoveForStartCommand_ = false;
        world_model_info_.isMoveForStartCommand();
        if(world_model_info_.IsMoveForStartCommand_)
            isPenalty_ = false;
#endif
    }
}

void NuBotControl::pubStrategyInfo()
{
    nubot_common::StrategyInfo strategy_info;
    strategy_info.header.stamp = ros::Time::now();
    strategy_info.AgentID     = world_model_info_.AgentID_;

    strategy_info.targetNum1  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getTargetNum(1);
    strategy_info.targetNum2  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getTargetNum(2);
    strategy_info.targetNum3  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getTargetNum(3);
    strategy_info.targetNum4  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getTargetNum(4);

    strategy_info.staticcatchNum  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getcatchNum();
    strategy_info.staticpassNum  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getpassNum();

    strategy_info.is_dribble  = world_model_info_.DribbleState_.is_dribble_;

    /// 机器人传球的命令
    strategy_info.pass_cmd.is_dynamic_pass = false;
    bool is_static_pass = world_model_info_.is_static_pass_;
    bool not_catch_robot = (world_model_info_.catch_ID_!= world_model_info_.AgentID_);
    bool not_pass_robot = (world_model_info_.pass_ID_!= world_model_info_.AgentID_) ;

    if( is_static_pass && not_catch_robot && not_pass_robot )
        world_model_info_.clearPassState(true);

    if(!is_static_pass)
    {
        if(m_strategy_->ActiveRole_.currentstate_ == TurnToPass && m_strategy_->selected_role_==ACTIVE)
            world_model_info_.is_dynamic_pass_ = true;
        else if(world_model_info_.static_pass_time_ > WAIT_SECS)
            world_model_info_.clearPassState(true);
        else
            world_model_info_.clearPassState(false);
    }
    strategy_info.pass_cmd.is_passout = world_model_info_.is_passed_out_;
    strategy_info.is_kickoff          = world_model_info_.is_passed_out_;
    strategy_info.pass_cmd.is_dynamic_pass =  world_model_info_.is_dynamic_pass_;
    strategy_info.pass_cmd.is_static_pass  =  world_model_info_.is_static_pass_;

    if(world_model_info_.catch_ID_ == -1)
        strategy_info.pass_cmd.catch_id = 255;
    else
        strategy_info.pass_cmd.catch_id = world_model_info_.catch_ID_;
    if(is_static_pass)
        strategy_info.pass_cmd.pass_id  = world_model_info_.pass_ID_;
    else
        strategy_info.pass_cmd.pass_id  = world_model_info_.AgentID_;

    world_model_info_.pass_pt_  =  world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getLocation();
    strategy_info.pass_cmd.pass_pt.x  = world_model_info_.pass_pt_.x_;
    strategy_info.pass_cmd.pass_pt.y  = world_model_info_.pass_pt_.y_;
    strategy_info.pass_cmd.catch_pt.x = world_model_info_.catch_pt_.x_;
    strategy_info.pass_cmd.catch_pt.y = world_model_info_.catch_pt_.y_;
    strategy_info.pass_cmd.is_valid = true;
    strategy_info.role   =  m_strategy_->selected_role_;
    strategy_info.action =  m_strategy_->selected_action_;
    strategy_info.role_time  = world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getRolePreserveTime();
    if(world_model_info_.is_passed_out_)
    {
        world_model_info_.pass_state_.set(world_model_info_.AgentID_,
                                          world_model_info_.catch_ID_,
                                          world_model_info_.pass_pt_,
                                          world_model_info_.catch_pt_,
                                          world_model_info_.is_dynamic_pass_,
                                          world_model_info_.is_static_pass_);
        world_model_info_.clearPassState(false);
    }

    if(world_model_info_.pass_state_.time_lock_ >160 && world_model_info_.pass_state_.is_passing_ &&
            world_model_info_.pass_state_.is_dynamic_pass_ &&
            world_model_info_.pass_state_.pass_id_ == world_model_info_.AgentID_ ) //180
    {
        strategy_info.pass_cmd.catch_id   = world_model_info_.pass_state_.catch_id_;
        strategy_info.pass_cmd.pass_id    = world_model_info_.pass_state_.pass_id_;
        strategy_info.pass_cmd.pass_pt.x  = world_model_info_.pass_state_.pass_pt_.x_;
        strategy_info.pass_cmd.pass_pt.y  = world_model_info_.pass_state_.pass_pt_.y_;
        strategy_info.pass_cmd.catch_pt.x = world_model_info_.pass_state_.catch_pt_.x_;
        strategy_info.pass_cmd.catch_pt.y = world_model_info_.pass_state_.catch_pt_.y_;
        strategy_info.pass_cmd.is_static_pass  = false;
        strategy_info.pass_cmd.is_dynamic_pass = world_model_info_.pass_state_.is_dynamic_pass_;
        strategy_info.pass_cmd.is_passout = world_model_info_.pass_state_.is_passing_;
        strategy_info.pass_cmd.is_valid = true;
    }
    strategy_info_pub_.publish(strategy_info);
}

void NuBotControl::handleBall()
{
    /// 是主攻机器人
    bool is_active = (m_strategy_->selected_role_ == ACTIVE) ;
    /// 静态传球接球机器人
    bool is_catch_static = (world_model_info_.static_pass_time_ < WAIT_SECS && world_model_info_.catch_ID_ == world_model_info_.AgentID_);
    /// 动态传球接球机器人
    bool is_catch_dynam = (world_model_info_.pass_cmds_.is_dynamic_pass && world_model_info_.pass_cmds_.catchrobot_id == world_model_info_.AgentID_);

    if(is_active || is_catch_static || is_catch_dynam)
        handle_enable_ = 1;
    else
        handle_enable_ = 0;
    /// 机器人堵转时，增加转速
    if(m_strategy_->ActiveRole_.stuckflg_)
        handle_enable_ = 2;
    /// 比赛停止，停止转动
    if(match_mode_!=STARTROBOT)
        handle_enable_ = 0;
}

void NuBotControl::kickBall()
{
    /// 重新获得球权，不能射门
    if(!world_model_info_.RegainBall_
            &&world_model_info_.CurActiveRobotNums_>1)
        shootPos_ =  PASS;
    /// 主攻转身传球
    else if(m_strategy_->selected_role_ == ACTIVE && m_strategy_->ActiveRole_.currentstate_==TurnToPass)
        shootPos_ =  PASS;
    /// 静态传球
    else if(world_model_info_.is_static_pass_ && world_model_info_.pass_ID_ == world_model_info_.AgentID_)
        shootPos_ =  PASS;
    else
        shootPos_ = SHOOT;

    if(match_mode_!= STOPROBOT && m_strategy_->selected_role_ == ACTIVE)
    {
        if(m_strategy_->ActiveRole_.kick_enable_ && world_model_info_.DribbleState_.is_dribble_ && !shootcnt_)
        {
            /// m_strategy_.kick_force_ 赋值根据你的标定结果，就是机器人于球门之间的距离 DPoint(900,0)
#ifdef SIMULATION
            strength_ = m_strategy_->ActiveRole_.kick_force_;
#else
            strength_ = m_strategy_->ActiveRole_.kick_force_;
#endif
            shootflg_ = true;
            m_strategy_->ActiveRole_.kick_enable_ = false;
            /// 将球踢出，newpassing置位
            world_model_info_.is_passed_out_ = true;
            isNewpassing_=true;
        }
        else
            strength_=0;
    }
    else
        strength_=0;

    if(shootflg_)
    {
        m_strategy_->ActiveRole_.kick_enable_ = false;
        shootcnt_++;
        if(shootcnt_>15)
        {
            shootflg_ = false;
            shootcnt_ = 0;
        }
    }
}

void NuBotControl::setEthercatCommond()
{
    /// 最后是解算出来的结果
    nubot_common::ActionCmd command;
    char move_action;
    char rotate_action;
    int  rotate_mode;

    move_action=m_plan_.m_behaviour_.move_action_;
    rotate_action=m_plan_.m_behaviour_.rotate_action_;
    rotate_mode=m_plan_.m_behaviour_.rotate_mode_;

    m_plan_.m_behaviour_.move_action_  =No_Action;
    m_plan_.m_behaviour_.rotate_action_=No_Action;
    m_plan_.m_behaviour_.rotate_mode_  =0;

    /// 运动参数
    command.move_action =move_action;
    command.rotate_acton=rotate_action;
    command.rotate_mode=rotate_mode;
    command.target.x=m_plan_.m_behaviour_.target_.x_;
    command.target.y=m_plan_.m_behaviour_.target_.y_;
    command.target_vel.x=m_plan_.m_behaviour_.target_vel_.x_;
    command.target_vel.y=m_plan_.m_behaviour_.target_vel_.y_;
    command.maxvel=m_plan_.m_behaviour_.maxvel_;
    command.target_ori=m_plan_.m_behaviour_.target_ori_;
    command.maxw=m_plan_.m_behaviour_.maxw_;


    if(m_plan_.m_behaviour_.maxvel_>maxvel_)
        command.maxvel=maxvel_;
    else
        command.maxvel=m_plan_.m_behaviour_.maxvel_;

    if(m_plan_.m_behaviour_.maxw_>maxw_)
        command.maxw=maxw_;
    else
        command.maxw=m_plan_.m_behaviour_.maxw_;

    /// 机器人人位置信息
    command.robot_pos.x=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getLocation().x_;
    command.robot_pos.y=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getLocation().y_;
    command.robot_vel.x=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getVelocity().x_;
    command.robot_vel.y=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getVelocity().y_;
    command.robot_ori=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getHead().radian_;
    command.robot_w=world_model_info_.RobotInfo_[world_model_info_.AgentID_-1].getW();

    /// 带球及射门选择
    command.handle_enable=handle_enable_;
    command.strength=strength_;
    if(strength_ != 0)
        std::cout<<"passed out"<<strength_<<std::endl;
    //        command.strength = 15;//zpm190609
    //    command.shootPos= SHOOT;
    command.shootPos = shootPos_;
    /// 传一次后，力量清0,防止多次射门
    strength_=0;
    action_cmd_pub_.publish(command);
}
/// \brief 中场于助攻在机器人动态传球时会出现穿过传球线的现象，在此矫正传球时候，中场与助攻的跑位点，防止传球失败
void NuBotControl::coorrect_target(bool & isDynamicStart, bool & isDynamicOut,  bool & IsDribble ,
                                   const DPoint & robot_pos, DPoint & target)
{
    /// 不需要校正
    if(!isDynamicStart && !isDynamicOut && !IsDribble)
        return;
    DPoint start_pt = ball_pos_;
    DPoint end_pt  = field_.oppGoal_[GOAL_MIDDLE];
    if(isDynamicStart )
    {
        start_pt  = world_model_info_.pass_cmds_.pass_pt;
        end_pt    = world_model_info_.pass_cmds_.catch_pt;
    }
    else if(isDynamicOut)
    {
        start_pt  =  world_model_info_.pass_state_.pass_pt_;
        end_pt    =  world_model_info_.pass_state_.catch_pt_;
    }
    else
    {
        start_pt = ball_pos_;
        end_pt   = field_.oppGoal_[GOAL_MIDDLE];
    }

    DPoint vec = end_pt -  start_pt;
    DPoint end_pt_tmp = end_pt + 150.0/ vec.length() * vec;
    LineSegment Pass2CatchLine(start_pt,end_pt_tmp);
    LineSegment Robot2TargetLine(robot_pos,target);
    DPoint cross_pt(0,0);
    double robot2LinePassCatch  = Pass2CatchLine.distance(robot_pos);
    double target2LinePassCatch = Pass2CatchLine.distance(target);
    /// 我方发球 不影响的情形，中常和助攻的正常跑位
    if(!Pass2CatchLine.crosspoint(Robot2TargetLine,cross_pt) && robot2LinePassCatch > 150 && target2LinePassCatch > 150 )
        return ;
    /// 其他情况则将中场和助攻点限制到一定的范围，即再中场、助攻跑位线上寻找一个合适的点；
    std::vector < DPoint> pts;
    pts.reserve(10);
    double max_y(start_pt.y_),min_y(end_pt.y_);
    if(start_pt.y_ < end_pt.y_)
    {
        max_y = end_pt.y_;
        min_y = start_pt.y_;
    }
    DPoint upper_pt =  start_pt;
    DPoint down_pt  =  end_pt;
    if( start_pt.x_ < end_pt.x_)
    {
        upper_pt = end_pt;
        down_pt  = start_pt;
    }
    pts.push_back(down_pt);
    pts.push_back(upper_pt);
    pts.push_back(DPoint(900.0*LENGTH_RATIO,upper_pt.y_));
    pts.push_back(DPoint(900.0*LENGTH_RATIO,600.0*WIDTH_RATIO));
    pts.push_back(DPoint(-900.0*LENGTH_RATIO,600.0*WIDTH_RATIO));
    pts.push_back(DPoint(-900.0*LENGTH_RATIO,down_pt.y_));
    /// 表示机器人要向y600方向运动
    if(pnpoly(pts,robot_pos))
    {
        /// 目标点首先再当前位置处，根据当前位置进行调整
        target.x_ =robot_pos.x_;
        /// 机器人靠近
        if(robot_pos.x_ >upper_pt.x_ && robot_pos.x_ < upper_pt.x_ +100)
            target.x_ = upper_pt.x_+100;
        else if(robot_pos.x_ < down_pt.x_ && robot_pos.x_ > down_pt.x_ -100)
            target.x_ = down_pt.x_-100;
        else if(robot_pos.x_ >= down_pt.x_ && robot_pos.x_ <= upper_pt.x_)
        {
            Angle delta_ang = DPoint(upper_pt.x_ -down_pt.x_,upper_pt.y_ -down_pt.y_).angle();
            if(delta_ang.radian_ > 0)
                target.x_ = down_pt.x_ -100;
            else
                target.x_ = upper_pt.x_+100;
        }
        if(fabs(target.x_) > 650 *WIDTH_RATIO)
            target.x_ = 650.0*target.x_/fabs(target.x_);

        target.y_ = max_y+200;
        if(fabs(target.y_) > 500 *WIDTH_RATIO)
            target.y_ = 500.0*WIDTH_RATIO;
        if(fabs(target.y_) < 400 *WIDTH_RATIO)
            target.y_ = 400.0*WIDTH_RATIO;
    }
    else
    {
        target.x_ =robot_pos.x_;
        if(robot_pos.x_ >upper_pt.x_ && robot_pos.x_ < upper_pt.x_ +100)
            target.x_ = upper_pt.x_+100;
        else if(robot_pos.x_ < down_pt.x_ && robot_pos.x_ > down_pt.x_ -100)
            target.x_ = down_pt.x_-100;
        else if(robot_pos.x_ >= down_pt.x_ && robot_pos.x_ <= upper_pt.x_)
        {
            Angle delta_ang = DPoint(upper_pt.x_ -down_pt.x_,upper_pt.y_ -down_pt.y_).angle();
            if(delta_ang.radian_ > 0)
                target.x_ = upper_pt.x_ +100;
            else
                target.x_ = down_pt.x_-100;
        }
        if(fabs(target.x_) > 650 *WIDTH_RATIO)
            target.x_ = 650.0*target.x_/fabs(target.x_);

        target.y_ = min_y-200;
        if(fabs(target.y_) > 500 *WIDTH_RATIO)
            target.y_ = -500.0*WIDTH_RATIO;
        if(fabs(target.y_) < 400 *WIDTH_RATIO)
            target.y_ = -400.0*WIDTH_RATIO;
    }
}

bool NuBotControl::pnpoly(const std::vector<DPoint> & pts, const DPoint & test_pt)
{
    int nvert=pts.size();
    int minX(100000),maxX((-100000)),maxY(-100000),minY((100000));
    for(std::size_t i = 0; i <nvert ;i++)
    {
        if(pts[i].x_<minX)
            minX=pts[i].x_;
        if(pts[i].x_>maxX)
            maxX=pts[i].x_;
        if(pts[i].y_<minY)
            minY=pts[i].y_;
        if(pts[i].y_>maxY)
            maxY=pts[i].y_;
    }

    if (test_pt.x_ < minX || test_pt.x_ > maxX || test_pt.y_< minY || test_pt.y_ > maxY)
        return false;

    int i, j;
    bool c = false;
    for (i = 0, j = nvert-1; i < nvert; j = i++)
    {
        if ( ( (pts[i].y_>test_pt.y_) != (pts[j].y_>test_pt.y_) ) &&
             (test_pt.x_ < (pts[j].x_-pts[i].x_) * (test_pt.y_-pts[i].y_)/double((pts[j].y_-pts[i].y_))+ pts[i].x_) )
            c = !c;
    }
    return c;
}

////@hbx
void NuBotControl::Opposite(const std_msgs::Bool& isOpposite_msg){
    _isOpposite = isOpposite_msg.data;
    if(_isOpposite){
        ROS_INFO("control_isopposite? %d",_isOpposite);}
}
