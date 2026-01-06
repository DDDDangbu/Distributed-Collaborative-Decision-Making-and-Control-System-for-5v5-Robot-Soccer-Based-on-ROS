#include "nubot_control/plan.h"
using namespace  nubot;
const double closeloop_time_delay = 0.05;
const double max_acc= 200.0;
Plan::Plan()
{

}

void
Plan::catchBallForCoop()
{
    double max_speed=150.0;
    if(robot_pos_.y_+50>FIELD_YLINE1)
        max_speed = MAXVEL*0.1;
    else if(robot_pos_.y_-50<FIELD_YLINE6)
        max_speed = MAXVEL*0.1;
    else if(robot_pos_.x_+50>FIELD_XLINE1)
        max_speed = MAXVEL*0.1;
    else if(robot_pos_.x_-50<FIELD_XLINE7)
        max_speed = MAXVEL*0.1;
    else
        max_speed = 150.0;

    float thetaofr2b = ball_pos_.angle(robot_pos_).radian_;

    //    DPoint catballvel = DPoint(0.0,0.0);

    m_behaviour_.setTarget(ball_pos_,max_speed,KickCoop/*,catballvel*/);
    m_behaviour_.setOrienation(thetaofr2b,3,KickCoop);
}

void
Plan::update()
{
    robot_pos_ = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    robot_ori_ = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    robot_vec_ = world_model_->RobotInfo_[world_model_->AgentID_-1].getVelocity();
    ball_pos_  = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    if((world_model_->BallInfo_[world_model_->AgentID_-1].getVelocity().x_ !=0)
            &&(world_model_->BallInfo_[world_model_->AgentID_-1].getVelocity().y_ != 0))
        ball_vec_  = world_model_->BallInfo_[world_model_->AgentID_-1].getVelocity();
//    else if(robot_vec_.length()==0)
//        ball_vec_  = world_model_->BallInfo_[world_model_->AgentID_-1].getVelocity();
    m_subtargets_.ball_pos_  =  ball_pos_;
}

void
Plan::catchBallSlowly()
{
    catchBallForCoop();
}

void
Plan::catchBall()
{
    /// 静态传球以及动态传球构成中，接球机器人的状态强制用动态求接法
    bool  is_pass = (world_model_->static_pass_time_ < WAIT_SECS &&  world_model_->KickSelection_ == KickBySelf)||
            (world_model_->pass_state_.is_passing_  && world_model_->pass_state_.is_dynamic_pass_  &&
             world_model_->pass_state_.catch_id_ == world_model_->AgentID_);

    double thetaofr2b = ball_pos_.angle(robot_pos_).radian_;
    double thetaofb2r = robot_pos_.angle(ball_pos_).radian_;
    double ballveltheta = ball_vec_.angle().radian_;
    double theta_d = ballveltheta - thetaofb2r;
    theta_d = angularnorm(theta_d);

    static bool isMoveless=false;
    if(isMoveless && ball_vec_.length()>40)  //zdx_note: the speed of ball is more than 0.4m/s, the state of ball is thought as moving
        isMoveless = false;
    if(!isMoveless && ball_vec_.length()<20)
        isMoveless = true;

    double max_speed;
    double max_w;
    //zdx_note: limitation which is to stop robot to go out the field or get into the penalty area
    if(robot_pos_.y_+50>FIELD_YLINE1)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else if(robot_pos_.y_-50<FIELD_YLINE6)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else if(robot_pos_.x_+50>FIELD_XLINE1)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else if(robot_pos_.x_-50<FIELD_XLINE7)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else
    {
        max_speed = MAXVEL;
        max_w=MAXW;
    }

    if( is_pass && fabs(theta_d)<deg(40) && ball_vec_.length()>50 )
    {
        DPoint p  = pglobal2rel(ball_pos_,ballveltheta,robot_pos_);
        DPoint p2 = prel2global(ball_pos_,ballveltheta,DPoint(p.x_,0.0));

        m_behaviour_.setTarget(p2,max_speed,CatchBall);
        m_behaviour_.setOrienation(thetaofr2b,max_w,CatchBall);
    }
    else
    {
        //std::cout<<"ball velocity"<<ball_vec_.length()<<std::endl;
        if(isMoveless)
        {
            catchMotionlessBall();
        }
        else
            catchMovingBall();
    }
}


void
Plan::catchMovingBall()
{
    //    std::cout <<"moving "<<std::endl;
    double max_speed;
    double max_w;
    if(robot_pos_.y_+50>FIELD_YLINE1)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else if(robot_pos_.y_-50<FIELD_YLINE6)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else if(robot_pos_.x_+50>FIELD_XLINE1)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else if(robot_pos_.x_-50<FIELD_XLINE7)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else
    {
        max_speed = MAXVEL;
        max_w=MAXW;
    }

    /// 一系列夹角
    double thetaofr2b = ball_pos_.angle(robot_pos_).radian_;
    double thetaofb2r = robot_pos_.angle(ball_pos_).radian_;
    double ballveltheta = ball_vec_.angle().radian_;
    double theta_d = ballveltheta - thetaofb2r;
    theta_d = angularnorm(theta_d);

    static bool isMovetorward = false;
    static bool isFar = false;
    static bool isbigVec = true;

    bool exist_obs=false;
    bool catch_ball=false;
    nubot::DPoint obs_point=nubot::DPoint(0.0,0.0);
    if(world_model_->Obstacles_.size()!=0)
    {
        for(int i =0; i<world_model_->Obstacles_.size();i++)
        {
            nubot::DPoint cur_obs;
            cur_obs = world_model_->Obstacles_.at(i);
            if(cur_obs.distance(ball_pos_)<70.0 /*&& cur_obs.distance(ball_pos_)<1.5*robot_pos_.distance(ball_pos_)*/)
            {
                obs_point = cur_obs;
                exist_obs=true;
                break;
            }
        }
    }
    if(exist_obs)
    {
        nubot::DPoint r2b,r2o;
        r2b = ball_pos_-robot_pos_;
        r2o = robot_pos_-obs_point;
        if((r2b.dot(r2o)/r2b.length()/r2o.length()>sqrt(3.0)*0.5)&&fabs(robot_ori_.radian_ - r2b.angle().radian_)<0.2)
        {
            catch_ball =true;
        }
        else
            catch_ball =false;
    }
    else
        catch_ball = true;
    //    std::cout<<"catch ball is "<<catch_ball<<"  exist_obs is "<<exist_obs<<std::endl;
    /// 一系列夹角
//    double thetaofr2b = ball_pos_.angle(robot_pos_).radian_;
//    double thetaofb2r = robot_pos_.angle(ball_pos_).radian_;
    if(catch_ball==true)
    {
        if(!isFar && robot_pos_.distance(ball_pos_)>150)
            isFar=true;
        else if(isFar && robot_pos_.distance(ball_pos_)<100)  //over 150 is far
            isFar=false;
        if(!isMovetorward && fabs(theta_d)<deg(50))
            isMovetorward=true;
        else if(isMovetorward && fabs(theta_d)>deg(60)) //less than 50 degree is move toward
            isMovetorward=false;
        if(!isbigVec && ball_vec_.length()>100)
            isbigVec=true;
        else if(isbigVec && ball_vec_.length()<80)  //ball's speed is over 100 is big velocity
            isbigVec=false;

        /// 球已出界，不再抓球
        if( fabs(ball_pos_.x_) > FIELD_LENGTH/2 + LOCATIONERROR*3 || fabs(ball_pos_.y_) > FIELD_WIDTH/2  + LOCATIONERROR*3)
        {
            m_behaviour_.clear();
            return;
        }
        else
        {
            if(isMovetorward && isbigVec)
            {
                DPoint p  = pglobal2rel(ball_pos_,ballveltheta,robot_pos_);
                DPoint p2 = prel2global(ball_pos_,ballveltheta,DPoint(p.x_,0.0));

                m_behaviour_.setTarget(p2,max_speed,CatchBall_slow);
                m_behaviour_.setOrienation(thetaofr2b,max_w,CatchBall_slow);
            }
            /// 球远离机器人运动
            else
            {
                // close loop time delay from image to movment
                DPoint future_ball = ball_pos_ + ball_vec_*closeloop_time_delay;//predict the motion of ball
                thetaofr2b = thetaof2p(robot_pos_,future_ball);
                DPoint catch_vel = vglobal2rel(ball_vec_,robot_ori_.radian_); //according to the velocity of ball and orientation of robot to caculate the speed
                catch_vel.y_ = 0;
                DPoint target_temp = future_ball;


                if(isFar)
                {
                    move2Positionwithobs(target_temp,max_speed);
                    m_behaviour_.setOrienation(thetaofr2b,max_w,CatchBall);
                }
                else
                {
                    m_behaviour_.setTarget(target_temp,max_speed,CatchBall,catch_vel);            //适当减速
                    m_behaviour_.setOrienation(thetaofr2b,max_w,CatchBall);
                }

            }
        }
    }
    else
    {
        int tmp_count=0;
        for(int i=0;i<world_model_->Opponents_.size();i++)
            if(world_model_->field_info_.isOurField(world_model_->Opponents_[i]))
                tmp_count++;

        DPoint target_temp;
        if(tmp_count==1)
            target_temp = ball_pos_.pointofline(obs_point,-150.0);
        else
            target_temp = ball_pos_.pointofline(world_model_->field_info_.ourGoal_[GOAL_MIDDLE],150.0);

        move2Positionwithobs(target_temp,max_speed);
        m_behaviour_.setOrienation(thetaofr2b,max_w,CatchBall);
    }
}

void
Plan::catchMotionlessBall()
{
//    std::cout<<"motion less zzq"<<std::endl;
    double max_speed;                                                   //边界减速
    double max_w;
    if(robot_pos_.y_+50>FIELD_YLINE1)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else if(robot_pos_.y_-50<FIELD_YLINE6)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else if(robot_pos_.x_+50>FIELD_XLINE1)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else if(robot_pos_.x_-50<FIELD_XLINE7)
    {
        max_speed = MAXVEL*0.1;
        max_w=MAXW*0.1;
    }
    else
    {
        max_speed = MAXVEL;
        max_w=MAXW;
    }


    bool exist_obs=false;
    bool catch_ball=false;
    nubot::DPoint obs_point=nubot::DPoint(0.0,0.0);
    if(world_model_->Obstacles_.size()!=0)
    {
        for(int i =0; i<world_model_->Obstacles_.size();i++)
        {
            nubot::DPoint cur_obs;
            cur_obs = world_model_->Obstacles_.at(i);
            if((cur_obs.distance(ball_pos_)<80.0 /*&& cur_obs.distance(ball_pos_)<1.5*robot_pos_.distance(ball_pos_)*/))
            {
                //                std::cout<<"exist obs"<<cur_obs.distance(ball_pos_)<<"  "<<robot_pos_.distance(ball_pos_)<<std::endl;
                obs_point = cur_obs;
                exist_obs=true;
                break;
            }
        }
    }
    if(exist_obs)
    {
        nubot::DPoint r2b,r2o;  //position of robot to ball and obstacles
        r2b = ball_pos_-robot_pos_;
        r2o = obs_point-ball_pos_;
        //std::cout<<r2b.angle().radian_<<"   r2b  and r2o "<<r2o.angle().radian_<<std::endl;
        if((r2b.dot(r2o)/r2b.length()/r2o.length()>sqrt(3.0)*0.5)&&fabs(robot_ori_.radian_ - r2b.angle().radian_)<0.2) //theory of cosine

        {
            ROS_WARN("break out %f",r2b.dot(r2o)/r2b.length()/r2o.length());
            catch_ball =true;
        }
        else
            catch_ball =false;
    }
    else
    {
        //        ROS_INFO("set catch ture");
        catch_ball = true;
    }
    //    std::cout<<"catch ball is "<<catch_ball<<"  exist_obs is "<<exist_obs<<std::endl;
    const double closeloop_time_delay = 0;
    DPoint future_ball = ball_pos_ + ball_vec_*closeloop_time_delay;
    float thetaofr2b = future_ball.angle(robot_pos_).radian_;
    float disofr2b = robot_pos_.distance(future_ball);

    if(catch_ball == true)
    {
        //        ROS_INFO("exist obs catch ball");
        static bool  islessdis =false;                                    //dw 3.31  判断
        if(!islessdis&&disofr2b<100)
            islessdis=true;
        if(islessdis&&disofr2b>150)
            islessdis=false;

        DPoint vel_real = vglobal2rel(ball_vec_,robot_ori_.radian_);

        if(!islessdis)
        {
            move2Positionwithobs(future_ball,max_speed);
            m_behaviour_.setOrienation(thetaofr2b,max_w,CatchBall);
        }
        else
        {
//            move2Positionwithobs(future_ball,max_speed);
//            m_behaviour_.setOrienation(thetaofr2b,max_w,CatchBall);
            DPoint catch_vel =ball_vec_;
            if(catch_vel.length()<=100)
                catch_vel = (catch_vel.length() + 50.0)/(catch_vel.length()+1) * catch_vel;//speed up to catch the ball
            else
                catch_vel = 1.8 * catch_vel; //for 1.5 old
            m_behaviour_.setTarget(future_ball,max_speed,CatchBall_slow,catch_vel);
            m_behaviour_.setOrienation(thetaofr2b,max_w,CatchBall_slow);
        }
        //        std::cout<<"catch less ball"<<std::endl;
    }
    else
    {
        int tmp_count=0;
        for(int i=0;i<world_model_->Opponents_.size();i++)
            if(world_model_->field_info_.isOurField(world_model_->Opponents_[i]))
                tmp_count++;

        DPoint target_temp;
        if(tmp_count==1)
            target_temp = ball_pos_.pointofline(obs_point,-150.0);
        else
            target_temp = ball_pos_.pointofline(world_model_->field_info_.ourGoal_[GOAL_MIDDLE],150.0);

        move2Positionwithobs(target_temp,max_speed);
        m_behaviour_.setOrienation(thetaofr2b,max_w,CatchBall);
    }
}

void
Plan::move2Positionwithobs(DPoint target, float maxvel, bool avoid_ball,bool is_static_passing)
{
    if(world_model_->AgentID_!=world_model_->pass_ID_&&world_model_->AgentID_!=world_model_->catch_ID_&&world_model_->is_static_pass_
            &&world_model_->CoachInfo_.MatchMode ==STARTROBOT)
    {
        std::cout<<"my agentid is"<<world_model_->AgentID_<<std::endl;
        m_subtargets_.subtarget(target, robot_pos_, avoid_ball,true);
    }
    else
        m_subtargets_.subtarget(target, robot_pos_, avoid_ball,false);
    double theta;
    nubot::DPoint target_vel,target_temp;
    if(m_subtargets_.subtargets_pos_==target)
    {
        m_behaviour_.setTarget(target,maxvel,AvoidObs,target_vel);
        theta = thetaof2p(robot_pos_,target);
    }
    else
    {
        m_behaviour_.setTarget(m_subtargets_.subtargets_pos_,maxvel,AvoidObs,target_vel);
        theta = thetaof2p(robot_pos_,m_subtargets_.subtargets_pos_);
    }
//        if(target.distance(robot_pos_)>20.0)
//        {
//            target_temp = robot_pos_.pointofline(target,20);
//            double acc_vel = sqrt(robot_vec_.length()* robot_vec_.length() + 2.0 * max_acc * 20);
//            double dcc_vel = sqrt(2.0*max_acc*(target.distance(robot_pos_)-20.0));
//            double velocity = acc_vel<dcc_vel ? acc_vel:dcc_vel;
//            nubot::DPoint r2t = target-robot_pos_;
//            if(velocity>maxvel)
//                velocity =maxvel;
//            //            std::cout<<acc_vel<<" acc "<<dcc_vel<<" dcc and final "<<velocity<<std::endl;
//            target_vel = DPoint(velocity*r2t.angle().Angcos(),velocity*r2t.angle().Angsin());
//            m_behaviour_.setTarget(target_temp,maxvel,AvoidObs,target_vel);
//            theta = thetaof2p(robot_pos_,target);
//        }
//        else
//        {
//            double acc_vel = sqrt(robot_vec_.length() * robot_vec_.length() + 2.0 * max_acc *target.distance(robot_pos_) );
//            double dcc_vel = sqrt(2.0*max_acc*(target.distance(robot_pos_)));
//            double velocity = acc_vel<dcc_vel ? acc_vel:dcc_vel;
//            nubot::DPoint r2t = target-robot_pos_;
//            if(velocity>maxvel)
//                velocity =maxvel;
//            target_vel = DPoint(velocity*r2t.angle().Angcos(),velocity*r2t.angle().Angsin());
//            m_behaviour_.setTarget(target,maxvel,AvoidObs,target_vel);
//            theta = thetaof2p(robot_pos_,target);
//        }
//    }
//    else
//    {
//        double sub_tar_vel = sqrt(2.0*max_acc*target.distance(m_subtargets_.subtargets_pos_));

//        if(m_subtargets_.subtargets_pos_.distance(robot_pos_)>20.0)
//        {
//            target_temp = robot_pos_.pointofline(m_subtargets_.subtargets_pos_,20);
//            double acc_vel = sqrt(robot_vec_.length()* robot_vec_.length() + 2.0 * max_acc * 20);
//            double dcc_vel = sqrt(sub_tar_vel * sub_tar_vel + 2.0*max_acc*(target.distance(robot_pos_)-20.0));
//            double velocity = acc_vel<dcc_vel ? acc_vel:dcc_vel;
//            nubot::DPoint r2t = m_subtargets_.subtargets_pos_-robot_pos_;
//            if(velocity>maxvel)
//                velocity =maxvel;
//            //            std::cout<<acc_vel<<" acc "<<dcc_vel<<" dcc and final "<<velocity<<std::endl;
//            target_vel = DPoint(velocity*r2t.angle().Angcos(),velocity*r2t.angle().Angsin());
//            m_behaviour_.setTarget(target_temp,maxvel,AvoidObs,target_vel);
//            theta = thetaof2p(robot_pos_,target);
//        }
//        else
//        {
//            double acc_vel = sqrt(robot_vec_.length() * robot_vec_.length() + 2.0 * max_acc *m_subtargets_.subtargets_pos_.distance(robot_pos_) );
//            double dcc_vel = sqrt(sub_tar_vel * sub_tar_vel + 2.0*max_acc*(m_subtargets_.subtargets_pos_.distance(robot_pos_)));
//            double velocity = acc_vel<dcc_vel ? acc_vel:dcc_vel;
//            nubot::DPoint r2t = m_subtargets_.subtargets_pos_ -robot_pos_;
//            nubot::DPoint st2t = target-m_subtargets_.subtargets_pos_;
//            double delta_theta = st2t.angle().radian_ - r2t.angle().radian_;
//            if(delta_theta>M_PI)
//                delta_theta = delta_theta - 2 *M_PI;
//            if(delta_theta<-M_PI)
//                delta_theta = delta_theta + 2* M_PI;
//            double sub_theta = r2t.angle().radian_ +delta_theta/2.0;
//            if(sub_theta>M_PI)
//                sub_theta = sub_theta - 2*M_PI;
//            if(sub_theta<-M_PI)
//                sub_theta = sub_theta + 2*M_PI;
//            if(velocity>maxvel)
//                velocity =maxvel;
//            target_vel = DPoint(velocity*Angle(sub_theta).Angcos(),velocity*Angle(sub_theta).Angsin());
//            m_behaviour_.setTarget(m_subtargets_.subtargets_pos_,maxvel,AvoidObs,target_vel);
//            theta = thetaof2p(robot_pos_,m_subtargets_.subtargets_pos_);
////        }
//        //        m_behaviour_.setTarget(m_subtargets_.subtargets_pos_,maxvel,AvoidObs);
//    }
}

void
Plan::move2Positionwithobs_withball(DPoint target, float maxvel)
{
    int obs_num=world_model_->Obstacles_.size();
    double near_obs_num=0;
    for(int i=0;i<obs_num;i++)
    {
        double dis=robot_pos_.distance(world_model_->Obstacles_.at(i));
        if(dis<250)
            near_obs_num++;
    }
    if(near_obs_num>0)
    {
        m_subtargets_.subtarget(target, robot_pos_, false);
        target=m_subtargets_.subtargets_pos_;
    }
    double theta=thetaof2p(robot_pos_,target);
    static bool target_change=true;
    static DPoint last_target(10000,10000);
    double ang_err=angularnorm(theta-robot_ori_.radian_);
    double radius_=50;
    double circle_v=200;

    if(last_target.distance(target)>0.1)
    {
        target_change=true;
        last_target=target;
    }

    if(robot_pos_.distance(target)<100)
    {
        m_behaviour_.setTarget(target,200,AvoidObs);
    }
    /// 如果不是朝向目标，则需要先带球转圈圈至目标朝向,因为转圈时速度时向前的，这样最不容易丢球
    else if(fabs(ang_err)>=60*SINGLEPI_CONSTANT/180&&target_change)
    {
        m_behaviour_.setTarget(target,circle_v,MoveWithBall);
        DPoint target_in_rob_coordinate=pglobal2rel(robot_pos_,robot_ori_.radian_,target);
        if(target_in_rob_coordinate.y_<0)
            m_behaviour_.setOrienation(0,-circle_v/radius_,MoveWithBall);
        else
            m_behaviour_.setOrienation(0,circle_v/radius_,MoveWithBall);
    }
    /// 机器人基本朝向目标时直接用PD调整
    else
    {
        target_change=false;
        m_behaviour_.setTarget(target,maxvel,AvoidObs);
        m_behaviour_.setOrienation(theta,4,AvoidObs);
    }
}

/// \brief Function:Find an available Point within 3m area, when interrupted by opponent.
/// The return pos is not the best one, just available pos need to reach.
/// use Polar Coordinates to generate the pos
DPoint
Plan::getAvailablePosIn3m(DPoint dribble_pos)
{
    /// 障碍物膨胀半径
    float obs_expand_radii = 80;
    float tooFarDis = 300;
    std::map<double,DPoint> pointToChoose;

    /// 均匀角度产生的num个点
    int num_angle=20,num_radii=4;
    for(int i=0;i<num_angle;i++)
    {
        float angle_temp = i*DOUBLEPI_CONSTANT/num_angle;
        for(int i=0;i<num_radii;i++)
        {
            /// 在290cm范围内每隔10cm,随机选一个
            float  radii_temp = 100 + i*50;
            float  dis2obs_sum = 0;
            double profit =0;
            DPoint pos_temp = DPoint(radii_temp*cos(angle_temp),radii_temp*sin(angle_temp)) + dribble_pos;
            //        cout<<i<<"  pos x: "<<pos_temp.x_<<" y: "<<pos_temp.y_<<endl;
            if(((robot_pos_.x_<world_model_->field_info_.xline_[2])&&(pos_temp.x_<robot_pos_.x_))
                    ||(fabs(pos_temp.x_)>=world_model_->field_info_.xline_[1])
                    ||(fabs(pos_temp.y_)>=(world_model_->field_info_.yline_[0]-100)))
                break;
            /// 去掉离障碍物很近的点
            if(world_model_->Opponents_.size()==0)
            {
                profit = 2*(pos_temp.x_-dribble_pos.x_)
                        - 0.5*fabs(pos_temp.y_)
                        - 0.6*dribble_pos.distance(pos_temp);
                pointToChoose.insert(make_pair(profit,pos_temp));
            }
            else
                for(int i=0;i<world_model_->Opponents_.size();i++)
                {
                    if(world_model_->Opponents_.at(i).distance(pos_temp) < obs_expand_radii)
                        break;
                    else if(world_model_->Opponents_.at(i).distance(pos_temp) < tooFarDis)
                        dis2obs_sum += world_model_->Opponents_.at(i).distance(pos_temp);
                    if(i == world_model_->Opponents_.size()-1)
                    {
                        /// 计算每个点的收益值
                        double profit = 2*(pos_temp.x_-dribble_pos.x_)
                                - 0.5*fabs(pos_temp.y_)
                                + 1.5*dis2obs_sum
                                - 0.6*dribble_pos.distance(pos_temp);
                        pointToChoose.insert(make_pair(profit,pos_temp));
                    }
                }
        }
    }
    if(pointToChoose.size()==0)
        return robot_pos_;
    std::map<double,DPoint>::iterator mapiter_end = pointToChoose.end();
    mapiter_end--;
    //    cout<<"final target in plan: x; "<<mapiter_end->second.x_<<" y: "<<mapiter_end->second.y_<<endl;
    /// map类型，按照key值从小到大存储数据，已经自动排序,最后选最后一个做为目标点
    return DPoint(mapiter_end->second.x_,mapiter_end->second.y_);
}

int
Plan::rand_num(int l, int r)
{
    unsigned int seed = std::clock();
    static unsigned int last_seed = seed;

    if (seed == last_seed)
        seed += std::rand();
    std::srand(seed);
    last_seed = seed;
    return std::rand() % (r - l) + l;
}

DPoint
Plan::repulsive2obstacle(float threshold, DPoint obstacle_pos, DPoint robot_pos, Angle robot_ori, float lamda)
{
    /// 障碍物到机器人的向量
    DPoint obs2robo;
    /// 障碍物到机器人的距离
    float length=0;

    DPoint rep_vector_sum;
    float tar_theta=0;
    float repulseForce=0;
    float force_length=0;


    length = robot_pos.distance(obstacle_pos) * 0.01;
    obs2robo = 0.8*(obstacle_pos - robot_pos)+0.2*(robot_pos-DPoint(0,0));
    obs2robo.x_ *= 0.01;
    obs2robo.y_ *= 0.01;

    if(length!=0 && threshold!=0)
        repulseForce = lamda * (1/length - 1/threshold)*(1/(length*length));
    if(obs2robo.length()!=0)
    {
        rep_vector_sum.x_ += repulseForce * obs2robo.x_/pow(obs2robo.length(),2);
        rep_vector_sum.y_ += repulseForce * obs2robo.y_/pow(obs2robo.length(),2);
    }

    /// 计算排斥速度
    tar_theta = rep_vector_sum.angle().radian_;
    force_length = rep_vector_sum.distance(DPoint(0,0)) * 100;
    double add_vx_ = -force_length * cos(tar_theta - robot_ori.radian_);
    double add_vy_ = -force_length * sin(tar_theta - robot_ori.radian_);

    return DPoint(add_vx_,add_vy_);
}

/// \brief  在世界坐标robot_pos处，direction方向上设置一个矩形区域，并且判断obs_info是否在这个矩形区域内这个矩形区域
/// 在这儿主要是判断机器人周围一定区域内是否存在这障碍物
/// \param  障碍物位置，机器人位置，矩形方向，矩形的相对位置
/// \return 是否存在障碍物
bool
Plan::isNullInTrap(const std::vector<DPoint> & obs_info, const DPoint & robot_pos, const Angle & direction,
                   double back_width, double front_width, double back_len, double front_len)
{
    /** 设计的矩形区域有问题，或者障碍物为空，直接返回true，表示区域内没有障碍物*/
    if(front_len <= 0 || obs_info.empty()|| back_len > front_len)
        return true;
    int nums_pts = 4;
    std::vector<DPoint> pts;
    pts.resize(nums_pts);
    /** 定义的矩形的四个顶点的在机器人体坐标系下的局部坐标*/
    std::vector<PPoint> real_pts;
    real_pts.resize(nums_pts);
    real_pts[0] = PPoint(DPoint(back_len,-back_width));
    real_pts[1] = PPoint(DPoint(front_len,-front_width));
    real_pts[2] = PPoint(DPoint(front_len,front_width));
    real_pts[3] = PPoint(DPoint(back_len,back_width));
    /** 局部坐标转换为世界坐标*/
    for(std::size_t i = 0 ; i < nums_pts; i++)
    {
        real_pts[i].angle_ = real_pts[i].angle_ + direction;
        pts[i] = robot_pos+DPoint(real_pts[i]);
    }
    return !pnpoly(pts,obs_info);
}

/// \brief  在世界坐标robot_pos处，在两个direction方向之间设置矩形区域，并且判断obs_info是否在这个区域内
/// 在这儿主要是判断机器人在小角度的旋转方向上是否有障碍物，如果有就放弃小角度旋转，转而选择大角度旋转
/// \param  障碍物位置，机器人位置，机器人朝向，旋转目标的朝向
/// \return false：逆时针旋转，true：顺时针旋转
bool
Plan::rotateMode(const std::vector<DPoint> & obs_info, const DPoint & robot_pos, const Angle & robot_direction, const Angle & target_direction)
{
    float rotate_angle=target_direction.radian_-robot_direction.radian_;

    while(rotate_angle > SINGLEPI_CONSTANT) rotate_angle = rotate_angle-2*SINGLEPI_CONSTANT;
    while(rotate_angle <= -SINGLEPI_CONSTANT) rotate_angle = rotate_angle+2*SINGLEPI_CONSTANT;

    int nums_pts = 3;
    std::vector<DPoint> pts;
    pts.resize(nums_pts+1);
    /// 定义矩形的其三个顶点在机器人体坐标系下的局部坐标
    std::vector<PPoint> real_pts;
    real_pts.resize(nums_pts);
    real_pts[0] = PPoint(robot_direction,150);
    real_pts[1] = PPoint(DPoint(0,0));
    real_pts[2] = PPoint(target_direction,150);
    /// 局部坐标转换为世界坐标
    for(std::size_t i = 0 ; i < nums_pts; i++)
        pts[i] = robot_pos+DPoint(real_pts[i]);

    /// 最后一个顶点根据前面三个点得到
    DPoint tmp=DPoint(pts[0].x_/2+pts[2].x_/2,pts[0].y_/2+pts[2].y_/2)-pts[1];
    pts[3]=DPoint(150*tmp.x_/tmp.length(),150*tmp.y_/tmp.length())+pts[1];
    /// 小角度为顺时针，若存在障碍物则逆时针旋转
    if(rotate_angle>0)
        return !pnpoly(pts,obs_info);
    /// 小角度为逆时针，若存在障碍物则顺时针旋转
    else
        return pnpoly(pts,obs_info);
}

/// \brief  判断点是否在凸多边形内部，用于检测机器人周围是否存在着障碍物
/// \param  凸多边形顶点位置，待检测障碍物位置
/// \return 存在障碍物在pts围成的区域内表示，返回true，否则位false
bool
Plan::pnpoly(const std::vector<DPoint> & pts, const std::vector<DPoint> & obs_pts)
{
    int nvert=pts.size();
    bool isInPoly= false;
    int nums_obs_pts = obs_pts.size();
    for(int k = 0 ; k < nums_obs_pts ; k++ )
    {
        DPoint  test_pt = obs_pts[k];
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
        /// 已经可以说明点在多边形外部,继续下一个循环
        if (test_pt.x_ < minX || test_pt.x_ > maxX || test_pt.y_ < minY || test_pt.y_ > maxY)
            continue;
        bool c =false;
        int i, j;
        for (i = 0, j = nvert-1; i < nvert; j = i++)
        {
            if ( ( (pts[i].y_>test_pt.y_) != (pts[j].y_>test_pt.y_) ) &&
                 (test_pt.x_ < (pts[j].x_-pts[i].x_) * (test_pt.y_-pts[i].y_)/double((pts[j].y_-pts[i].y_))+ pts[i].x_) )
                c = !c;
        }
        isInPoly = c;
        if(isInPoly)
            break;
    }
    return isInPoly;
}

/// \brief  获得某一角色机器人的位置
/// \param  角色
/// \return 该角色世界坐标
DPoint
Plan::getWhichRolePos(Roles role)
{
    for(std::size_t i = 0 ; i < world_model_->RobotInfo_.size(); i++)
    {
        nubot::Robot &robot_info = world_model_->RobotInfo_[i];
        if(robot_info.isValid() && robot_info.getCurrentRole() == role)
            return robot_info.getLocation();
    }
    /// 返回无效坐标
    return DPoint(10000,10000);
}
Angle
Plan::getWhichRoleOri(Roles role)
{
    for(std::size_t i = 0 ; i < world_model_->RobotInfo_.size(); i++)
    {
        nubot::Robot &robot_info = world_model_->RobotInfo_[i];
        if(robot_info.isValid() && robot_info.getCurrentRole() == role)
            return robot_info.getHead();
    }
    /// 返回无效坐标
    return Angle(0);
}
/// \brief  获得某一角色机器人的ID
/// \param  角色
/// \return 该角色ID
int
Plan::getWhichRoleID(Roles role)
{
    for(int i = 0 ; i < world_model_->RobotInfo_.size(); i++)
    {
        nubot::Robot &robot_info = world_model_->RobotInfo_[i];
        if(robot_info.isValid() && robot_info.getCurrentRole() == role)
            return robot_info.getID();
    }
    /// 返回无效坐标
    return -1;
}
