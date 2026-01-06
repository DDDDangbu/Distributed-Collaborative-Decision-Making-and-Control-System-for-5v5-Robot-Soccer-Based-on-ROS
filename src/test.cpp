#include "nubot_control/test.h"

using namespace nubot;

NubotTest::NubotTest(World_Model_Info& _world_model,Plan& _plan,ActiveRole& _active_role)
{
    t_world_model=&_world_model;
    t_plan=&_plan;
    t_active_role=&_active_role;
    trigger_shoot_ = false;
}
//!测试函数，点到点跑位
void NubotTest::TestStart2End(char id_A, DPoint& startPos,Angle& startOri,DPoint& endPos,Angle& endOri,char& Mode)
{
    static unsigned int state = 0;
    static unsigned int substate = 0;
    static DPoint target_pos=startPos;
    static Angle target_ang=startOri;
    static bool  is_next_start_pos = false;
    static int count_t = 0;
    bool isdribble = t_world_model->DribbleState_.is_dribble_;
    DPoint rob_pos=t_plan->robot_pos_;
    const int WAIT_TIME = 60;

    if(t_world_model->AgentID_ == id_A)
    {
        if((Mode==Move_Ball_NoAvoid||Mode==Move_Ball_Avoid) && !isdribble)
            state = 0;
        else
            state = 1;

        switch (state)
        {
        case 0:
            if(TestCatchBall(t_world_model->AgentID_))
                state = 1;
            break;
        case 1:
            switch (substate)
            {
            case 0:
                if(Mode==Move_Ball_NoAvoid||Mode==Move_NoBall_NoAvoid)//不避障
                    t_plan->m_behaviour_.setTarget(target_pos,300,CatchBall);
                else if(Mode==Move_Ball_Avoid||Mode==Move_NoBall_Avoid)//避障
                    t_plan->move2Positionwithobs(target_pos,300,false);
                t_plan->m_behaviour_.setOrienation(target_ang.radian_,5,Positioned);

                if(rob_pos.distance(target_pos)<20)
                    substate = 1;
                break;
            case 1:
                t_plan->m_behaviour_.clear();
                count_t++;
                if(count_t > WAIT_TIME)
                {
                    if(is_next_start_pos)
                    {
                        target_pos = startPos;
                        target_ang = startOri;
                    }
                    else
                    {
                        target_pos = endPos;
                        target_ang = endOri;
                    }
                    count_t = 0;
                    is_next_start_pos = !is_next_start_pos;
                    substate = 0;
                }
                break;
            default:
                printf("substate %d is invalid\n", substate);
                break;
            }
            break;
        default:
            printf("state %d is invalid\n", state);
            break;
        }
    }
    else
    {
        t_plan->m_behaviour_.clear();
        //printf("I am not Robot %d. I cannot move!\n", id_A);
    }
}
void NubotTest::TestPass(char& id_A,char& id_B,DPoint& PointA,DPoint& PointB,bool& is_pass)//id_A机器人抓球后传球，id_B机器人接球
{
    int robot_id=t_world_model->AgentID_;//获得机器人自身的ID
    DPoint rob_pos=t_plan->robot_pos_;
    Angle rob_ori=t_plan->robot_ori_;
    DPoint catch_pos;//接球机器人站位后的位置
    DPoint pass_pos;//传球机器人展位后的位置
    DPoint target_pos;
    Angle target_ori;

    if(robot_id == id_A)//如果我是id_A即传球机器人
    {
        target_pos=PointA;
        if(!is_pass)
        {
            if(t_world_model->DribbleState_.is_dribble_)//如果已经抓到球，则要跑到起始点
            {
                if(rob_pos.distance(target_pos)>5)//到达目标点附近5cm视为到达目标点，没到达则继续跑位
                    t_plan->move2Positionwithobs(target_pos,200,false);
                else//如果跑到目标点则调整角度传球
                {
                    catch_pos=t_world_model->RobotInfo_[id_B-1].getLocation();
                    target_ori=catch_pos.angle(rob_pos);//因为需要传球尽量精确，角度计算需要直接用到接球机器人的位置
                    if(fabs((rob_ori-target_ori).degree())>3)//如果误差大于3度，则继续调整
                        t_plan->m_behaviour_.setOrienation(target_ori.radian_,10,Positioned);
                    else
                    {
                        t_world_model->kick_force_=3+rob_pos.distance(catch_pos)/350;//调整传球力量
                        trigger_shoot_=true;//表示可以传球
                        if(rob_pos.distance(t_plan->ball_pos_) < 10)
                            is_pass=false;
                        else is_pass = true;
                    }
                }
            }
            else
                TestCatchBall(robot_id);
        }

    }
    else if(robot_id == id_B)//如果我是接球机器人,则只需要跑到位置后调整朝向
    {
        target_pos=PointB;
        if(rob_pos.distance(target_pos)>5)
            t_plan->move2Positionwithobs(target_pos,200,true);
        else
        {
            pass_pos=t_world_model->RobotInfo_[id_A-1].getLocation();
            target_ori=pass_pos.angle(t_plan->ball_pos_);
            if(fabs((rob_ori-target_ori).degree())>3)//如果误差大于3度，则继续调整
                t_plan->m_behaviour_.setOrienation(target_ori.radian_,10,Positioned);
        }
    }
    else
        t_plan->m_behaviour_.clear();
}
//!抓球测试函数,注意先确保使用该函数时，此函数内类的成员变量已经被初始化
bool NubotTest::TestCatchBall(char id_A)
{
    std::cout<<"testcatchBall::  "<<int(id_A)<<std::endl;
    static bool isMoveless=false;
    if(t_world_model->AgentID_ == id_A)
    {
        if(t_world_model->DribbleState_.is_dribble_)//如果抓到球，则开始带球
        {
            std::cout<<"is_dribble_::  "<<int(id_A)<<std::endl;
            t_plan->m_behaviour_.clear();
            return true;
        }
        else
        {
            if(isMoveless && t_plan->ball_vec_.length()>80)
                isMoveless = false;
            if(!isMoveless && t_plan->ball_vec_.length()<40)
                isMoveless = true;
            if(isMoveless)
                t_plan->catchMotionlessBall();           //球的速度比较小时的抓取动作
            else
                t_plan->catchMovingBall();
            return false;
        }
    }
    else
    {
        t_plan->m_behaviour_.clear();
        return false;
    }

}
//!测试机器人射门的函数; 注意，该函数被用于正是比赛中的点球，所以请调好这个函数!!!
bool NubotTest::TestShoot(char id_A, DPoint& kick_target, char& delta_force)
{
    if(t_world_model->AgentID_ == id_A)
    {
        if(TestCatchBall(t_world_model->AgentID_))//如果抓到球，则调整角度射门
        {
            if(t_active_role->evaluateKick())//先计算当前位置是否适合射门
            {
                t_active_role->turn4Shoot();//调整射门角度
                t_active_role->triggerShoot(delta_force);//计算射门力量
                return true;
            }
            else
                return false;
        }
        else
            return false;
    }
    else
    {
        t_plan->m_behaviour_.clear();
        return false;
    }
}
void NubotTest::TestCircle(char id_A, double &vx, double &radius)
{
    ROS_ERROR("agent %d circle test",id_A);
    Angle rob_ori=t_plan->robot_ori_;
    if(t_world_model->AgentID_ == id_A)
    {
//        ROS_ERROR("agent %d circle test",id_A);
//        if(t_world_model->DribbleState_.is_dribble_)
//        if(t_plan->ball_pos_.distance(t_plan->robot_pos_)<55)
//        if(t_world_model->DribbleState_.is_dribble_)
        if(1)
        {

//            t_plan->ball_pos_.x_ = 100;
//            t_plan->ball_pos_.y_ = 100;
//            if(t_plan->ball_pos_.distance(t_plan->robot_pos_)>100)
//            {
//                std::cout<<"move to origin"<<std::endl;
//                t_plan->m_behaviour_.setTarget(t_plan->ball_pos_,200,/*CircleTest*/TurnToPass);
//                t_plan->m_behaviour_.setOrienation(t_plan->ball_pos_.angle(t_plan->robot_pos_).radian_,2,/*CircleTest*/TurnToPass);
//            }
//            else
            if(1)
            {
//            t_plan->m_behaviour_.setTarget(DPoint(0,0),vx,CircleTest);                       //没有什么意义，只是单纯用来传参数
                if(radius!=0)
                {
                    Angle cur_r2b;
                    cur_r2b = t_plan->ball_pos_.angle(t_plan->robot_pos_);
                    if(cur_r2b.radian_<0)
                    {
                        double target_ori = cur_r2b.radian_+0.1>0? 0:cur_r2b.radian_+0.1;
//                        t_plan->m_behaviour_.setOrienation(target_ori,1,Positioned_Static);
                        DPoint target_vel;
                        //                    target_vel.x_ = 0;
                        //                    target_vel.y_ = 50;
                        std::cout<<"zzq1"<<std::endl;
                        DPoint target_pos;
                        double tar_w;
                        double tar_ori;
                        if(cur_r2b.radian_+0.1<0)
                        {
                            target_vel.x_ = -radius * Angle(target_ori).Angsin();
                            target_vel.y_ = -radius * Angle(target_ori).Angcos();
                            tar_ori = cur_r2b.radian_+0.1;
                            tar_w   = 1.0;
                            target_pos=t_plan->ball_pos_.pointofline(t_plan->robot_pos_,radius);
                            target_pos=target_pos.rotate(t_plan->ball_pos_,Angle(0.1)) ;
                            std::cout<<target_pos.x_<<"   "<<target_pos.y_<<std::endl;
                        }
                        else
                        {
                            tar_w   = 0.0;
                            tar_ori = 0.0;
                            target_vel.x_=0;
                            target_vel.y_=0;
                            target_pos=t_plan->ball_pos_.pointofline(t_plan->robot_pos_,radius);
                            target_pos=target_pos.rotate(t_plan->ball_pos_,Angle(0.0 - cur_r2b.radian_)) ;
                        }
                        //                    target_vel.x_=0;
                        //                    target_vel.y_=0;
                        t_plan->m_behaviour_.setOrienation(tar_ori,2,/*CircleTest*/TurnToPass,1,tar_w);
                        t_plan->m_behaviour_.setTarget(target_pos,300,/*CircleTest*/TurnToPass,target_vel);
                    }
                    else
                    {

                        double target_ori = cur_r2b.radian_- 0.1>0? cur_r2b.radian_-0.1 : 0;
//                        t_plan->m_behaviour_.setOrienation(target_ori,1,Positioned_Static);
                        DPoint target_vel;
                        //                    target_vel.x_ = 0;
                        //                    target_vel.y_ = 50;
                        std::cout<<"zzq1"<<std::endl;
                        DPoint target_pos;
                        double tar_w;
                        double tar_ori;
                        if(cur_r2b.radian_-0.1>0)
                        {
                            target_vel.x_ = radius * Angle(target_ori).Angsin();
                            target_vel.y_ = radius * Angle(target_ori).Angcos();
                            tar_ori = cur_r2b.radian_-0.1;
                            tar_w   = -1.0;
                            target_pos=t_plan->ball_pos_.pointofline(t_plan->robot_pos_,radius);
                            target_pos=target_pos.rotate(t_plan->ball_pos_,Angle(-0.1)) ;
                        }
                        else
                        {
                            tar_w   = 0.0;
                            tar_ori = 0.0;
                            target_vel.x_=0;
                            target_vel.y_=0;
                            target_pos=t_plan->ball_pos_.pointofline(t_plan->robot_pos_,radius);
                            target_pos=target_pos.rotate(t_plan->ball_pos_,Angle(cur_r2b.radian_)) ;
                        }
                        t_plan->m_behaviour_.setOrienation(tar_ori,2,/*CircleTest*/TurnToPass,-1,tar_w);
                        t_plan->m_behaviour_.setTarget(target_pos,300,/*CircleTest*/TurnToPass,target_vel);
                    }
                }
                else
                {
                    std::cout<<"round round "<<std::endl;
                    t_plan->m_behaviour_.setOrienation(rob_ori.radian_+2,MAXW,/*CircleTest*/TurnToPass);
                }
            }
        }
        else
        {
            DPoint target_pos;
            target_pos =  t_plan->ball_pos_.pointofline(t_plan->robot_pos_,50);
            t_plan->m_behaviour_.setTarget(target_pos,300,CircleTest);
            double target_ori;
            target_ori = t_plan->ball_pos_.angle(t_plan->robot_pos_).radian_;
            t_plan->m_behaviour_.setOrienation(target_ori,3,CircleTest);
            TestCatchBall(id_A);
        }
    }
    else
        t_plan->m_behaviour_.clear();
}
//!机器人的定位测试函数，让机器人跑球场上现场可以测量出来的点，看视觉定位是否准确
void NubotTest::TestLocation(char id_A)
{
    if(t_world_model->AgentID_ == id_A)
    {
        DPoint rob_pos=t_plan->robot_pos_;
        Angle rob_ori=t_plan->robot_ori_;
        static DPoint Point[5]={DPoint(0,0),DPoint(0,-FIELD_CENTER_RADIUS),DPoint(FIELD_XLINE3,-FIELD_YLINE2),DPoint(FIELD_XLINE3,FIELD_YLINE2),DPoint(0,FIELD_CENTER_RADIUS)};//这是在家里用的点
        static int i=0;//用于记录跑到第几个目标点
        static DPoint target_pos=Point[i];
        static ros::Time reach_time=ros::Time::now();
        ros::Time current_time=ros::Time::now();
        ros::Duration stay_time;
        float dis2target=rob_pos.distance(target_pos);
        static bool is_reach=false;
        if(dis2target<10&!is_reach)//记录下跑到目标点附近20cm时的时间
        {
            reach_time=ros::Time::now();
            is_reach=true;
        }
        stay_time=current_time-reach_time;//记录停留的时间
        if(stay_time.toSec()>2&&dis2target<10&&is_reach)//如果在目标点附近停留2秒以上，则跑到下一个目标点
        {
            is_reach=false;
            i++;
            if(i>4) i=0;//跑到最后一个点则要从头跑
            target_pos=Point[i];
        }
        t_plan->m_behaviour_.setTarget(target_pos,200,Positioned);
        t_plan->m_behaviour_.setOrienation(0,10,Positioned);//朝向始终是0度
    }
    else
        t_plan->m_behaviour_.clear();
}
