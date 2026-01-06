#include "nubot_control/subtargets.h"
using namespace nubot;

Subtargets::Subtargets()
{

}

int
Subtargets::Min_num(int n,double *q)              
{
    int min_out_num=0;
    for(int i=1;i<n;i++)
    {
        if(q[i]<q[min_out_num])
            min_out_num=i;
    }
    return min_out_num;
}

double
Subtargets::Min(int n,double *q)
{
    double min_out=q[0];
    for(int i=1;i<n;i++)
    {
        if(q[i]<min_out)
            min_out=q[i];
    }
    return min_out;
}

int
Subtargets::Max_num(int n,double *q)
{
    int max_out_num=0;
    for(int i=1;i<n;i++)
    {
        if(q[i]>q[max_out_num])
            max_out_num=i;
    }
    return max_out_num;
}

double
Subtargets::Max(int n,double *q)
{
    double max_out=q[0];
    for(int i=1;i<n;i++)
    {
        if(q[i]>max_out)
            max_out=q[i];
    }
    return max_out;
}

void
Subtargets::subtarget(DPoint &target_pos_, DPoint robot_pos, bool avoid_ball,bool is_static )
{
    std::vector<DPoint> obstacle_pos;
    /// 如果需要避让足球，必须优先保证足球的压入

//    if(fabs(robot_pos.angle(ball_pos_).radian_-world_model_->RobotInfo_[world_model_->AgentID_-1].getHead().radian_)<0.4&&(world_model_->pass_ID_==world_model_->AgentID_||world_model_->CurActiveNotGoalieNums_==1))
//        avoid_ball =false;
    if(avoid_ball)
        obstacle_pos.push_back(ball_pos_);
    /// 选择性的压入场地上的障碍物（选择距离自己在8米以内的障碍物压入）
    for(int i=0;i<world_model_->Obstacles_.size();i++)
        if(robot_pos.distance(world_model_->Obstacles_.at(i))<800||world_model_->Obstacles_.size()<9)
            obstacle_pos.push_back(world_model_->Obstacles_.at(i));
    /// 由于程序上的不完善，必须需要9个障碍物才能执行，不足的用无效点补齐
    for(int i=obstacle_pos.size();i<9;i++)
        obstacle_pos.push_back(DPoint(10000,10000));
    is_static =false;
    double radius_robot=40,radius_Obs=50,radius_kickoff =250.0;
    double a[9],b[9];
    DPoint point_=target_pos_- robot_pos;
//    std::cout<<"point "<<point_.x_<<"  "<<point_.y_<<std::endl;
//    std::cout<<"avoidball is"<<avoid_ball<<std::endl;
    int i=0,j=0,k=0;
    int B[9]={0};
    int First_num=0;
    double minB=0;

    int G[9]={-1,-1,-1,-1,-1,-1,-1,-1,-1};
    int G_num=0;
    int G_Obstacles_[9]={1,1,1,1,1,1,1,1,1};

    double b_positive[10]={0};
    double b_negative[10]={0};
    int left=0,right=0,sign_side=0,bp_num=0,bn_num=0;

    double atemp=0;
    double alpha_left[9]={0};
    double alpha_right[9]={0};
    double alpha_i=0;
    int    alpha_k=0;

    bool canpass=0;

    for(i=0;i<9;i++)
    {
        int temp=0;
        DPoint point1_= obstacle_pos.at(i)- robot_pos;
        if( point_.cross(point1_)==0 )
            temp=0;
        else if( point_.cross(point1_)>0 )
            temp=1;
        else
            temp=-1;

        a[i] = point_.ddot(point1_)/point_.length();
        b[i] = temp*fabs(point_.cross(point1_))/point_.length();
    }
    //obtain B that may hit
    for(i=0;i<9;i++)
    {
        if((a[i]>0)&&(a[i]<point_.length())&&(fabs(b[i])<(radius_robot+radius_Obs)))
        {
            B[j]=i;
            j++;
        }
    }

    if(j!=0)
    {
        //determain first Obstacles_
        First_num=B[0];
        minB=a[B[0]];
        for(i=1;i<j;i++)
        {
            if(minB<a[B[i]])
                minB=minB;
            else
            {
                minB=a[B[i]];
                First_num=B[i];
            }
        }

        //Grouping--the Obstacles_ that must be avoided
        G[0]=First_num;
        G_num=0;
        G_Obstacles_[First_num]=0;

        for(i=0;i<9;i++)
        {
            if(G_Obstacles_[i]==1)
            {
                for(k=0;k<=G_num;k++)
                {
                    if(obstacle_pos.at(i).distance(obstacle_pos.at(G[k]))<(2*radius_robot+2*radius_Obs))
                    {
                        G_num++;
                        G[G_num]=i;
                        G_Obstacles_[i]=0;
                        i=-1;
                        break;
                    }
                }
            }
        }
        for(i=0;i<=G_num;i++)
        {
            atemp=obstacle_pos.at(G[i]).distance(robot_pos);
            if(atemp<radius_Obs)
            {
                subtargets_pos_ = robot_pos;
                target_pos_     = robot_pos;
//                std::cout<<"fouls "<<std::endl;
                canpass=1;
                return;

            }
            //            std::cout<<sign_side<<" the sign side is "<<G_num<<std::endl;
            alpha_left[i]=atan2(b[G[i]],a[G[i]])+asin(radius_Obs/atemp) + atan2(radius_robot,sqrt(atemp*atemp - radius_Obs *radius_Obs));
            alpha_right[i]=atan2(b[G[i]],a[G[i]])-asin(radius_Obs/atemp) - atan2(radius_robot,sqrt(atemp*atemp - radius_Obs *radius_Obs));
        }
        if(fabs(Max(G_num+1,alpha_left)) < fabs(Min(G_num+1,alpha_right)))
        {
            left =1;
            alpha_i=Max(G_num+1,alpha_left);
            alpha_k=Max_num(G_num+1,alpha_left);
        }
        else
        {
            right =1;
            alpha_i=Min(G_num+1,alpha_right);
            alpha_k=Min_num(G_num+1,alpha_right);
        }

        DPoint tmp_ball_pos_;
        tmp_ball_pos_=ball_pos_;
        DPoint point1_= tmp_ball_pos_ - robot_pos;
        ///to avoid the robot run into the 3m circle
        double ball_a=0,ball_b=0,the2center=0,delta_theta=0,left_theta=0,right_theta=0;
        bool ball_obs = false;
        if(is_static)
        {
            std::cout<<"is static true"<<std::endl;
            ball_a = point_.ddot(point1_)/point_.length();
            ball_b = point_.cross(point1_)/point_.length();
            the2center = atan2(ball_b,ball_a);
            /// target or robot locates in the 3m circle
            if(target_pos_.distance(tmp_ball_pos_)<radius_kickoff
                    ||point1_.length()<radius_kickoff)
            {
                subtargets_pos_ = robot_pos;
                target_pos_     = robot_pos;
                return;
            }
            else
            {
                delta_theta = asin(radius_kickoff/point1_.length()) +atan2(radius_robot,(sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff)));
                left_theta  = the2center + delta_theta;
                right_theta = the2center - delta_theta;
            }
        }

        DPoint left_target_pos_,right_target_pos_;
        //std::cout<<"max and min "<<Max(G_num+1,alpha_left)<<"  "<<Min(G_num+1,alpha_right)<<std::endl;
        //std::cout<<"substarget"<<std::endl;
        //std::cout<<left_theta<<"left and right"<<right_theta<<std::endl;
        ///ball-obs-obs-ball
        if(left_theta>Max(G_num+1,alpha_left) && right_theta<Min(G_num+1,alpha_right))
        {
            //std::cout<<"case 1"<<std::endl;
            ball_obs = true;
            right_target_pos_.x_= robot_pos.x_+(cos(right_theta)*point_.x_-sin(right_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);
            right_target_pos_.y_= robot_pos.y_+(sin(right_theta)*point_.x_+cos(right_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);

            left_target_pos_.x_= robot_pos.x_+(cos(left_theta)*point_.x_-sin(left_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);
            left_target_pos_.y_= robot_pos.y_+(sin(left_theta)*point_.x_+cos(left_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);

            if((fabs(right_theta)>fabs(left_theta)||!world_model_->field_info_.isInInterField(right_target_pos_)) && world_model_->field_info_.isInInterField(left_target_pos_))
            {

                subtargets_pos_ = left_target_pos_;
                return;
            }
            else if(world_model_->field_info_.isInInterField(right_target_pos_))
            {
                subtargets_pos_ = right_target_pos_;
                return;
            }
            else
            {
                subtargets_pos_ = robot_pos;
                target_pos_     = robot_pos;
                std::cout<<"subtarget_out_of_field "<<std::endl;
                return;
            }
        }
        ///obs-obs-ball-ball or ball-ball-obs-obs or obs-ball-ball-obs
        else if(right_theta>=Max(G_num+1,alpha_left) || left_theta<=Min(G_num+1,alpha_right)||
                (right_theta>=Min(G_num+1,alpha_right)&&left_theta<=Max(G_num+1,alpha_left)))
        {

            //std::cout<<"case 2"<<std::endl;
            ball_obs = false;
            left_target_pos_.x_= robot_pos.x_+(cos(Max(G_num+1,alpha_left))*point_.x_-sin(Max(G_num+1,alpha_left))*point_.y_)/point_.length() *
                    sqrt(obstacle_pos.at(G[Max_num(G_num+1,alpha_left)]).distance(robot_pos) * obstacle_pos.at(G[Max_num(G_num+1,alpha_left)]).distance(robot_pos) - radius_Obs*radius_Obs + radius_robot*radius_robot);
            left_target_pos_.y_= robot_pos.y_+(sin(Max(G_num+1,alpha_left))*point_.x_+cos(Max(G_num+1,alpha_left))*point_.y_)/point_.length() *
                    sqrt(obstacle_pos.at(G[Max_num(G_num+1,alpha_left)]).distance(robot_pos) * obstacle_pos.at(G[Max_num(G_num+1,alpha_left)]).distance(robot_pos) - radius_Obs*radius_Obs + radius_robot*radius_robot);

            right_target_pos_.x_= robot_pos.x_+(cos(Min(G_num+1,alpha_right))*point_.x_-sin(Min(G_num+1,alpha_right))*point_.y_)/point_.length() *
                    sqrt(obstacle_pos.at(G[Min_num(G_num+1,alpha_right)]).distance(robot_pos) * obstacle_pos.at(G[Min_num(G_num+1,alpha_right)]).distance(robot_pos) - radius_Obs*radius_Obs + radius_robot*radius_robot);
            right_target_pos_.y_= robot_pos.y_+(sin(Min(G_num+1,alpha_right))*point_.x_+cos(Min(G_num+1,alpha_right))*point_.y_)/point_.length() *
                    sqrt(obstacle_pos.at(G[Min_num(G_num+1,alpha_right)]).distance(robot_pos) * obstacle_pos.at(G[Min_num(G_num+1,alpha_right)]).distance(robot_pos) - radius_Obs*radius_Obs + radius_robot*radius_robot);

//            std::cout<<"left "<<left_target_pos_.x_<<"   "<<left_target_pos_.y_<<std::endl;

//            std::cout<<"right "<<right_target_pos_.x_<<"   "<<right_target_pos_.y_<<std::endl;
            if((fabs(Min(G_num+1,alpha_right))>fabs(Max(G_num+1,alpha_left))||!world_model_->field_info_.isInInterField(right_target_pos_)) && world_model_->field_info_.isInInterField(left_target_pos_))
            {

                subtargets_pos_ = left_target_pos_;
                return;
            }
            else if(world_model_->field_info_.isInInterField(right_target_pos_))
            {
                subtargets_pos_ = right_target_pos_;
                return;
            }
            else
            {
                subtargets_pos_ = robot_pos;
                target_pos_     = robot_pos;
//                std::cout<<"subtarget_out_of_field "<<std::endl;
                return;
            }
        }
        else
        {
            ///ball-obs-ball-obs
            if(right_theta<=Min(G_num+1,alpha_right))
            {

//                std::cout<<"case 3"<<std::endl;
                ball_obs = true;
                right_target_pos_.x_= robot_pos.x_+(cos(right_theta)*point_.x_-sin(right_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);
                right_target_pos_.y_= robot_pos.y_+(sin(right_theta)*point_.x_+cos(right_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);

                left_target_pos_.x_= robot_pos.x_+(cos(Max(G_num+1,alpha_left))*point_.x_-sin(Max(G_num+1,alpha_left))*point_.y_)/point_.length() *
                        sqrt(obstacle_pos.at(G[Max_num(G_num+1,alpha_left)]).distance(robot_pos) * obstacle_pos.at(G[Max_num(G_num+1,alpha_left)]).distance(robot_pos) - radius_Obs*radius_Obs + radius_robot*radius_robot);
                left_target_pos_.y_= robot_pos.y_+(sin(Max(G_num+1,alpha_left))*point_.x_+cos(Max(G_num+1,alpha_left))*point_.y_)/point_.length() *
                        sqrt(obstacle_pos.at(G[Max_num(G_num+1,alpha_left)]).distance(robot_pos) * obstacle_pos.at(G[Max_num(G_num+1,alpha_left)]).distance(robot_pos) - radius_Obs*radius_Obs + radius_robot*radius_robot);
                if((fabs(right_theta)>fabs(Max(G_num+1,alpha_left))||!world_model_->field_info_.isInInterField(right_target_pos_)) && world_model_->field_info_.isInInterField(left_target_pos_))
                {

                    subtargets_pos_ = left_target_pos_;
                    return;
                }
                else if(world_model_->field_info_.isInInterField(right_target_pos_))
                {

                    subtargets_pos_ = right_target_pos_;
                    return;
                }
                else
                {
                    subtargets_pos_ = robot_pos;
                    target_pos_     = robot_pos;
//                    std::cout<<"subtarget_out_of_field "<<std::endl;
                    return;
                }
            }
            ///obs-ball-obs-ball
            else if(left_theta>=Max(G_num+1,alpha_left))
            {

//                std::cout<<"case 4"<<std::endl;
                ball_obs = true;
                left_target_pos_.x_= robot_pos.x_+(cos(left_theta)*point_.x_-sin(left_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);
                left_target_pos_.y_= robot_pos.y_+(sin(left_theta)*point_.x_+cos(left_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);

                right_target_pos_.x_= robot_pos.x_+(cos(Min(G_num+1,alpha_right))*point_.x_-sin(Min(G_num+1,alpha_right))*point_.y_)/point_.length() *
                        sqrt(obstacle_pos.at(G[Min_num(G_num+1,alpha_right)]).distance(robot_pos) * obstacle_pos.at(G[Min_num(G_num+1,alpha_right)]).distance(robot_pos) - radius_Obs*radius_Obs + radius_robot*radius_robot);
                right_target_pos_.y_= robot_pos.y_+(sin(Min(G_num+1,alpha_right))*point_.x_+cos(Min(G_num+1,alpha_right))*point_.y_)/point_.length() *
                        sqrt(obstacle_pos.at(G[Min_num(G_num+1,alpha_right)]).distance(robot_pos) * obstacle_pos.at(G[Min_num(G_num+1,alpha_right)]).distance(robot_pos) - radius_Obs*radius_Obs + radius_robot*radius_robot);

                if((fabs(Min(G_num+1,alpha_right))>fabs(left_theta)||!world_model_->field_info_.isInInterField(right_target_pos_)) && world_model_->field_info_.isInInterField(left_target_pos_))
                {    subtargets_pos_ = left_target_pos_;
                    return;}

                else if(world_model_->field_info_.isInInterField(right_target_pos_))
                {    subtargets_pos_ = right_target_pos_;
                    return;
                 }
                else
                {
                    subtargets_pos_ = robot_pos;
                    target_pos_     = robot_pos;
//                    std::cout<<"subtarget_out_of_field "<<std::endl;
                    return;
                }
            }
            else
            {
                subtargets_pos_=target_pos_;
//                std::cout<<"wrong"<<std::endl;
                return;
            }
        }
    }
    else
    {
        DPoint tmp_ball_pos_;
        tmp_ball_pos_.x_=0;
        tmp_ball_pos_.y_=0;
        ///to avoid the robot run into the 3m circle
        double ball_a=0,ball_b=0,the2center=0,delta_theta=0,left_theta=0,right_theta;
        bool ball_obs = false;
        DPoint point1_= tmp_ball_pos_ - robot_pos;
        if(is_static)
        {
            ball_a = point_.ddot(point1_)/point_.length();
            ball_b = point_.cross(point1_)/point_.length();
            the2center = atan2(ball_b,ball_a);
            /// target or robot locates in the 3m circle
            if(target_pos_.distance(tmp_ball_pos_)<radius_kickoff ||
                    point1_.length() < radius_kickoff )
            {
                std::cout<<target_pos_.distance(tmp_ball_pos_)<<std::endl;
                std::cout<<point1_.length()<<std::endl;
                subtargets_pos_ = robot_pos;
                target_pos_     = robot_pos;
//                std::cout<<"fouls "<<std::endl;
                return;
            }
            else
            {
                delta_theta = asin(radius_kickoff/point1_.length()) +atan2(radius_robot,(sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff)));
                left_theta  = the2center + delta_theta;
                right_theta = the2center - delta_theta;
            }
        }
        ///targetline-ballright-ballleft or ballright-ballleft-targetline
        if(right_theta>=0||left_theta<=0)
        {
            subtargets_pos_=target_pos_;
            ball_obs = false;
            return;
        }
        ///ballright-targetline-ballleft
        else
        {
            ball_obs = true;
            DPoint left_target_pos_,right_target_pos_;

            right_target_pos_.x_= robot_pos.x_+(cos(right_theta)*point_.x_-sin(right_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);
            right_target_pos_.y_= robot_pos.y_+(sin(right_theta)*point_.x_+cos(right_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);

            left_target_pos_.x_= robot_pos.x_+(cos(left_theta)*point_.x_-sin(left_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);
            left_target_pos_.y_= robot_pos.y_+(sin(left_theta)*point_.x_+cos(left_theta)*point_.y_)/point_.length() * sqrt(point1_.length()*point1_.length() - radius_kickoff * radius_kickoff + radius_robot*radius_robot);

            if((fabs(right_theta)>fabs(left_theta)||!world_model_->field_info_.isInInterField(right_target_pos_)) && world_model_->field_info_.isInInterField(left_target_pos_))
            {

                subtargets_pos_ = left_target_pos_;
                return;
            }
            else if(world_model_->field_info_.isInInterField(right_target_pos_))
            {

                subtargets_pos_ = right_target_pos_;
                return;
            }

            else
            {
                subtargets_pos_ = robot_pos;
                target_pos_     = robot_pos;
//                std::cout<<"subtarget_out_of_field "<<std::endl;
                return;
            }
        }
    }
}
