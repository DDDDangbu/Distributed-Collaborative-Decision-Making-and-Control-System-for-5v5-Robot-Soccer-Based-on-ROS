#include "nubot_control/assistrole.h"

using namespace nubot;

AssistRole::AssistRole()
{
    angle_.reserve(30);
    angle_dis_.reserve(30);
    pass_sight_ = 0;
    pass_direction_ = 0;
    can_pass_ = false;
    init_finished_ = false;
    assist_pt_ = DPoint(250,0);
}

AssistRole::~AssistRole()
{}

void
AssistRole::process()
{
    if(world_model_->CoachInfo_.MatchMode ==STOPROBOT)
    {
        plan_->m_behaviour_.clear();
        return;
    }
    else if(world_model_->CoachInfo_.MatchMode == STARTROBOT)
    {
        move2AssitPt();
        return;
    }
}

void AssistRole::move2AssitPt()
{
    DPoint robot_pos=world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    DPoint ball_pos=world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    Angle  robot_ori=world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    Angle angle_position, delta_ang;
    if(world_model_->BallInfoState_ != NOTSEEBALL )
    {
        angle_position = ball_pos.angle(robot_pos);
        delta_ang = angle_position- robot_ori;
    }
    else
    {
        angle_position = robot_ori;
        delta_ang = Angle(0);
    }

//    cout<<"assist pt x "<<assist_pt_.x_<<" y "<<assist_pt_.y_<<endl;
//    if(world_model_->is_static_pass_ &&world_model_->catch_ID_!= world_model_->AgentID_&&world_model_->pass_ID_!= world_model_->AgentID_)
//    {
//        plan_->move2Positionwithobs(assist_pt_,MAXVEL,true,true);
//    }
//    else
        plan_->move2Positionwithobs(assist_pt_,MAXVEL,true);
    if(fabs(delta_ang.degree())>ANGLEERROR )
        plan_->m_behaviour_.setOrienation(angle_position.radian_,fabs(delta_ang.degree())*MAXW/(2*M_PI),Positioned);
}

void AssistRole::assistCalculate()              //助攻站位点计算
{
    DPoint ball_pos=world_model_->fuseBallInfo_.getGlobalLocation();
    DPoint assist_pos=plan_->getWhichRolePos(ASSISTANT);
    DPoint active_pos= plan_->getWhichRolePos(ACTIVE);
    DPoint midfield_pos=plan_->getWhichRolePos(MIDFIELD);

    double mid_weight = (midfield_pos != DPoint(10000,10000)) ? 0.4 : 0;
    double assist_weight = (assist_pos != DPoint(10000,10000)) ? 0.2 : 0;

    //助攻的跑位点在主攻周围的圆环区域内选
    float radii_lower = (FIELD_WIDTH/2 > 500) ? 500 : (FIELD_WIDTH/2);   //圆环 内 半径
    float radii_upper = FIELD_WIDTH*4/5;        //圆环 外 半径
    int pass_line_expand = 120;  //传球线膨胀
    int tooFarObs = 400;
    int step_radii = 40;
    int num_angle =  80;   //射线数=360/步长
    int obs_expand_radii = 100;
    std::map<double,DPoint> pointToChoose;//<(待选点的收益值),(x,y)>

    //均匀角度产生的n个点
    for(int i=1;i<num_angle;i++) //angle
    {
        float angle_temp = i*DOUBLEPI_CONSTANT/num_angle;
        double profit=0;

        for(float j=radii_lower;j<radii_upper;j+=step_radii)  //radii
        {
            float radii_temp = j;
            //极坐标，场地x为正向
            DPoint pos_temp = DPoint(radii_temp*cos(angle_temp),radii_temp*sin(angle_temp)) + active_pos;
//            cout<<"Pos i: x: "<<pos_temp.x_<<" y: "<<pos_temp.y_<<endl;
            if ((fabs(pos_temp.x_) > FIELD_XLINE3) || (fabs(pos_temp.y_ ) > (FIELD_WIDTH/2 - 150)))
                break;    //x方向上去掉大于禁区线的; y方向上去掉距离边界150内的

            float dis2obs_sum=0;
            //去掉离障碍物很近的点
            if(world_model_->Opponents_.size()==0)
            {
                //计算待选点的收益值，都是距离的函数，需要调试
                profit = -0.3*fabs(pos_temp.x_ - FIELD_LENGTH/4)   //助攻距离场地的x坐标1/4位置较好； //0.8
                        -1*fabs(active_pos.distance(pos_temp) - FIELD_LENGTH*0.22)  //助攻与主攻的距离为场地长的0.22倍比较好,便于传接球； //1
                        +mid_weight*midfield_pos.distance(pos_temp)   //助攻与中场不能太近
                        -1*assist_weight*assist_pos.distance(pos_temp);     //距离当前助攻位置越近越好 //1

                pointToChoose.insert(make_pair(profit,pos_temp));
            }
            else
                for(int i=0;i<world_model_->Opponents_.size();i++)
                {

                    if(pos_temp.distance(world_model_->Opponents_.at(i)) < obs_expand_radii)
                        break;
                    double dis2ball = pos_temp.distance(ball_pos);
                    //将球的位置向待选点方向推100，需要调试
                    DPoint ball_pos_closer = ball_pos +
                            100.0/dis2ball*DPoint(pos_temp.x_-ball_pos.x_, pos_temp.y_-ball_pos.y_);

                    LineSegment ball2temp_pos = LineSegment(ball_pos_closer,pos_temp);
                    // (x,y) 坐标落在线段"内"的障碍物，计算其到线段的距离;
                    //距离小于pass_line_expand，去掉该点
                    if( (ball2temp_pos.decideWhere(world_model_->Opponents_.at(i)) == ball2temp_pos.BetweenStartAndEnd) &&
                            (ball2temp_pos.distance(world_model_->Opponents_.at(i),true) < pass_line_expand) )
                        break;

                    if(pos_temp.distance(world_model_->Opponents_.at(i)) < tooFarObs)
                        dis2obs_sum += pos_temp.distance(world_model_->Opponents_.at(i));

                    if(i==world_model_->Opponents_.size()-1)
                    {
                        //计算待选点的收益值，都是距离的函数，需要调试
                        profit = -0.3*fabs(pos_temp.x_ - FIELD_LENGTH/4)   //助攻距离场地的x坐标1/4位置较好；
                                -1*fabs(active_pos.distance(pos_temp) - FIELD_LENGTH*0.22)  //助攻与主攻的距离为场地长的0.22倍比较好,便于传接球；
                                +mid_weight*midfield_pos.distance(pos_temp)   //助攻与中场不能太近
                                -1*assist_weight*assist_pos.distance(pos_temp)     //距离当前助攻位置越近越好
                                +0.3*dis2obs_sum;                      //距离障碍物越远越好

                        pointToChoose.insert(make_pair(profit,pos_temp));
                    }
                }
        }
    }
    if(pointToChoose.size() != 0)
    {
        std::map<double,DPoint>::iterator mapiter_end = pointToChoose.end();
        mapiter_end--;
        assist_pt_ = mapiter_end->second;
//        cout<<"assist pos x "<<assist_pt_.x_<<"  y  "<<assist_pt_.y_<<endl;
        return;
    }
    else
    {
        assist_pt_ = assist_pos;
//        cout<<"assist pos x "<<assist_pt_.x_<<"  y  "<<assist_pt_.y_<<endl;
        return;
    }

}

bool AssistRole::findEmptyPlace()
{
    if(!init_finished_)
    {
        // assist location
        int interval = 100;
        int offset = 150;
        int xstart =  world_model_->field_info_.xline_[3] + offset;
        int xend   =  world_model_->field_info_.xline_[2] - offset;
        int ystart =  world_model_->field_info_.yline_[5] + offset;
        int yend   =  world_model_->field_info_.yline_[0] - offset;
        samples_.reserve(50);
        LocationWeight locWeight;
        for(int x = xstart ;  x <xend; x+=interval )
            for(int y = ystart ;  y < yend; y+=interval )
            {
                locWeight.circle_ = Circle(interval*0.7071,DPoint(x,y));
                locWeight.weight_ = 0.0;
                locWeight.disweight_ = 0.0;
                samples_.push_back(locWeight);
            }
        init_finished_ = true;
    }
    std::vector<LineSegment> Ball2OppsLines;
    bool update = false;
    DPoint fuseBallPos = world_model_->fuseBallInfo_.getGlobalLocation();
    if(world_model_->BallInfoState_!=NOTSEEBALL && world_model_->Opponents_.size()!=0)
    {
        for(int i = 0 ; i < world_model_->Opponents_.size() ; i++)
        {
            double distance  = fuseBallPos.distance(world_model_->Opponents_[i]);
            if(distance >0 && world_model_->Opponents_[i].x_!=0 && world_model_->Opponents_[i].y_!=0)
            {
                DPoint inteval_pt = world_model_->Opponents_[i];
                if(distance > 100)
                {
                    double t = distance-100.0 / distance;
                    inteval_pt = fuseBallPos + t * DPoint(world_model_->Opponents_[i].x_ - fuseBallPos.x_,world_model_->Opponents_[i].y_ - fuseBallPos.y_);
                    Ball2OppsLines.push_back(LineSegment(fuseBallPos, inteval_pt));
                }
                else
                    Ball2OppsLines.push_back(LineSegment(fuseBallPos, inteval_pt));
            }
        }
    }
    if(Ball2OppsLines.size() > 0)
        update = true;
    if(!update) return update;

    for(int i = 0 ; i <  samples_.size() ; i++)
    {
        samples_[i].weight_ = 0;
        double distance  = fuseBallPos.distance(samples_[i].circle_.center_);
        samples_[i].disweight_ = (distance > 600||distance < 200) ? 0.1: (0.4-0.0015*fabs(distance - 450)); //0.1~0.4
    }

    for(int i = 0 ; i < Ball2OppsLines.size(); i++)
        for(int j = 0; j < samples_.size() ; j++)
        {
            int where =  Ball2OppsLines[i].decideWhere(samples_[j].circle_.center_);
            if(where != LineSegment::EndPtRight) //样本点在障碍物之前；
                continue;
            LineSegment lineBall2center(fuseBallPos,samples_[j].circle_.center_);
            Angle deltaAng( fabs( (Ball2OppsLines[i].vector_.angle() - lineBall2center.vector_.angle()).radian_) ); //-pi ~pi
            Angle goodAngle  = atan(60.0/Ball2OppsLines[i].distance());
            if(fabs(deltaAng.radian_) < fabs(goodAngle.radian_))
                samples_[j].weight_ += 1;
        }
    // std::sort(samples_.begin(),samples_.end(),[](nubot::LocationWeight & a, nubot::LocationWeight & b){return (a.weight_ < b.weight_);});

    std::vector< std::vector<LocationWeight> > classify_samples;
    classify_samples.resize(samples_.size());
    for(int i = 0 ; i < samples_.size() ; i++)
        classify_samples[int(samples_[i].weight_)].push_back(samples_[i]);
    for(int i = 0 ; i < classify_samples.size() ; i++)
    {
        std::vector<LocationWeight> & sample = classify_samples[i];
        if(sample.size() > 0)
        {
            double maxWeight = -FLT_MAX;
            int index = -1;
            for(int j = 0 ; j < sample.size() ; j++ )
            {
                if(sample[j].disweight_ > maxWeight )
                {
                    index = j;
                    maxWeight =sample[j].disweight_;
                }
            }
            assist_pt_ = sample[index].circle_.center_;
        }
        break;
    }
    return true;
}

double AssistRole::searchMaxSight(double &dest_angle, const DPoint &_point_from, const double &_angle_min, const double &_angle_max, const double &_max_distance)
{
        bool is_cross_PI = false;//给定的角度阈值是否跨越±PI，阈值判断时用到
        if( _angle_min > _angle_max )
            is_cross_PI = true;
        double best_angle=0;//最大角度间隔对应的中分角
        double max_angle_dis = 0;//最大角度间隔
        double obs_num = world_model_->Opponents_.size();// 障碍物的数目；

        angle_dis_.clear();
        angle_.clear();
        angle_dis_.resize(obs_num);
        angle_.resize(obs_num);

        if( obs_num == 0 )//场地内没有障碍物
        {
            if(!is_cross_PI)
            {
                best_angle = (_angle_min + _angle_max ) / 2;
                max_angle_dis = _angle_max -_angle_min;
            }
            else
            {
                best_angle = (_angle_min + _angle_max + DOUBLEPI_CONSTANT ) / 2;
                if( best_angle>SINGLEPI_CONSTANT ) best_angle -= DOUBLEPI_CONSTANT;
                max_angle_dis = _angle_max - _angle_min + DOUBLEPI_CONSTANT;
            }
            dest_angle = best_angle;
            return max_angle_dis;
        }

        int num = 0;//有效角度的个数（即在_max_distance阈值范围之内的）
        for(int i=0; i<obs_num; i++)
            angle_[i] = -SINGLEPI_CONSTANT;
        double angle_current;
        for(int i=0; i<obs_num; i++)
        {
            if( (world_model_->Opponents_[i]-_point_from).length()>_max_distance )
                continue;
            angle_current = world_model_->Opponents_[i].angle(_point_from).radian_;
            if(num==0)
                angle_[0] = angle_current;
            else
            for(int j=num; j>=0; j--)//angle 从小到大排列存储
            {
                if(j==0)
                    angle_[j] = angle_current;
                else if(angle_current>angle_[j-1])
                {
                    angle_[j] = angle_current;
                    break;
                }
                else
                    angle_[j] = angle_[j-1];
            }
            num++;
        }


        //判断某个角度是否在阈值范围内的宏定义
    #define IsInThresh(in) ( !is_cross_PI && in > _angle_min && in < _angle_max ) \
        || ( is_cross_PI && in > -SINGLEPI_CONSTANT        && in < _angle_max ) \
        || ( is_cross_PI && in > _angle_min && in < SINGLEPI_CONSTANT         )

        //检测角度阈值内是否有障碍物
        bool is_obs_valid = false;
        for( int i=0; i<num; i++ )
        {
            if( IsInThresh(angle_[i]) )
                is_obs_valid = true;
        }
        if( !is_obs_valid )
        {
            if(!is_cross_PI)
            {
                best_angle = (_angle_min + _angle_max ) / 2;
                max_angle_dis = _angle_max -_angle_min;
            }
            else
            {
                best_angle = (_angle_min + _angle_max + DOUBLEPI_CONSTANT ) / 2;
                if( best_angle>SINGLEPI_CONSTANT ) best_angle -= DOUBLEPI_CONSTANT;
                max_angle_dis = _angle_max - _angle_min + DOUBLEPI_CONSTANT;
            }
            dest_angle = best_angle;
            return max_angle_dis;
        }

        //有障碍物，则需要计算相邻障碍物之间间隔，并搜索最大间隔↓↓↓
        //第一个障碍物和最后一个障碍物间隔的计算不方便放在for循环中，单独计算（只有一个障碍物时也兼容）
        angle_dis_[0] = angle_[0] - angle_[num-1] + DOUBLEPI_CONSTANT;
        double best_angle_temp = (angle_[0] + angle_[num-1] + DOUBLEPI_CONSTANT)/2;
        if( best_angle_temp>SINGLEPI_CONSTANT ) best_angle_temp -= DOUBLEPI_CONSTANT;
        if( IsInThresh(best_angle_temp) )
        {
            max_angle_dis = angle_dis_[0];
            best_angle = best_angle_temp;
        }
        for(int i=1; i<num; i++)
        {
            angle_dis_[i] = angle_[i] -angle_[i-1];
            if( angle_dis_[i]>max_angle_dis )
            {
                best_angle_temp = (angle_[i] + angle_[i-1])/2;
                if( IsInThresh(best_angle_temp) )
                {
                    max_angle_dis = angle_dis_[i];
                    best_angle = best_angle_temp;
                }
            }
        }

        //最后计算两条角度阈值边界的视野
        double angle_dis_min = 0;
        if( _angle_min<angle_[0] )
            angle_dis_min = std::min( _angle_min-angle_[num-1]+DOUBLEPI_CONSTANT, angle_[0]-_angle_min);
        else if( _angle_min>angle_[num-1] )
            angle_dis_min = std::min( _angle_min-angle_[num-1], angle_[0]-_angle_min+DOUBLEPI_CONSTANT);
        else for(int i=1; i<num; i++)
        {
            if(_angle_min<angle_[i])
            {
                angle_dis_min = std::min( _angle_min-angle_[i-1], angle_[i]-_angle_min);
                break;
            }
        }

        double angle_dis_max = 0;
        if( _angle_max<angle_[0] )
            angle_dis_max = std::min( _angle_max-angle_[num-1]+DOUBLEPI_CONSTANT, angle_[0]-_angle_max);
        else if( _angle_max>angle_[num-1] )
            angle_dis_max = std::min( _angle_max-angle_[num-1], angle_[0]-_angle_max+DOUBLEPI_CONSTANT);
        else for(int i=1; i<num; i++)
        {
            if(_angle_max<angle_[i])
            {
                angle_dis_max = std::min( _angle_max-angle_[i-1], angle_[i]-_angle_max);
                break;
            }
        }
        if( angle_dis_min*2 > max_angle_dis )
        {
            max_angle_dis = angle_dis_min*2;
            best_angle = _angle_min;
        }
        if( angle_dis_max*2 > max_angle_dis )
        {
            max_angle_dis = angle_dis_max*2;
            best_angle = _angle_max;
        }

        dest_angle = best_angle;//在最后才去赋值仅仅是防止可能的多线程冲突而已,意义不大
        return max_angle_dis;
    #undef IsInThresh
}

/// \brief 判断助攻站位点是否阻碍主攻射门（仅在对方半场判断）
bool
AssistRole::isObsActiver(DPoint candidate_point)
{

    bool isobsactiver=false;
    DPoint ball_pos   = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    DPoint left_goal  = world_model_->field_info_.oppGoal_[GOAL_UPPER]+DPoint(0,30);
    DPoint right_goal = world_model_->field_info_.oppGoal_[GOAL_LOWER]+DPoint(0,-30);
    double left_tan = left_goal.angle(ball_pos).radian_;
    double right_tan= right_goal.angle(ball_pos).radian_;

    if(candidate_point.x_<ball_pos.x_)
        isobsactiver=false;
    else if(candidate_point.y_ > ((candidate_point.x_-ball_pos.x_)*left_tan + ball_pos.y_+100)||
            candidate_point.y_ < ((candidate_point.x_-ball_pos.x_)*right_tan + ball_pos.y_-100))//+200
        isobsactiver=false;
    else if(ball_pos.x_ < 0)
        isobsactiver=false;
    else
        isobsactiver=true;

    return isobsactiver;

}
