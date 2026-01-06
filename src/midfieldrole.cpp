#include "nubot_control/midfieldrole.h"

using namespace nubot;

MidfieldRole::MidfieldRole()
{
    midfield_pt_ = DPoint(0.0,0.0);
}

MidfieldRole::~MidfieldRole()
{}

void
MidfieldRole::process()
{
    if(world_model_->CoachInfo_.MatchMode ==STOPROBOT)
    {
        plan_->m_behaviour_.clear();
        return;
    }
    if(world_model_->CoachInfo_.MatchMode == STARTROBOT)
    {
        move2MidPt();
        return;
    }
    else
        return;
}

void MidfieldRole::move2MidPt()
{
    DPoint robot_pos=world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    DPoint ball_pos=world_model_->fuseBallInfo_.getGlobalLocation();
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

    if(robot_pos.distance(midfield_pt_)>LOCATIONERROR)
        plan_->move2Positionwithobs(midfield_pt_,midfield_pt_.distance(robot_pos)*MAXVEL/600,true);
    if(fabs(delta_ang.degree())>ANGLEERROR)
        plan_->m_behaviour_.setOrienation(angle_position.radian_,fabs(delta_ang.degree())*MAXW/(2*M_PI),Positioned);
}

/*void MidfieldRole::midCalculate()
{
    int pass_line_expand = 80;  //传球线膨胀
    int obs_expand_radii = 80;  //障碍物的膨胀半径
    int tooFarObs = 400;         //去掉太远的障碍物，大于500
    int step_radii = 25;
    int num_angle =  80;   //射线数=360/步长
    float radii_lower = 0;   //圆环 内 半径
    float radii_upper = FIELD_WIDTH;        //圆环 外 半径
    std::map<double,DPoint> pointToChoose;//<(待选点的收益值),(x,y)>

    active_pt_ = plan_->getWhichRolePos(ACTIVE);
    assist_pt_ = plan_->getWhichRolePos(ASSISTANT);
    DPoint midfield_pos = plan_->getWhichRolePos(MIDFIELD);

    double mid_weight = (midfield_pos != DPoint(10000,10000)) ? 0.2 : 0;
    double assist_weight = (assist_pt_ != DPoint(10000,10000)) ? 0.4 : 0;

    //均匀角度产生的n个点
    for(int i=0;i<num_angle;i++) //angle
    {
        float angle_temp = i*DOUBLEPI_CONSTANT/num_angle;
        double profit=0;

        for(float j=radii_lower;j<radii_upper;j+=step_radii)  //radii
        {
            float radii_temp = j;
            //极坐标，场地x为正向
            DPoint pos_temp = DPoint(radii_temp*cos(angle_temp),radii_temp*sin(angle_temp)) + DPoint(10,10);
            //            cout<<"Pos i: x: "<<pos_temp.x_<<" y: "<<pos_temp.y_<<endl;
            if((fabs(pos_temp.x_) > FIELD_LENGTH*0.20) || (fabs(pos_temp.y_) > (FIELD_WIDTH/2 - 100)))
                break; //x方向上去掉大于 FIELD_LENGTH*0.333 的; y方向上去掉距离边界150内的
//            if ( world_model_->IsOurDribble_ && (pos_temp.x_ < 0 ) )
//                break;    //我方带球，中场 pos 到对方半场
//            if ( !world_model_->IsOurDribble_ && (pos_temp.x_ > 0 ))
//                break;    //我方不带球，中场 pos 到己方半场

            if(world_model_->Opponents_.size()==0)
            {
                //计算待选点的收益值，都是距离的函数，需要调试
                profit = -3*fabs(pos_temp.y_ - active_pt_.y_) //尽量在主攻y方向沿线上
                        -0.7*fabs(pos_temp.x_)    //尽量在场地中心
                        -1*fabs(active_pt_.distance(pos_temp) - FIELD_LENGTH*0.3)  //与主攻的距离为场地长的0.22倍比较好,便于传接球；
                        +assist_weight*assist_pt_.distance(pos_temp)   //助攻与中场不能太近
                        -mid_weight*midfield_pos.distance(pos_temp);     //距离当前位置越近越好

                pointToChoose.insert(make_pair(profit,pos_temp));
            }
            //去掉离障碍物很近的点
            else
            {
                float dis2obs_sum=0;
                for(int i=0;i<world_model_->Opponents_.size();i++)
                {

                    if(pos_temp.distance(world_model_->Opponents_.at(i)) < obs_expand_radii)
                        break;

                    double dis2active = pos_temp.distance(active_pt_);
                    //将active的位置向待选点方向推80，需要调试
                    DPoint active_pos_closer = active_pt_ +
                            100.0/dis2active*DPoint(pos_temp.x_-active_pt_.x_, pos_temp.y_-active_pt_.y_);

                    LineSegment active2temp_pos = LineSegment(active_pos_closer,pos_temp);
                    // (x,y) 坐标落在线段"内"的障碍物，计算其到线段的距离;
                    //距离小于pass_line_expand，去掉该点
                    if( (active2temp_pos.decideWhere(world_model_->Opponents_.at(i)) == active2temp_pos.BetweenStartAndEnd) &&
                            (active2temp_pos.distance(world_model_->Opponents_.at(i),true) < pass_line_expand) )
                        break;

                    if(pos_temp.distance(world_model_->Opponents_.at(i)) < tooFarObs)
                        dis2obs_sum += pos_temp.distance(world_model_->Opponents_.at(i));
                    if(i==world_model_->Opponents_.size()-1)
                    {
                        //计算待选点的收益值，都是距离的函数，需要调试
                        profit = -3*fabs(pos_temp.y_ - active_pt_.y_) //尽量在主攻y方向沿线上
                                -0.7*fabs(pos_temp.x_)    //尽量在场地中心
                                -1*fabs(active_pt_.distance(pos_temp) - FIELD_LENGTH*0.3)  //与主攻的距离为场地长的0.22倍比较好,便于传接球；
                                +assist_weight*assist_pt_.distance(pos_temp)   //助攻与中场不能太近
                                -mid_weight*midfield_pos.distance(pos_temp)     //距离当前位置越近越好
                                +0.3*dis2obs_sum;
                        pointToChoose.insert(make_pair(profit,pos_temp));
                    }
                }
            }
        }
    }
    if(pointToChoose.size() != 0)
    {
        std::map<double,DPoint>::iterator mapiter_end = pointToChoose.end();
        mapiter_end--;
        midfield_pt_ = mapiter_end->second;
//        cout<<"mid pos x "<<midfield_pt_.x_<<"  y  "<<midfield_pt_.y_<<endl;
        return;
    }
    else
    {
        midfield_pt_ = midfield_pos;
//        cout<<"mid pos x "<<midfield_pt_.x_<<"  y  "<<midfield_pt_.y_<<endl;
        return;
    }
}*/


void MidfieldRole::midCalculate()
{
    assist_pt_ = plan_->getWhichRolePos(ASSISTANT);
    active_pt_ = plan_->getWhichRolePos(ACTIVE);
    DPoint  fuseBallPos = world_model_->fuseBallInfo_.getGlobalLocation();

    static DPoint pre_ball_pos=DPoint(0,0);
    static bool mid_can_move=true;
    if(world_model_->CoachInfo_.MatchMode == OUR_KICKOFF)   // prevent the midfield robot to cross through two robots when kickoff
    {
        mid_can_move=false;
        pre_ball_pos=fuseBallPos;
    }
    else if(world_model_->CoachInfo_.MatchType == OUR_KICKOFF && world_model_->CoachInfo_.MatchMode == STARTROBOT && mid_can_move==false)
    {
        double tmp_dis=pre_ball_pos.distance(fuseBallPos);
        if(tmp_dis>200)
            mid_can_move=true;
        else
            midfield_pt_=DPoint(-0.15 * FIELD_LENGTH, 0.0);
    }
    else
    {
        DPoint nearest_opp(-10000,0);
        getNearestOpp(nearest_opp, DPoint(0,0), false);
        if(-10000 != nearest_opp.x_)
            midfield_pt_ = getPointInline(nearest_opp, fuseBallPos, 150);
        else
            midfield_pt_ = DPoint(-100,0);
        //printf("nearest opp:(%.1f, %.1f)\n", nearest_opp.x_, nearest_opp.y_);

        // 以下为进一步修正或处理
        if(midfield_pt_.distance(active_pt_) < 150.0)
        {
            if(midfield_pt_.y_ > 0.0)
            {
                midfield_pt_ = midfield_pt_.rotate(Angle(M_PI/2.0, true));        // 这个rotate函数写的旋转方向和数学通用的相反，当角度为正的时候为顺时针转而不是逆时针转！
                //printf("clockwise\n");
            }
            else
            {
                midfield_pt_ = midfield_pt_.rotate(Angle(-M_PI/2.0, true));
                //printf("counterclockwise\n");
            }
        }
    }
}

bool MidfieldRole::getNearestOpp(DPoint &loc, DPoint ref_pt, bool ourfield)
{
    /** 计算距离x坐标x_thes最近的对手，若存在任一对手在己方半场，则返回1,否则返回0 */
    /** 若our_field=1，则返回在我方半场的离x_thres最近的对手，否则返回全 */
    /** 场离x_thres最近的对手                                       */
    bool isOppInOurField = false;
    double min_dis = 10000;
    double x_thres = ref_pt.x_;

    for(DPoint opp : world_model_->fuseOpps_)
    {
        if(world_model_->field_info_.isOurField(opp))
            isOppInOurField = true;

        double dis = opp.distance(ref_pt);
        if(opp.x_ > x_thres &&  dis < min_dis)
        {
            if((ourfield && world_model_->field_info_.isOurField(opp)) ||
                    !ourfield)
            {
                loc = opp;
                min_dis = dis;
            }
        }
    }
    return isOppInOurField;
}

DPoint MidfieldRole::getPointInline(DPoint start, DPoint end, double dis)
{
    DPoint vec = end - start;
    double length = vec.length();
    if(length < 1e-3)
        length += 1;

    if(dis < length)
        return dis/length * vec + start;
    else
    {
        if(start.y_>0.0)
            vec = world_model_->field_info_.ourGoal_[GOAL_LOWER] - start;
        else
            vec = world_model_->field_info_.ourGoal_[GOAL_UPPER] - start;

        length = vec.length();
        if(length < 1e-3)
            length += 1;
        return dis/length * vec + start;
    }
}

#if 0
//计算周围是否有障碍物
bool
MidfieldRole::isNullInTrap(const DPoint & robot_pos, const std::vector<DPoint> & obs_pts)
{
    int nums_pts = 4;
    std::vector<DPoint> pts;
    pts.resize(nums_pts);
    pts[0] = robot_pos+DPoint(-50,-50);
    pts[1] = robot_pos+DPoint(-50,50);
    pts[2] = robot_pos+DPoint(50,50);
    pts[3] = robot_pos+DPoint(50,-50);

    bool isInPoly= false;
    int nums_obs_pts = obs_pts.size();
    for(int k = 0 ; k < nums_obs_pts ; k++ )
    {
        DPoint  test_pt = obs_pts[k];
        int minX(100000),maxX((-100000)),maxY(-100000),minY((100000));
        for(std::size_t i = 0; i <nums_pts ;i++)
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
        /** 已经可以说明点在多边形外部,继续下一个循环*/
        if (test_pt.x_ < minX || test_pt.x_ > maxX || test_pt.y_ < minY || test_pt.y_ > maxY)
            continue;
        bool c =false;
        int i, j;
        for (i = 0, j = nums_pts-1; i < nums_pts; j = i++)
        {
            if ( ( (pts[i].y_>test_pt.y_) != (pts[j].y_>test_pt.y_) ) &&
                 (test_pt.x_ < (pts[j].x_-pts[i].x_) * (test_pt.y_-pts[i].y_)/double((pts[j].y_-pts[i].y_))+ pts[i].x_) )
                c = !c;
        }
        isInPoly = c;
        if(isInPoly)
            break;
    }
    return !isInPoly;
}
#endif
#if 0
//我方不带球时的中场站位
void
MidfieldRole::asPassiver()
{
    DPoint robot_pos=world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    DPoint ball_pos=world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    Angle  robot_ori=world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();

    midfield_pt_=robot_pos;
    midfield_ori_=robot_ori;                        //如果找不到站位点呆在原地不动

    std::vector<nubot::DPoint> candidate_pos;                  //存放候选接球点

    //因为定位误差是30cm，所以以50cm间隔取点，得到以现在足球位置为中心的矩形区域
    DPoint candidate_point;
    DPoint candidate_first=DPoint(-500,-300);                 //第一个点

    for(int i=0;i<9;i++)                                              //找到备选点
    {
        for(int j=0;j<13;j++)
        {
            candidate_point=candidate_first+DPoint(i*50,j*50);
            double distance = ball_pos.distance(candidate_point);
            bool isObsOpprole = isObsOpp(candidate_point);
            if(isObsOpprole && distance > 200)      //最佳站位点，且阻挡对方机器人射门
                candidate_pos.push_back(candidate_point);          //得到一系列的备选点
        }
    }
    int num_candidate=candidate_pos.size();
    if(num_candidate>0)
    {
        double dis2ball=ball_pos.distance(candidate_pos[0]);

        int choose_num=0;
        for(int i=1;i<num_candidate;i++)
        {
            if(ball_pos.distance(candidate_pos[i]) < dis2ball)
            {
                dis2ball=ball_pos.distance(candidate_pos[i]);
                choose_num=i;
            }
        }
        midfield_pt_=candidate_pos[choose_num];
        midfield_ori_=ball_pos.angle(robot_pos);
    }
    plan_->move2Positionwithobs(midfield_pt_,MAXVEL,true);
    plan_->m_behaviour_.setOrienation(midfield_ori_.radian_,MAXW,Positioned);
}
#endif
#if 0
//我方带球进攻时中场站位
void
MidfieldRole::asAssist()
{
    DPoint robot_pos=world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    DPoint ball_pos=world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    Angle  robot_ori=world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();

    midfield_pt_=robot_pos;
    midfield_ori_=robot_ori;                        //如果找不到站位点呆在原地不动
    std::vector<nubot::DPoint> candidate_pos;                  //存放候选接球点
    DPoint opp_goal_mid=world_model_->field_info_.oppGoal_[GOAL_MIDDLE];

    //因为定位误差是30cm，所以以50cm间隔取点，得到以现在机器人位置为中心的矩形区域
    DPoint candidate_point;
    DPoint candidate_first=DPoint(-250,-300);

    for(int i=0;i<9;i++)                                              //找到备选点
    {
        for(int j=0;j<13;j++)
        {
            candidate_point=candidate_first+DPoint(i*50,j*50);
            double distance = ball_pos.distance(candidate_point);
            bool isNullAroundRobot = isNullInTrap(candidate_point,world_model_->Obstacles_);     //判断周围是否有障碍物
            bool isObsActiverole = isObsActiver(candidate_point);
            if(isNullAroundRobot && !isObsActiverole && distance > 300)
                candidate_pos.push_back(candidate_point);          //得到一系列的备选点
        }
    }

    int num_candidate=candidate_pos.size();

    double dis2goal;                              //备选点到球门的距离
    double dis2ball;                              //备选点到球的距离
    double energy_o;
    std::vector<double> energy;                   //启发函数值
    int    choose_num=0;                          //选择的备选点
    int    near_obs=0;                            //临近的障碍物个数
    double max_energy=0.0;
    double ldetAng_shoot;
    double rdetAng_shoot;
    if(num_candidate>0)
    {
        for(int i=0;i<num_candidate;i++)
        {
            dis2goal=candidate_pos[i].distance(opp_goal_mid);
            dis2ball=candidate_pos[i].distance(ball_pos);
            energy_o=fabs(candidate_pos[i].y_)<100 ? 0.1: 0.3*600/(dis2goal+dis2ball);//到球门距离的启发函数项

            ldetAng_shoot = world_model_->field_info_.oppGoal_[GOAL_UPPER].angle(candidate_pos[i]).radian_;
            rdetAng_shoot = world_model_->field_info_.oppGoal_[GOAL_LOWER].angle(candidate_pos[i]).radian_;
            energy_o += 0.1 * fabs(ldetAng_shoot-rdetAng_shoot);                         //射门角度的启发函数项

            LineSegment Line2goal = LineSegment(candidate_pos[i],opp_goal_mid);
            LineSegment Line2ball = LineSegment(candidate_pos[i],ball_pos);
            near_obs = 0;
            for(int j=0 ; j < world_model_->Obstacles_.size() ; j++)
                if(Line2goal.distance(world_model_->Obstacles_[j],true)<100||Line2ball.distance(world_model_->Obstacles_[j],true)<100)
                    near_obs++;
            if(near_obs>2)
                near_obs=2;
            energy_o += 0.15 * (2-near_obs)+0.1;                                        //障碍物的启发函数项

            energy_o += 0.0002 *(300-robot_pos.distance(candidate_pos[i]));                                                            //避免大范围转移，离目前的点越近效能值越高
            energy.push_back(energy_o);
        }
        for(int i=0;i<energy.size();i++)
        {
            if(energy[i]>max_energy)
            {
                max_energy=energy[i];
                choose_num=i;
            }
        }
        midfield_pt_=candidate_pos[choose_num];
        midfield_ori_=ball_pos.angle(robot_pos);
    }
    plan_->move2Positionwithobs(midfield_pt_,MAXVEL,true);
    plan_->m_behaviour_.setOrienation(midfield_ori_.radian_,MAXW,Positioned);
}
#endif
#if 0
bool
MidfieldRole::isObsOpp(DPoint candidate_point)
{

    bool isobsopp=false;
    DPoint ball_pos   = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    DPoint left_goal  = world_model_->field_info_.ourGoal_[GOAL_UPPER]+DPoint(0,30);;
    DPoint right_goal = world_model_->field_info_.ourGoal_[GOAL_LOWER]+DPoint(0,-30);;
    double left_tan = left_goal.angle(ball_pos).radian_;
    double right_tan= right_goal.angle(ball_pos).radian_;

    if(ball_pos.x_>0)
        isobsopp=true;
    else if(candidate_point.x_>ball_pos.x_)
        isobsopp=false;
    else if(candidate_point.y_ > (candidate_point.x_-ball_pos.x_)*left_tan + ball_pos.y_+200||
            candidate_point.y_ < (candidate_point.x_-ball_pos.x_)*right_tan + ball_pos.y_-200)
        isobsopp=false;
    else
        isobsopp=true;

    return isobsopp;
}
#endif
#if 0
bool
MidfieldRole::isObsActiver(DPoint candidate_point)
{
    bool isobsactiver=false;
    DPoint ball_pos   = world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    DPoint left_goal  = world_model_->field_info_.oppGoal_[GOAL_UPPER]+DPoint(0,30);     //为了保证不阻挡主攻射门，区域放大了一个误差长度
    DPoint right_goal = world_model_->field_info_.oppGoal_[GOAL_LOWER]+DPoint(0,-30);
    double left_tan = left_goal.angle(ball_pos).radian_;
    double right_tan= right_goal.angle(ball_pos).radian_;

    if(candidate_point.x_<ball_pos.x_)
        isobsactiver=false;
    else if(candidate_point.y_ > ((candidate_point.x_-ball_pos.x_)*left_tan + ball_pos.y_+200)||
            candidate_point.y_ < ((candidate_point.x_-ball_pos.x_)*right_tan + ball_pos.y_-200))
        isobsactiver=false;
    else if(ball_pos.x_ < 0)
        isobsactiver=false;
    else
        isobsactiver=true;

    return isobsactiver;
}
#endif
