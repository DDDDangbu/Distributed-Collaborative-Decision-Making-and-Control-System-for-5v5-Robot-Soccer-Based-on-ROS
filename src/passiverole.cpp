#include "nubot_control/passiverole.h"
#define sgn(x) ( ((x)<0) ? (-1) : (1) )

using namespace nubot;

PassiveRole::PassiveRole()
{}

PassiveRole::~PassiveRole()
{}

void
PassiveRole::process()
{
    if(world_model_->CoachInfo_.MatchMode ==STOPROBOT)
    {
        plan_->m_behaviour_.clear();
        return;
    }
    if(world_model_->CoachInfo_.MatchMode == STARTROBOT)
    {
        move2PassivePt();
        return;
    }
}

void PassiveRole::move2PassivePt()
{
    DPoint ball_pos =world_model_->BallInfo_[world_model_->AgentID_-1].getGlobalLocation();
    DPoint robot_pos=world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation();
    Angle  robot_ori=world_model_->RobotInfo_[world_model_->AgentID_-1].getHead();
    Angle  angle_position, delta_ang;

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

    /// 2m以上的距离跑位避障
    if(robot_pos.distance(passive_pt_)>200)
        plan_->move2Positionwithobs(passive_pt_,MAXVEL);
    /// 2m以内不再避障
    else if(robot_pos.distance(passive_pt_)>LOCATIONERROR)
    {
#if 0
        DPoint avoid_vel=DPoint(0,0);
        /// 获得主攻的位置
        DPoint active_pos = plan_->getWhichRolePos(ACTIVE);
        /// 获得排斥速度
        if(active_pos.distance(robot_pos)<150)
            avoid_vel = plan_->repulsive2obstacle(150,active_pos, robot_pos, robot_ori);

        plan_->m_behaviour_.setTarget(passive_pt_,passive_pt_.distance(robot_pos)*MAXVEL/600,Positioned,avoid_vel);
#else
        plan_->m_behaviour_.setTarget(passive_pt_,MAXVEL,Positioned);
#endif
    }

    if(fabs(delta_ang.degree())>ANGLEERROR)
        plan_->m_behaviour_.setOrienation(angle_position.radian_,MAXW,Positioned);
}

/// 计算防守站位点
/// 机器人站在球与球门中心连线上，机器人不能在禁区内，且在我方半场
void PassiveRole::passiveCalculate()
{
    DPoint  fuseBallPos = world_model_->fuseBallInfo_.getGlobalLocation();
    DPoint  fuseBallVel = world_model_->fuseBallInfo_.getVelocity();
    Angle   fuseBallori = fuseBallVel.angle();
#if 0
    DPoint  nearest_opp(0,0);

    if(world_model_->field_info_.isOppField(fuseBallPos))
    {
        if(!getNearestOpp(nearest_opp, DPoint(-700*LENGTH_RATIO,0.0), true))
            passive_pt_ = getPointInline(world_model_->field_info_.ourGoal_[GOAL_MIDDLE], fuseBallPos, 400);
        else
            passive_pt_ = getPointInline(nearest_opp, world_model_->field_info_.ourGoal_[GOAL_MIDDLE], 200);
    }
    else
        passive_pt_ = getPointInline(fuseBallPos, world_model_->field_info_.ourGoal_[GOAL_MIDDLE], 200);
#else
    /// if there is no passive role
    DPoint passive_pos=plan_->getWhichRolePos(PASSIVE);
    DPoint active_pos=plan_->getWhichRolePos(ACTIVE);
    Angle  thetaB2P=passive_pos.angle(fuseBallPos);
    Angle  thetaB2A=active_pos.angle(fuseBallPos);

    float ori_P=fabs((thetaB2P-fuseBallori).radian_)/M_PI;
    float ori_A=fabs((thetaB2A-fuseBallori).radian_)/M_PI;
    float dis_P=passive_pos.distance(fuseBallPos)/(FIELD_LENGTH/4);
    float dis_A=active_pos.distance(fuseBallPos)/(FIELD_LENGTH/4);
    float dis_P2A=passive_pos.distance(active_pos);
    if(passive_pos.x_ ==10000 && passive_pos.y_ == 10000)
    {
        passive_pt_ = passive_pos;
        return;
    }
    /// 球滚动到我方半场，且我方没带上球，防守主动上前，期望能够切换为主攻，但如果距离主攻太近，任然没有切换，避免犯规撤走防守
    if(world_model_->field_info_.isOurField(fuseBallPos)&&!world_model_->IsOurDribble_
       &&((ori_P+dis_P)<(ori_A+dis_A))&&dis_P2A>300)
    {
        passive_pt_=fuseBallPos;
    }
    /// 防守队员站在对方机器人前方，贴紧
    else
    {   /// \brief 寻找到位于我方半场距离球门最近但到球距离不是最近的敌方机器人，这个机器人极有可能是传球对象
        DPoint  fuseBallPos = world_model_->fuseBallInfo_.getGlobalLocation();
        float   less_goaldis=FIELD_LENGTH/2;
        float   less_balldis=FIELD_LENGTH/2;
        int     obs_lab=-1;
        int     ball_lab=-1;

        for(int i=0;i<world_model_->Opponents_.size();i++)
        {
            if(world_model_->field_info_.isOurField(world_model_->Opponents_[i]))
                break;
            else if(i==world_model_->Opponents_.size()-1)
            {
                passive_pt_ = getPointInline(world_model_->field_info_.ourGoal_[GOAL_MIDDLE], fuseBallPos, 400);
                return;
            }
        }

        /// 滤除距离球最近的障碍物
        for(int i=0;i<world_model_->Opponents_.size();i++)
        {
            float dis2ball=fuseBallPos.distance(world_model_->Opponents_[i]);
            if(dis2ball<less_balldis)
            {
                less_balldis=dis2ball;
                ball_lab=i;
            }
        } // ball_lab wil not be -1
        /// 得到距离球门最近的障碍物
        for(int i=0;i<world_model_->Opponents_.size();i++)
        {
            if(i==ball_lab)
                continue;
            float dis2goal=world_model_->field_info_.ourGoal_[GOAL_MIDDLE].distance(world_model_->Opponents_[i]);
            if(world_model_->field_info_.isOurField(world_model_->Opponents_[i])&&dis2goal>100&&dis2goal<less_goaldis)
            {
                less_goaldis=dis2goal;
                obs_lab=i;
            }
        }

        if(obs_lab != -1)       // at least two obs
        {
            DPoint obs_pos = world_model_->Opponents_[obs_lab];
            passive_pt_ = getPointInline(obs_pos, fuseBallPos, 200);
            if(world_model_->field_info_.isOurPenalty(passive_pt_))
                passive_pt_ = getPointInline(obs_pos, fuseBallPos, 150);
            if(world_model_->field_info_.isOurPenalty(passive_pt_))
                passive_pt_ = obs_pos;
        }
        else
        {
            if(world_model_->field_info_.isOurField(fuseBallPos))
            {
                LineSegment Line = LineSegment(fuseBallPos,world_model_->field_info_.ourGoal_[GOAL_MIDDLE]);
                if(active_pos.distance(world_model_->Opponents_[ball_lab])<100 && Line.distance(active_pos,true) < 100)
                    passive_pt_ = getPointInline(fuseBallPos, world_model_->field_info_.ourGoal_[GOAL_MIDDLE], 400);
                else
                    passive_pt_ = getPointInline(fuseBallPos, world_model_->field_info_.ourGoal_[GOAL_MIDDLE], 200);

                if(world_model_->field_info_.isOurGoal(DPoint(passive_pt_.x_-50, passive_pt_.y_)))
                {
                    passive_pt_ = getPointInline(fuseBallPos, world_model_->field_info_.ourGoal_[GOAL_MIDDLE], 100);
//                    passive_pt_ = DPoint(-FIELD_LENGTH/3.0, 0.0);
                }
//                double len = fuseBallPos.distance(active_pos);
//                passive_pt_ = 1 / len * (fuseBallPos - world_model_->Opponents_[ball_lab])  * 200 + fuseBallPos;
            }
            else
                passive_pt_ = getPointInline(world_model_->field_info_.ourGoal_[GOAL_MIDDLE], fuseBallPos, 400);
        }
    }

//    if(passive_pt_.distance(active_pos)<200)
//        passive_pt_.y_=-passive_pt_.y_;
#endif
}
#if 0
bool PassiveRole::getNearestOpp(DPoint &loc, DPoint ref_pt, bool ourfield)
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
#else
//
#endif
DPoint PassiveRole::getPointInline(DPoint start, DPoint end, double dis)
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
int PassiveRole::numInOurPenalty(DPoint notConsiderPt)
{
    int numPenalty = 0;
    for(int i=1; i<OUR_TEAM;i++)
    {
        if(world_model_->RobotInfo_[i].getLocation().x_ != notConsiderPt.x_ &&
                world_model_->RobotInfo_[i].getLocation().y_ != notConsiderPt.y_ &&
                world_model_->field_info_.isOurPenalty(world_model_->RobotInfo_[i].getLocation()))
            numPenalty++;
    }
    return numPenalty;
}

void PassiveRole::sortIncreasing(std::vector<DPoint> &pos_vec, DPoint ref_pt)
{
    std::sort (pos_vec.begin(), pos_vec.end(), Compare(ref_pt));
}
#endif

