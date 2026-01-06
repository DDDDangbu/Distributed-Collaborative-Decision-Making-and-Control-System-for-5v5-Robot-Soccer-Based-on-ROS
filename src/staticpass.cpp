#include "nubot_control/staticpass.h"
using namespace nubot;
using namespace std;

StaticPass::StaticPass()
{
    isPosition_=false;
    m_nCanBeInPenaltyArea=0;
    m_nPassNumber_=-1;
    m_nCatchNumber_=-1;
    ballNumber_=-1;
    ballPos_=DPoint(0,0);
    backFieldPoint_=DPoint(-600,0);
    for(int i=0;i<OUR_TEAM;i++)
        isAllocation_[i]=false;

    targetInitialize();
    target_=DPoint(0,0);                            //为分配的目标点
    m_nPassNumber_=0;
    m_nCatchNumber_=0;

    fuseOpps_.reserve(10);
    lscp_.reserve(2);                               // 球与机器人的连线线段与大禁区边界交点最多两个
    oppPos_ = DPoint(0,0);                          // fuseOpps_中的其中一个元素
    k_ = RADIUS*RADIUS;
    preTarget_ = DPoint(0,0);
    index_ = 0;
}

StaticPass::~StaticPass()
{}

void StaticPass::staticReady_()                    //判断何种站位
{
    fuseBall_();
    fuseOpp_();

    switch (world_model_->CoachInfo_.MatchMode)
    {
    case OUR_KICKOFF:
        OurkickoffReady_();
        break;
    case OPP_KICKOFF:
        OppkickoffReady_();
        break;
    case OUR_FREEKICK:
        OurDefaultReady_();
        break;
    case OPP_FREEKICK:
        OppDefaultReady_();
        break;
    case OUR_GOALKICK:
        OurDefaultReady_();
        break;
    case OPP_GOALKICK:
        OppGoalReady_();
        break;
    case OUR_CORNERKICK:
        OurDefaultReady_();
        break;
    case OPP_CORNERKICK:
        OppCornerReady_();
        break;
    case OUR_THROWIN:
        OurDefaultReady_();
        break;
    case OPP_THROWIN:
        OppDefaultReady_();
        break;
    case OUR_PENALTY:
        OurPenaltyReady_();
        break;
    case OPP_PENALTY:
        OppPenaltyReady_();
        break;
    case DROPBALL:
        DropBallReady_();
        break;
    default:
        break;
    }
}

bool StaticPass::islargestNum_()
{
    largestNum_=-1;
    for(int i=0;i<OUR_TEAM;i++)
        if(world_model_->RobotInfo_[i].isValid() && i>largestNum_)
            largestNum_=i;
    if(largestNum_==world_model_->AgentID_-1)
        return true;
    else
        return false;
}

bool StaticPass::fuseBall_()                       //距离球最近的机器人看到的足球位置为最后的位置
{
    float distance_min = 2000.0;
    float distance = distance_min;
    ballNumber_=-1;

    for(int i=0;i<OUR_TEAM;i++)
        if(world_model_->BallInfo_[i].isLocationKnown() && world_model_->RobotInfo_[i].isValid())
        {
            distance=world_model_->BallInfo_[i].getGlobalLocation().
                     distance(world_model_->RobotInfo_[i].getLocation());
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
        ballPos_=world_model_->BallInfo_[ballNumber_].getGlobalLocation();
        return true;
    }
}

bool StaticPass::isOurRobot(DPoint tmp)
{
    for(int i =0; i<OUR_TEAM; i++)
    {
        DPoint pos = world_model_->RobotInfo_[i].getLocation();
        if(fabs(pos.x_ - tmp.x_)<50 && fabs(pos.y_-tmp.y_)<50)
            return true;
    }
    return false;
}

bool StaticPass::fuseOpp_()                 //融合结果为距离足球最近的机器人看到的对方机器人位置
{
    bool is_valid_opp = true;
    fuseOpps_.clear();
    for(DPoint pos : world_model_->Opponents_)
    {
        pos.x_ = round(pos.x_/10.0)*10.0;
        pos.y_ = round(pos.y_/10.0)*10.0;
        is_valid_opp = true;
        // 排除对方守门员且排除可能是因为卡尔曼滤波的小bug出现的（0,0)的干扰
        if( (!field_.isOppGoal(pos)) && !(pos.x_==0.0 && pos.y_==0.0)
                && field_.isInInterField2(pos, -100, -30) )//&& !isOurRobot(pos))     // 不防守太靠后的对手
        {
            for(DPoint pt : fuseOpps_)
                if(pos.distance(pt) < COMBINE_OPP_DIS)
                {
                    is_valid_opp = false;
//                    ROS_INFO("DISTACE LESS THAN 50");
                }

            if(is_valid_opp)
                fuseOpps_.push_back(pos);
            //ROS_INFO("opp:%f %f",pos.x_, pos.y_);
        }
    }
    return true;
}

int StaticPass::getNearestRobot_(DPoint Pt)         //找到距离目标点最近的机器人
{
    float distance_min;
    float distance;
    int robot_num=-1;
    distance_min=2000.0;
    for(int i=1;i<OUR_TEAM;i++)                                    //守门员不算
        if(world_model_->RobotInfo_[i].isValid() && !isAllocation_[i])                   //机器人在场
        {
            distance=Pt.distance(world_model_->RobotInfo_[i].getLocation());
            if(distance<distance_min)
            {
                distance_min=distance;
                robot_num=i;
            }
        }
     if(robot_num==-1)
         return OUR_TEAM;
     else
     {
         isAllocation_[robot_num]=true;
         return robot_num;                                         //返回值1~4,对应2~5号机器人
     }
}

void StaticPass::targetReorder(vector<DPoint> &pts)
{
    DPoint ref_pt = ballPos_;
    for(int i=0; i<pts.size();i++)
    {
        for(int j=i+1;j<pts.size();j++)
        {
            if( pts[j].distance(ref_pt) < pts[i].distance(ref_pt))
            {
                DPoint tmp = pts[i];
                pts[i] = pts[j];
                pts[j] = tmp;
            }
        }
    }
}

DPoint StaticPass::targetAdjust(DPoint tmp)       //防止机器人出界
{
    int nNoLoop=100;                                  //控制次数
    double tmpAng=2*Ang2Rad;
    //超出上边界，往外推1m
    while(tmp.y_>FIELD_YLINE1+100)
    {
        //每次逆时针旋转10度
        tmp=tmp.rotate(ballPos_,Angle(tmpAng));
        nNoLoop--;
        if(nNoLoop<0)
            break;
    }
    nNoLoop=100;
    //超出下边界
    while(tmp.y_<FIELD_YLINE6-100)
    {
        tmp=tmp.rotate(ballPos_,Angle(-tmpAng));
        nNoLoop--;
        if(nNoLoop<0)
            break;
    }
    nNoLoop=100;
    //左边界
    //在左边界之外或者在禁区里
    while (tmp.x_<FIELD_XLINE7||(tmp.x_<FIELD_XLINE5 && tmp.y_<FIELD_YLINE2 && tmp.y_>FIELD_YLINE5))
    {
         //禁区名额未用完
        if((tmp.x_<FIELD_XLINE5 && tmp.y_<FIELD_YLINE2 && tmp.y_>FIELD_YLINE5)&&(m_nCanBeInPenaltyArea>0))
        {
            //球在禁区右边
            if(ballPos_.x_>FIELD_XLINE5 && ballPos_.y_<FIELD_YLINE2 && ballPos_.y_>FIELD_YLINE5)
            {
                tmp.x_=200*LENGTH_RATIO;
                tmp.y_=ballPos_.y_;
            }
            else
            {
                if(ballPos_.y_>FIELD_YLINE2)
                {
                    if(ballPos_.x_>FIELD_XLINE5)
                        tmp.x_=-700*LENGTH_RATIO;
                    else
                        tmp.x_=ballPos_.x_;
                    tmp.y_=300*WIDTH_RATIO;
                }
                if(ballPos_.y_<FIELD_YLINE5)
                {
                    if(ballPos_.x_>FIELD_XLINE5)
                        tmp.x_=-700*LENGTH_RATIO;
                    else
                        tmp.x_=ballPos_.x_;
                    tmp.y_=-300*WIDTH_RATIO;
                }
            }
            break;
        }
        if(ballPos_.y_>0)
            tmp=tmp.rotate(ballPos_,Angle(tmpAng));
        else
            tmp=tmp.rotate(ballPos_,Angle(-tmpAng));
        nNoLoop--;
        if(nNoLoop<0)
            break;
    }
    nNoLoop=100;
    //右边界
    while(tmp.x_>FIELD_XLINE1||(tmp.x_>FIELD_XLINE3 && tmp.y_>FIELD_YLINE5 && tmp.y_<FIELD_YLINE2))
    {
        if(ballPos_.y_>0)
            tmp=tmp.rotate(ballPos_,Angle(-tmpAng));
        else
            tmp=tmp.rotate(ballPos_,Angle(tmpAng));
        nNoLoop--;
        if(nNoLoop<0)
            break;
    }
    return tmp;
}

DPoint StaticPass::defendTargetAdjust(DPoint pt_in, DPoint ref_pt)
{
    int nNoLoop=100;                                  //控制次数
    double tmpAng=2*Ang2Rad;

    if(!OptimizedinOurPenalty(pt_in))
    {
        // 出界的站位点的处理
        if(field_.isOutBorder(LEFTBORDER, pt_in) ) //在左边界之外
            pt_in = calOptPos(ballPos_, ref_pt, ballPos_ + DPoint(100,0),2*RADIUS*RADIUS);
        else if(field_.isOutBorder(DOWNBORDER, pt_in)) //超出下边界
            pt_in = calOptPos(ballPos_, ref_pt,ballPos_ + DPoint(0,100),2*RADIUS*RADIUS);
        else if(field_.isOutBorder(UPBORDER, pt_in))   //超出上边界
            pt_in = calOptPos(ballPos_, ref_pt,ballPos_+ DPoint(0,-100),2*RADIUS*RADIUS);
        else if(field_.isOutBorder(RIGHTBORDER, pt_in) )    //在右边界之外，根据公式，这种情况不太可能发生，如果发生，强制让它回去。
            pt_in = calOptPos(ballPos_, ref_pt,ballPos_+ DPoint(-100,0),2*RADIUS*RADIUS);
    }

    // 未出界的站位点的处理（是否在大禁区内）
    if(field_.isOurPenalty(pt_in))
    {
        if(numInPenaltyArea(OUR_PENALTY_SIDE) == 0)     // 机器人可以在自己的大禁区内而且与球的距离不受限制。利用这个规则来调整站位点。
            OptimizedinOurPenalty(pt_in);
        else
        {
            nNoLoop=100;
            while(field_.isOurPenalty(pt_in))
            {
                pt_in = pt_in.rotate(ballPos_, Angle(tmpAng)); // RotAroundBallOpt(pt_in, tmpAng);
                nNoLoop--;
                if(nNoLoop<0)
                    break;
            }
        }
    }
    else if(field_.isOppPenalty(pt_in)) //在对方禁区里
    {
        ROS_INFO("in opp penalty! pt:(%.0f, %.0f)", pt_in.x_, pt_in.y_);
        if(numInPenaltyArea(OPP_PENALTY_SIDE) != 0 || field_.isOppGoal(pt_in))   //机器人不可以在禁区内
        {
            nNoLoop=100;
            while(field_.isOppPenalty(pt_in))
            {
                pt_in = pt_in.rotate(ballPos_, Angle(tmpAng)); // RotAroundBallOpt(pt_in, tmpAng);
                nNoLoop--;
                ROS_INFO("%d: adjust:(%.0f, %.0f)", nNoLoop, pt_in.x_, pt_in.y_);
                if(nNoLoop<0)
                    break;
            }
        }
        ROS_INFO("adjust pt:(%.0f, %.0f)", pt_in.x_, pt_in.y_);
    }

    return pt_in;
}

/** 根据规则，mode == OPP_CORNERKICK || mode == OPP_THROWIN || mode == OPP_FREEKICK **/
/** 的时候，机器人可以在自己的大禁区内而且与球的距离不受限制。利用这个规则来调整站位点。        **/
bool StaticPass::OptimizedinOurPenalty(DPoint &tmp)
{
    char mode = world_model_->CoachInfo_.MatchMode;
    DPoint init_pt = tmp;
    if(!field_.isOurPenalty(tmp))
        init_pt = DPoint(FIELD_XLINE7+100.0,0.0);       // assume it is inside our penalty area
                                        // TODO: 添加一个判断：如果tmp与足球距离比RADIUS大得较多，那么tmp这个点就是来盯防的，不需要优化到大禁区内
    if(!numInPenaltyArea(OUR_PENALTY_SIDE))   //机器人可以在禁区内
    {
        //除goalkick外，以下几种模式在大禁区内没有距离球3米的限制 ^_^
        if(mode == OPP_CORNERKICK || mode == OPP_THROWIN || mode == OPP_FREEKICK)
        {
            lscp_.clear();
            LineSegment ball_tmp(ballPos_, init_pt);

            for(int i=0;i<3;i++)
                if(ball_tmp.crosspoint(field_.ourPenaltyLine_[i],init_pt))
                    lscp_.push_back(init_pt);
            if(lscp_.size() == 0)
                return 0;               // 没有交点
            else if(lscp_.size() == 1)
                init_pt = lscp_.at(0);
            else
                init_pt = (lscp_.at(0).distance(ballPos_)<lscp_.at(1).distance(ballPos_) )?
                            lscp_.at(0) : lscp_.at(1);

            //接下来让站位点再往禁区里面走一点，防止误差导致机器人实际上不在禁区，造成犯规
            DPoint vec = field_.ourGoal_[GOAL_MIDDLE] - init_pt;
            double dis=20;                  // 往里走dis厘米
            if(vec.length() != 0.0)
            {
                init_pt = init_pt + dis/vec.length() * vec;
            }
            else        // 不可能进入这种情况，但是为了严谨，还是写了。
            {
                ROS_ERROR("vec.norm == 0! 分母为0!");
                return 0;
            }

            if( field_.isOurPenalty(tmp) ||
                (!field_.isOurPenalty(tmp) && (init_pt.distance(ballPos_) < tmp.distance(ballPos_))) )
            {
                tmp = init_pt;
                return 1;
            }
            else
                return 0;
        }
        else
            return 0;
    }
    else
        return 0;
}

/** curren_pos: 机器人当前位置; rad: 旋转的弧度， 始终为正               **/
/** 程序自动根据机器人当前位置使得机器人绕着足球旋转，一方面不会靠近足球       **/
/** 另一方面尽量不出界（往中间走）。第一、三象限顺时针转，第二、四象限逆时针转  **/
/** 该函数不是很准确，还需要进一步思考 FIXME **/
DPoint StaticPass::RotAroundBallOpt(DPoint curren_pos, double rad)
{
    int symbol = -sgn(ballPos_.x_)*sgn(ballPos_.y_);
    DPoint  new_pos = curren_pos.rotate(ballPos_,Angle(symbol*rad));
    return new_pos;
}

DPoint StaticPass::collisionAvoid_(DPoint tmp)     //避免机器人的碰撞
{
    for(int i=1;i<OUR_TEAM;i++)                                       //不管守门员
    {
        if(tmp!=targetInit_[i] && tmp.distance(targetInit_[i])<100)    //除去自己，和其他位置距离小于100的
        {
            tmp = tmp.rotate(ballPos_,Angle(30*Ang2Rad));
            ROS_INFO("[%f %f] adjust", tmp.x_, tmp.y_);
            break;
        }
    }
    return tmp;
}

DPoint StaticPass::calOptPos(DPoint ball, DPoint opp, DPoint direction, double k) // k的值由动态参数获得
{
    DPoint pos(0,0);
#if 1
    /** 机器人与足球的射线A以及A旋转90度后的射线B形成的角，     **/
    /** 该角平分线 与以足球为圆心，3m为半径的圆的交点即为站位点. **/
    Circle  ball_circle(OppDefaultDis, ball);
    Line_   ball_opp(opp, ball);
    vector<DPoint> cp; cp.reserve(2);
    DPoint cp1(0,0), cp2(0,0), cp3(0,0), mid_cp(0,0);
    cp = ball_circle.crosspoint(ball_opp);
    if(cp.size() == 2)
        cp1 = cp[0].distance(opp) < cp[1].distance(opp) ? cp[0] : cp[1];
    else
    {
        ROS_INFO("1. no two crosspoint");
        cp1 = ball + DPoint(330,0);
    }

    DPoint  dire_pt = direction;
    Line_   ball_direc(ball, dire_pt);
    cp.clear();
    cp = ball_circle.crosspoint(ball_direc);
    if(cp.size() == 2)
        cp2 = cp[0].distance(dire_pt) < cp[1].distance(dire_pt) ? cp[0] : cp[1];
    else
    {
        ROS_INFO("2. no two crosspoint");
        cp2 = ball - DPoint(330,0);
    }

    mid_cp = 0.5 * (cp1 + cp2);
    Line_ ball_mid(ball, mid_cp);
    cp.clear();
    cp = ball_circle.crosspoint(ball_mid);
    if(cp.size() == 2)
        cp3 = cp[0].distance(opp) < cp[1].distance(opp) ? cp[0] : cp[1];
    else
    {
        // ROS_INFO("3. no two crosspoint");
        cp3 = ball + DPoint(0,330);
    }

    return cp3;
#else
    /** 根据与对手机器人的距离以及我方机器人朝向来求得的最优站位点，**/
    /** 最小化d=|AR|^2 + 2*r^2*(1-cos(theta))              **/
    /** 但是得到的函数不光滑，导致站位点变化剧烈，需要改进        **/
    double xb=ball.x_, yb=ball.y_, x0=opp.x_, y0=opp.y_;
    double u=direction.x_, v=direction.y_;
    direction = direction.norm() == 1.0 ? direction : 1/direction.norm()*direction;   // 归一化

    double deno = sqrt( (2*OppDefaultDis*(x0-xb)-k*u)*(2*OppDefaultDis*(x0-xb)-k*u) +
                       (2*OppDefaultDis*(y0-yb)-k*v)*(2*OppDefaultDis*(y0-yb)-k*v));
    if(fabs(deno) < 1e-3)       // deno == 0
    {
        k = 2*OppDefaultDis*OppDefaultDis + OppDefaultDis;   // let k be bigger
        deno = sqrt( (2*OppDefaultDis*(x0-xb)-k*u)*(2*OppDefaultDis*(x0-xb)-k*u) +
                     (2*OppDefaultDis*(y0-yb)-k*v)*(2*OppDefaultDis*(y0-yb)-k*v));
        ROS_ERROR("denominator = 0! So change k to %f and re-calculate",k);
    }

    double cosphi = (2*OppDefaultDis*(x0-xb)-k*u)/deno;
    double sinphi = (2*OppDefaultDis*(y0-yb)-k*v)/deno;
    pos.x_ = OppDefaultDis*cosphi + xb;
    pos.y_ = OppDefaultDis*sinphi + yb;
#endif
    return pos;
}

int StaticPass::numInPenaltyArea(int which_side)
{
    int num=0;
    switch(which_side)
    {
        case OUR_PENALTY_SIDE:
        {
            for(int i=1; i<=index_; i++)
                if( field_.isOurPenalty(targetInit_[i]))        //除了本机器人外是否有其他机器人在大禁区
                    num++;
            return num;
            break;
        }
        case OPP_PENALTY_SIDE:
        {
        for(int i=1; i<=index_; i++)
            if( field_.isOppPenalty(targetInit_[i]) )
                num++;
            return num;
            break;
        }
        default:
        {
            ROS_ERROR("which penalty side do you specify??");
            return 100;
        }
    }
}

void StaticPass::OurDefaultReady_()                   //我方发球默认的站位
{
    m_nCanBeInPenaltyArea=5;
    //假设球靠上，计算位置
    DPoint tmpball = ballPos_;
    static bool flipField=true;
    if(ballPos_.y_<-50)
        flipField=true;
    if(ballPos_.y_>50)
        flipField=false;
    if(flipField)
        tmpball.y_=-tmpball.y_;
    DPoint tmp;
    DPoint posCatch;
    DPoint posPass;
    static bool bCatchOnMidLine;
    bCatchOnMidLine=false;

    if(world_model_->CoachInfo_.id_A==1)
    {
        static bool IsInOurArea=tmpball.x_<250? true:false;              //划分半场用的是250
        if(tmpball.x_>270)
            IsInOurArea=false;
        if(tmpball.x_<230)
            IsInOurArea=true;

        //根据球所在的半场来确定开球策略
        if(IsInOurArea)
        {
            //球处于我方半场，处理时尽量大脚转移，接球队员和球以及对方球门在一条直线上
            tmp=tmpball.pointofline(DPoint(700*LENGTH_RATIO,0.0),TAC_DIST_CATCH);
            targetInit_[2]=tmp;
            posCatch=tmp;
            if(abs(tmp.x_)<50)         //中场附近接球，需要对接球的位置进行调整
                bCatchOnMidLine=true;
            if(bCatchOnMidLine==true)
            {
                tmp=tmpball.pointofline(DPoint(700.0*LENGTH_RATIO,tmpball.y_),300);
                targetInit_[2]=tmp;
                posCatch=tmp;
            }
            //传球队员与球以及球门在一条直线上
            tmp=tmpball.pointofline(posCatch,-TAC_DIST_PASS);
            targetInit_[1]=tmp;
            posPass=tmp;
            //另一个队员
            tmp=tmpball.pointofline(targetInit_[2],400*WIDTH_RATIO);
            tmp=tmp.rotate(tmpball,Angle(-30*Ang2Rad));           //绕tmpball顺时针旋转30度
            targetInit_[3]=tmp;
            if(bCatchOnMidLine==true)
            {
                tmp=tmpball.pointofline(targetInit_[2],500*WIDTH_RATIO);
                tmp=tmp.rotate(tmpball,Angle(-10*Ang2Rad));           //绕tmpball顺时针旋转10度
                targetInit_[3]=tmp;
            }
        }
        else
        {
            //球处于对方半场，争取直接射门，接球进攻队员和球以及对方球门在一条直线上
            tmp=tmpball.pointofline(field_.oppGoal_[GOAL_MIDDLE],-TAC_DIST_CATCH);
            targetInit_[2]=tmp;
            posCatch=tmp;

            int nodead=0;
            while (fabs(posCatch.y_)>400*WIDTH_RATIO)          //接球机器人位置修正，避免出界
            {
                if(posCatch.y_>0)
                    posCatch=posCatch.rotate(tmpball,Angle(5*Ang2Rad));
                else if(posCatch.y_<0)
                    posCatch=posCatch.rotate(tmpball,Angle(-5*Ang2Rad));
                nodead ++;
                if(nodead>100)
                    break;
            }
            targetInit_[2]=posCatch;

            tmp=tmpball.pointofline(posCatch,-TAC_DIST_PASS);
            targetInit_[1]=tmp;
            posPass=tmp;

            if(tmpball.x_>700*LENGTH_RATIO)            //角球区域
            {
                tmp=posCatch.pointofline(field_.ourGoal_[GOAL_MIDDLE],350*WIDTH_RATIO);
                tmp=tmp.rotate(tmpball,Angle(40*Ang2Rad));
            }
            else
            {
                tmp=tmpball.pointofline(field_.ourGoal_[GOAL_MIDDLE],350*WIDTH_RATIO);
                tmp=tmp.rotate(tmpball,Angle(45*Ang2Rad));
            }
            targetInit_[3]=tmp;
        }

        //修正跑动位置，避免与球的距离太近
        if(ballPos_.distance(backFieldPoint_)>300)                 //后卫点backfieldpoint
            targetInit_[4]=backFieldPoint_;
        else
            targetInit_[4]=backFieldPoint_+DPoint(0.0,-300*WIDTH_RATIO);
        targetInit_[4]=collisionAvoid_(targetInit_[4]);            //直接得到的定点，由于不能根据球的位置动态变化，要避免与其他点的冲突

        if(targetInit_[2].x_<100)
            targetInit_[3].x_=200;                   //发球过后还得动态传球到达对方半场

        if(flipField)            //镜像变化
        {
            targetInit_[1].y_=-targetInit_[1].y_;
            targetInit_[2].y_=-targetInit_[2].y_;
            targetInit_[3].y_=-targetInit_[3].y_;
            targetInit_[4].y_=-targetInit_[4].y_;
            posCatch.y_=-posCatch.y_;
            posPass.y_=posPass.y_;
        }
    }
    else
    {
        static bool IsInOurArea=tmpball.x_<-250? true:false;              //划分半场用的是-250
        if(tmpball.x_>-230)
            IsInOurArea=false;
        if(tmpball.x_<-250)                                               //我方场地的发球代表比较靠后场的发球
            IsInOurArea=true;

        point4OurReady_(IsInOurArea);
    }
    targetAllocation();
}


void StaticPass::OppCornerReady_()
{
    DPoint vec;
    DPoint pos;
    index_ = 0;
    vector<DPoint> penalty_opps;
    int max_y = 0;
    int max_y_id = -1;


    for(int i=0; i<fuseOpps_.size(); i++)
    {
        ROS_INFO("obs pos: (%f, %f)", fuseOpps_[i].x_, fuseOpps_[i].y_);
        if(fabs(fuseOpps_[i].y_) > max_y)
        {
            max_y = fabs(fuseOpps_[i].y_);
            max_y_id = i;
        }
    }

    for(int i=0; i<fuseOpps_.size(); i++)
    {
        // if(i == max_y_id)
        //    continue;
        if(fuseOpps_[i].x_ < -0.32 * FIELD_LENGTH)
            penalty_opps.push_back(fuseOpps_[i]);
    }

    for(int i=0; i<penalty_opps.size(); i++)
    {
        for(int j=i+1;j<penalty_opps.size();j++)
        {
            if(penalty_opps[j].distance(ballPos_) < penalty_opps[i].distance(ballPos_))
            {
                DPoint tmp = penalty_opps[i];
                penalty_opps[i] = penalty_opps[j];
                penalty_opps[j] = tmp;
            }
        }
    }

    for(int i=0; i<penalty_opps.size();i++)
    {
        ROS_INFO("penalty opp pos: (%f, %f)", penalty_opps[i].x_, penalty_opps[i].y_);
    }

    if(penalty_opps.size() > 0)
    {
        vec = field_.ourGoal_[GOAL_MIDDLE] - penalty_opps[0];
        pos = 100.0 / vec.length() * vec + penalty_opps[0];
    }
    else
    {
        // 固定防守点
        vec = ballPos_ - field_.ourGoal_[GOAL_MIDDLE];
        pos = 350.0 / vec.length() * vec + field_.ourGoal_[GOAL_MIDDLE];
        if(ballPos_.x_ > -700.0)
            pos = pos.rotate(ballPos_, Angle(5*Ang2Rad));
        if(pos.distance(ballPos_) < OppDefaultDis)
        {
            pos = (vec.length() - OppDefaultDis) / vec.length() * vec + field_.ourGoal_[GOAL_MIDDLE];
            pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));
        }
    }
    targetInit_[1] = pos;
    index_ = 1;

    if(penalty_opps.size() > 1)
    {
        // defend opponent near our penalty area
        DPoint danger_opp_pos = penalty_opps.back();
        ROS_INFO("danger opp pos: (%f, %f)", danger_opp_pos.x_, danger_opp_pos.y_);
        vec = field_.ourGoal_[GOAL_MIDDLE] - danger_opp_pos;
        pos = 130.0 / vec.length() * vec + danger_opp_pos;
        // 未出界的站位点的处理（是否在大禁区内）
        pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));        // ref_pt here has no meaning
    }
    else    // no opponents near the penalty area
    {
        pos = DPoint(-1.0/3.0*FIELD_LENGTH, 0.0);
    }
    targetInit_[2] = pos;
    index_ = 2;

    targetReorder(fuseOpps_);
    if(fuseOpps_.size() > 0)
    {
        pos = fuseOpps_[0].pointofline(ballPos_, -150);
        // pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));
        if(field_.isOurPenalty(pos))
        {
            if(numInPenaltyArea(OUR_PENALTY_SIDE) == 0)     // 机器人可以在自己的大禁区内而且与球的距离不受限制。利用这个规则来调整站位点。
                OptimizedinOurPenalty(pos);
            else
            {
                int nNoLoop=100;
                while(field_.isOurPenalty(pos))
                {
                    pos = pos.rotate(ballPos_, Angle(3*Ang2Rad)); // RotAroundBallOpt(pt_in, tmpAng);
                    nNoLoop--;
                    if(nNoLoop<0)
                        break;
                }
            }
        }
        targetInit_[3] = pos;
        index_ = 3;
        pos = DPoint(-400.0, 0.0);
        targetInit_[4] = pos;
        index_ = 4;
    }
    else
    {
        pos = DPoint(-400.0, -200.0);
        targetInit_[3] = pos;
        index_ = 3;
        pos = DPoint(-400.0, 200.0);
        targetInit_[4] = pos;
        index_ = 4;
    }

//    for(int i=1;i<OUR_TEAM;i++)
//        targetInit_[i] = collisionAvoid_(targetInit_[i]);

    targetAllocation();

    for(int i = 1; i <= 4; i++)
        ROS_INFO("static_pos: %d:(%0.f, %0.f)",i, targetInit_[i].x_, targetInit_[i].y_);
}

void StaticPass::OppGoalReady_()
{
    DPoint vec;
    DPoint pos;
    index_ = 0;
    vector<DPoint> ourside_opps;

    for(int i=0; i<fuseOpps_.size(); i++)
    {
        if(field_.isOurField(fuseOpps_[i]))
            ourside_opps.push_back(fuseOpps_[i]);
    }

    for(int i=0; i<ourside_opps.size(); i++)
    {
        for(int j=i+1;j<ourside_opps.size();j++)
        {
            if(ourside_opps[j].x_ > ourside_opps[i].x_) //close to the midline is preferred
            {
                DPoint tmp = ourside_opps[i];
                ourside_opps[i] = ourside_opps[j];
                ourside_opps[j] = tmp;
            }
        }
    }

    for(int i=0; i<ourside_opps.size();i++)
    {
        ROS_INFO("penalty opp pos: (%f, %f)", ourside_opps[i].x_, ourside_opps[i].y_);
    }


    if(ourside_opps.size() == 0)
    {
        pos = DPoint(-1.0/3.0*FIELD_LENGTH, 0.0);
        targetInit_[1] = pos;
        index_ = 1;
        pos = DPoint(-1.0/5.0*FIELD_LENGTH, 0.0);
        targetInit_[2] = pos;
        index_ = 2;
    }
    else if(ourside_opps.size() == 1)
    {
        vec = ballPos_ - ourside_opps[0];
        pos = 100.0 / vec.length() * vec + ourside_opps[0];
        targetInit_[1] = pos;
        index_ = 1;
        pos = DPoint(-1.0/3.0*FIELD_LENGTH, 0.0);
        targetInit_[2] = pos;
        index_ = 2;
    }
    else if(ourside_opps.size() >= 2)
    {
        vec = ballPos_ - ourside_opps[0];
        pos = 100.0 / vec.length() * vec + ourside_opps[0];
        targetInit_[1] = pos;
        index_ = 1;
        vec = ballPos_ - ourside_opps[1];
        pos = 100.0 / vec.length() * vec + ourside_opps[1];
        targetInit_[2] = pos;
        index_ = 2;
    }

    targetInit_[3] = DPoint(1.0/5.0*FIELD_LENGTH, ballPos_.y_);
    index_ = 3;
    targetInit_[4] = DPoint(1.0/5.0*FIELD_LENGTH, -ballPos_.y_);
    index_ = 4;

//    for(int i=1;i<OUR_TEAM;i++)
//        targetInit_[i] = collisionAvoid_(targetInit_[i]);

    targetAllocation();

//    for(int i = 1; i <= 4; i++)
//        ROS_INFO("static_pos: %d:(%0.f, %0.f)",i, targetInit_[i].x_, targetInit_[i].y_);
}

void StaticPass::OppDefaultReady_()               //对方发球默认站位
{
#if 0
    /** 不盯人求得的最优站位 **/
    bool isOppNearBall = 0;
    int  numOppNearBall= 0;
    index_ = 0;

    // 固定防守点
    DPoint vec = ballPos_ - field_.ourGoal_[GOAL_MIDDLE];
    DPoint pos = 350.0 / vec.length() * vec + field_.ourGoal_[GOAL_MIDDLE];
    if(ballPos_.x_ > -700.0*LENGTH_RATIO)
        pos = pos.rotate(ballPos_, Angle(5.0*Ang2Rad));
    if(pos.distance(ballPos_) < OppDefaultDis)
    {
        pos = (vec.length() - OppDefaultDis) / vec.length() * vec + field_.ourGoal_[GOAL_MIDDLE];
        pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));
    }
    //ROS_INFO("defense:%f %f", pos.x_, pos.y_);
    targetInit_[1] = pos;
    index_ = 1;

    // 固定在对方的点
    vec = ballPos_ - field_.oppGoal_[GOAL_MIDDLE];
    pos = 400.0/vec.length() * vec + field_.oppGoal_[GOAL_MIDDLE];
    if(pos.distance(ballPos_) < OppDefaultDis)
    {
        if( ballPos_.x_<350.0*LENGTH_RATIO)
            pos = (vec.length() - OppDefaultDis) / vec.length() * vec + field_.oppGoal_[GOAL_MIDDLE];
        else
        {
            pos = ballPos_+ DPoint(OppDefaultDis,0.0);
            int nloop=100;
            while(field_.isOppPenalty(pos) || field_.isOutBorder(RIGHTBORDER, pos))
            {
                if(ballPos_.y_>0)
                    pos=pos.rotate(ballPos_,Angle(-10.0*Ang2Rad));
                else
                    pos=pos.rotate(ballPos_,Angle(10.0*Ang2Rad));
                nloop--;
                if(nloop<0)
                    break;
            }
        }
    }
    //ROS_INFO("opp :[%f %f]",pos.x_, pos.y_);
    targetInit_[2] = pos;
    index_ = 2;

    if(ballPos_.x_ > -100.0*LENGTH_RATIO && fabs(ballPos_.y_) < 100*WIDTH_RATIO)
    {
        DPoint tmppt = ballPos_ + DPoint(0.0,OppDefaultDis);
        pos = tmppt.rotate(ballPos_, Angle(45.0*Ang2Rad));
        targetInit_[3] = pos;
        index_ = 3;

        tmppt = ballPos_ - DPoint(0.0,OppDefaultDis);
        pos = tmppt.rotate(ballPos_, -Angle(60.0*Ang2Rad));
        targetInit_[4] = pos;
        index_ = 4;
    }
    else
    {
        vec = field_.ourGoal_[GOAL_MIDDLE] - ballPos_;
        pos = OppDefaultDis / vec.length() * vec + ballPos_;
        pos = RotAroundBallOpt(pos, 30*Ang2Rad);
        pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));        // ref_pt here has no meaning
        targetInit_[3] = pos;
        index_ = 3;

        if(ballPos_.x_ > 500.0*LENGTH_RATIO)
        {
            vec = DPoint(0.0,0.0) - ballPos_;
            pos =  0.5 * vec + ballPos_;
            pos = RotAroundBallOpt(pos, 30*Ang2Rad);
            targetInit_[4] = pos;
            index_ = 4;
        }
        else
        {
            vec = DPoint(0.0,0.0) - ballPos_;
            pos = OppDefaultDis / vec.length() * vec + ballPos_;
            pos = RotAroundBallOpt(pos, 30*Ang2Rad);
            pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));        // ref_pt here has no meaning
            targetInit_[4] = pos;
            index_ = 4;
        }
    }

    for(int i=0; i<fuseOpps_.size();i++)
    {
        oppPos_ = fuseOpps_[i];
        DPoint vec = oppPos_ - ballPos_;
        double len = vec.length();
        if(len < RADIUS)
        {
            numOppNearBall++;
            isOppNearBall = 1;
        }
    }

    if(!isOppNearBall)
    {
        DPoint tmp(0.0,0.0);
        tmp = DPoint(ballPos_.x_ - OppDefaultDis, ballPos_.y_);            // 在球后面等着
        if(field_.isOurPenalty(tmp) || field_.isOutBorder(LEFTBORDER, tmp))
        {
            if(!OptimizedinOurPenalty(tmp))
                tmp = DPoint(ballPos_.x_ + OppDefaultDis, ballPos_.y_);     // 在球前面等着
        }
        targetInit_[4] = tmp;
        index_ = 4;
    }

    for(int i=1;i<OUR_TEAM;i++)
        targetInit_[i] = collisionAvoid_(targetInit_[i]);

    targetAllocation();
#endif
#if 0
    /** 盯人防守站位，但是站位点抖动剧烈，需要改进 **/
    bool isOppNearBall = 0;
    int  numOppNearBall=0;
    index_ = 0;

    // 固定防守点
    DPoint vec = ballPos_ - field_.ourGoal_[GOAL_MIDDLE];
    DPoint pos = 350.0 / vec.length() * vec + field_.ourGoal_[GOAL_MIDDLE];
    if(ballPos_.x_ > -700.0)
        pos = pos.rotate(ballPos_, Angle(5*Ang2Rad));
    if(pos.distance(ballPos_) < OppDefaultDis)
    {
        pos = (vec.length() - OppDefaultDis) / vec.length() * vec + field_.ourGoal_[GOAL_MIDDLE];
        pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));
    }
    targetInit_[1] = pos;
    index_ = 1;

    // 固定在对方的点
    vec = ballPos_ - field_.oppGoal_[GOAL_MIDDLE];
    pos = 400.0/vec.length() * vec + field_.oppGoal_[GOAL_MIDDLE];
    if(pos.distance(ballPos_) < OppDefaultDis)
    {
        if( ballPos_.x_<350.0*LENGTH_RATIO)
            pos = (vec.length() - OppDefaultDis) / vec.length() * vec + field_.oppGoal_[GOAL_MIDDLE];
        else
        {
            pos = ballPos_+ DPoint(OppDefaultDis,0.0);
            int nloop=100;
            while(field_.isOppPenalty(pos) || field_.isOutBorder(RIGHTBORDER, pos))
            {
                if(ballPos_.y_>0)
                    pos=pos.rotate(ballPos_,Angle(-10.0*Ang2Rad));
                else
                    pos=pos.rotate(ballPos_,Angle(10.0*Ang2Rad));
                nloop--;
                if(nloop<0)
                    break;
            }
        }
    }
    targetInit_[2] = pos;
    index_ = 2;

    targetReorder(fuseOpps_);
    if(fuseOpps_.size() > 0)
    {
        for(int i=0; i<fuseOpps_.size();i++)
        {
            oppPos_ = fuseOpps_[i];
            DPoint vec = oppPos_ - ballPos_;
            double len = vec.length();
            if(len < RADIUS)
            {
                isOppNearBall = 1;
                numOppNearBall++;
            }
        }

        if(numOppNearBall <= 3 && numOppNearBall >=2)
        {
            for(int i=0; i<fuseOpps_.size();i++)
            {
                oppPos_ = fuseOpps_[i];
                DPoint vec = oppPos_ - ballPos_;
                double len = vec.length();
                if(len > RADIUS)
                {
                    pos = (len-ME_OPP_DIS)/len * vec + ballPos_;
                    if(pos.y_ > 0)  pos.y_ -= 80 * WIDTH_RATIO;               // 侧开一点
                    else            pos.y_ += 80 * WIDTH_RATIO;
                }
                else
                {
                    // 根据d=|AR|^2 + 2*r^2*(1-cos(theta))准则算出的最优站位点
                    DPoint direc = oppPos_.rotate(ballPos_, Angle(M_PI/2.0));
                    pos = calOptPos(ballPos_, oppPos_, direc, k_);
                }
                pos = defendTargetAdjust(pos, oppPos_);
                targetInit_[++index_]=pos;
                if(index_>=4)
                    break;
            }
        }
        else
        {
            DPoint vec = field_.ourGoal_[GOAL_MIDDLE] - ballPos_;
            DPoint pos = OppDefaultDis / vec.length() * vec + ballPos_;
            pos = RotAroundBallOpt(pos, 10*Ang2Rad);
            pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));        // ref_pt here has no meaning
            targetInit_[3] = pos;
            index_ = 3;

            pos = RotAroundBallOpt(pos, 30.0*Ang2Rad);
            pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));        // ref_pt here has no meaning
            targetInit_[4] = pos;
            index_ = 4;

        }

        if(!isOppNearBall)
        {
            DPoint tmp(0,0);
            tmp = DPoint(ballPos_.x_ - OppDefaultDis, ballPos_.y_);            // 在球后面等着
            if(field_.isOurPenalty(tmp) || field_.isOutBorder(LEFTBORDER, tmp))
            {
                if(!OptimizedinOurPenalty(tmp))
                    tmp = DPoint(ballPos_.x_ + OppDefaultDis, ballPos_.y_);     // 在球前面等着
            }
            targetInit_[4] = tmp;
            index_ = 4;
        }
    }
    else
    {
        DPoint tmp(0.0,0.0);
        tmp = DPoint(ballPos_.x_ - OppDefaultDis, ballPos_.y_);            // 在球后面等着
        if(field_.isOurPenalty(tmp) || field_.isOutBorder(LEFTBORDER, tmp))
        {
            if(!OptimizedinOurPenalty(tmp))
                tmp = DPoint(ballPos_.x_ + OppDefaultDis, ballPos_.y_);     // 在球前面等着
        }
        targetInit_[3] = tmp;
        index_ = 3;

        tmp = tmp.rotate(ballPos_, Angle(90.0*Ang2Rad));
        tmp = defendTargetAdjust(tmp, ballPos_ - DPoint(100.0,0.0));
        targetInit_[4] = tmp;
        index_ = 4;
    }
    targetAllocation();
#endif
#if 1
    DPoint vec;
    DPoint pos;
    bool isOppNearBall = 0;
    int  numOppNearBall=0;
    index_ = 0;
    vector<DPoint> penalty_opps;

    for(int i=0; i<fuseOpps_.size(); i++)
    {
        if(field_.isOurPenalty_expand(fuseOpps_[i], 150.0))
            penalty_opps.push_back(fuseOpps_[i]);
    }

    for(int i=0; i<penalty_opps.size(); i++)
    {
        for(int j=i+1;j<penalty_opps.size();j++)
        {
            if(penalty_opps[j].distance(ballPos_) < penalty_opps[i].distance(ballPos_))
            {
                DPoint tmp = penalty_opps[i];
                penalty_opps[i] = penalty_opps[j];
                penalty_opps[j] = tmp;
            }
        }
    }

    if(penalty_opps.size() > 0)
    {
        vec = field_.ourGoal_[GOAL_MIDDLE] - penalty_opps[0];
        pos = 100.0 / vec.length() * vec + penalty_opps[0];
    }
    else
    {
        // 固定防守点
        vec = ballPos_ - field_.ourGoal_[GOAL_MIDDLE];
        pos = 350.0 / vec.length() * vec + field_.ourGoal_[GOAL_MIDDLE];
        if(ballPos_.x_ > -700.0)
            pos = pos.rotate(ballPos_, Angle(5*Ang2Rad));
        if(pos.distance(ballPos_) < OppDefaultDis)
        {
            pos = (vec.length() - OppDefaultDis) / vec.length() * vec + field_.ourGoal_[GOAL_MIDDLE];
            pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));
        }
    }
    targetInit_[1] = pos;
    index_ = 1;

    if(penalty_opps.size() > 1)
    {
        // defend opponent near our penalty area
        DPoint danger_opp_pos = penalty_opps.back();
        ROS_INFO("danger opp pos: (%f, %f)", danger_opp_pos.x_, danger_opp_pos.y_);
        vec = field_.ourGoal_[GOAL_MIDDLE] - danger_opp_pos;
        pos = 130.0 / vec.length() * vec + danger_opp_pos;
        // 未出界的站位点的处理（是否在大禁区内）
        pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));        // ref_pt here has no meaning
    }
    else    // no opponents near the penalty area
    {
        pos = DPoint(-1.0/3.0*FIELD_LENGTH, 0.0);
    }
    targetInit_[2] = pos;
    index_ = 2;

    targetReorder(fuseOpps_);
    if(fuseOpps_.size() > 0)
    {
        for(int i=0; i<fuseOpps_.size();i++)
        {
            oppPos_ = fuseOpps_[i];
            DPoint vec = oppPos_ - ballPos_;
            double len = vec.length();
            if(len < RADIUS)
            {
                isOppNearBall = 1;
                numOppNearBall++;
            }
        }

        if(numOppNearBall <= 3 && numOppNearBall >=2)
        {
            for(int i=0; i<fuseOpps_.size();i++)
            {
                oppPos_ = fuseOpps_[i];
                DPoint vec = oppPos_ - ballPos_;
                double len = vec.length();
                if(len > RADIUS)
                {
                    pos = (len-ME_OPP_DIS)/len * vec + ballPos_;
                    if(pos.y_ > 0)  pos.y_ -= 80 * WIDTH_RATIO;               // 侧开一点
                    else            pos.y_ += 80 * WIDTH_RATIO;
                }
                else
                {
                    // 根据d=|AR|^2 + 2*r^2*(1-cos(theta))准则算出的最优站位点
                    DPoint direc = oppPos_.rotate(ballPos_, Angle(M_PI/2.0));
                    pos = calOptPos(ballPos_, oppPos_, direc, k_);
                }
                pos = defendTargetAdjust(pos, oppPos_);
                targetInit_[++index_]=pos;
                if(index_>=4)
                    break;
            }
        }
        else
        {
            DPoint vec = field_.ourGoal_[GOAL_MIDDLE] - ballPos_;
            DPoint pos = OppDefaultDis / vec.length() * vec + ballPos_;
            pos =  RotAroundBallOpt(pos, 10*Ang2Rad);
            pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));        // ref_pt here has no meaning
            targetInit_[3] = pos;
            index_ = 3;

            pos = RotAroundBallOpt(pos, 30.0*Ang2Rad);
            pos = defendTargetAdjust(pos, ballPos_ - DPoint(100.0,0.0));        // ref_pt here has no meaning
            targetInit_[4] = pos;
            index_ = 4;

        }

        if(!isOppNearBall)
        {
            DPoint tmp(0,0);
            tmp = DPoint(ballPos_.x_ - OppDefaultDis, ballPos_.y_);            // 在球后面等着
            if(field_.isOurPenalty(tmp) || field_.isOutBorder(LEFTBORDER, tmp))
            {
                if(!OptimizedinOurPenalty(tmp))
                    tmp = DPoint(ballPos_.x_ + OppDefaultDis, ballPos_.y_);     // 在球前面等着
            }
            targetInit_[4] = tmp;
            index_ = 4;
        }
    }
    else
    {
        DPoint tmp(0.0,0.0);
        tmp = DPoint(ballPos_.x_ - OppDefaultDis, ballPos_.y_);            // 在球后面等着
        if(field_.isOurPenalty(tmp) || field_.isOutBorder(LEFTBORDER, tmp))
        {
            if(!OptimizedinOurPenalty(tmp))
                tmp = DPoint(ballPos_.x_ + OppDefaultDis, ballPos_.y_);     // 在球前面等着
        }
        targetInit_[3] = tmp;
        index_ = 3;

        tmp = tmp.rotate(ballPos_, Angle(90.0*Ang2Rad));
        tmp = defendTargetAdjust(tmp, ballPos_ - DPoint(100.0,0.0));
        targetInit_[4] = tmp;
        index_ = 4;
    }

//    for(int i=1;i<OUR_TEAM;i++)
//        targetInit_[i] = collisionAvoid_(targetInit_[i]);

    targetAllocation();

    for(int i = 1; i <= 4; i++)
        ROS_INFO("static_pos: %d:(%0.f, %0.f)",i, targetInit_[i].x_, targetInit_[i].y_);
#endif
}

void StaticPass::OurPenaltyReady_()                   //penalty发球时的站位，不管足球的位置，跑到指定位置，不用分配
{   // TODO: avoid collision
#ifndef END_GAME_PENALTY
    DPoint tmp = field_.oppPenaltyPt_ + DPoint(-320*LENGTH_RATIO, 0.0);
    targetInit_[1]= field_.oppPenaltyPt_ + DPoint(-100*LENGTH_RATIO, 0.0);                  //主罚点球的机器人所在的位置
    targetInit_[2]= tmp.rotate(field_.oppPenaltyPt_, 70*Ang2Rad);
    targetInit_[3]= tmp.rotate(field_.oppPenaltyPt_, -70*Ang2Rad);;
    targetInit_[4]= DPoint(100, 0);
    targetAllocation();
#else
    targetInit_[1] = DPoint(50,0);                  //主罚点球的机器人所在的位置
    targetInit_[2] = DPoint(-300, FIELD_WIDTH/4);
    targetInit_[3] = DPoint(-300, 0);
    targetInit_[4] = DPoint(-300, -FIELD_WIDTH/4);
    targetAllocation();
#endif
}

void StaticPass::OppPenaltyReady_()               //对方penalty发球
{   // TODO: avoid collision
#ifndef END_GAME_PENALTY
    DPoint tmp = field_.ourPenaltyPt_ + DPoint(320*LENGTH_RATIO, 0.0);
    DPoint tmp2 = field_.ourPenaltyPt_ + DPoint(0.0, -440*WIDTH_RATIO);
    targetInit_[1]= tmp2.rotate(field_.ourPenaltyPt_, -26*Ang2Rad);          // 作为防守
    targetInit_[2]= tmp.rotate(field_.ourPenaltyPt_, 60*Ang2Rad);
    targetInit_[3]= tmp.rotate(field_.ourPenaltyPt_, -60*Ang2Rad);
    targetInit_[4]= DPoint(-100, 0);
    targetAllocation();
#else
    targetInit_[1] = DPoint(300, FIELD_WIDTH/6);
    targetInit_[2] = DPoint(300, FIELD_WIDTH/3);
    targetInit_[3] = DPoint(300, -FIELD_WIDTH/3);
    targetInit_[4] = DPoint(300, -FIELD_WIDTH/6);
    targetAllocation();
#endif
}

void StaticPass::OurkickoffReady_()                   //我方kickoff发球站位
{

    DPoint tmpball=ballPos_;
    DPoint posCatch;
    DPoint posPass;

    //传球队员-球-对方球门
    DPoint tmp=tmpball.pointofline(field_.oppGoal_[GOAL_MIDDLE],TAC_DIST_PASS);
    tmp=tmp.rotate(tmpball,Angle(90*Ang2Rad));
    targetInit_[1]=tmp;
    posPass=tmp;

    //接球进攻队员-球-对方球门
    tmp=tmpball.pointofline(field_.oppGoal_[GOAL_MIDDLE],TAC_DIST_CATCH);
    tmp=tmp.rotate(tmpball,Angle(-90*Ang2Rad));
    targetInit_[2]=tmp;
    posCatch=tmp;

    //其他站位点，避免与球太近
   // if(tmpball.distance(backFieldPoint_)>200)
   //     targetInit_[3]=backFieldPoint_;
   // else
   //     targetInit_[3]=DPoint(-400*LENGTH_RATIO,500*WIDTH_RATIO);

   // if(tmp.distance(DPoint(-600,-300))>200)
   //     targetInit_[4]=DPoint(-400*LENGTH_RATIO,-500*WIDTH_RATIO);
  //  else
  //  {
  //      if(tmpball.distance(backFieldPoint_)>200)
  //          targetInit_[4]=backFieldPoint_+DPoint(100,0);          //避免和3冲突，前移一米
  //      else
  //          targetInit_[4]=backFieldPoint_;
  //  }
    targetInit_[3] = DPoint(-400,0);
    targetInit_[4] = DPoint(-800,0);
    targetAllocation();
}

void StaticPass::OppkickoffReady_()               //对方kickoff发球站位
{

    DPoint tmpball=ballPos_;
    DPoint tmp;

    //根据球的位置站位，优先保证
    tmp=tmpball.pointofline(field_.ourGoal_[GOAL_MIDDLE],TAC_DIST_ANTI);         //在原始程序中是AROUNDBALL，就需要通过tmp来计算，然后赋值
    tmp=tmp.rotate(tmpball,Angle(-Ang2Rad));
    tmp=targetAdjust(tmp);
    targetInit_[1]=tmp;

    tmp=tmpball.pointofline(field_.ourGoal_[GOAL_MIDDLE],TAC_DIST_ANTI);         //在原始程序中是AROUNDBALL，就需要通过tmp来计算，然后赋值
    tmp=tmp.rotate(tmpball,Angle(70*Ang2Rad));
    tmp=targetAdjust(tmp);
    targetInit_[2]=tmp;

    tmp=tmpball.pointofline(field_.ourGoal_[GOAL_MIDDLE],TAC_DIST_ANTI);         //在原始程序中是AROUNDBALL，就需要通过tmp来计算，然后赋值
    tmp=tmp.rotate(tmpball,Angle(-70*Ang2Rad));
    tmp=targetAdjust(tmp);
    targetInit_[4]=tmp;

    //根据点的静态站位，避免冲突
    targetInit_[3]=backFieldPoint_;
    if(targetInit_[3].distance(tmpball)<TAC_DIST_ANTI)
        targetInit_[3].y_=tmpball.y_-TAC_DIST_ANTI;                //防止后卫点离球太近
    targetInit_[3]=collisionAvoid_(targetInit_[3]);

    targetAllocation();
}

void StaticPass::DropBallReady_()                  //dropball站位
{
    DPoint tmpball=ballPos_;                    //跟defaultReady_一样，先假设在上半场，计算后翻转
    static bool flipField=true;
    if(ballPos_.y_<-50)                         //放置中线附近的定位晃动
        flipField=true;
    if(ballPos_.y_>50)
        flipField=false;
    if(flipField)
        tmpball.y_=-tmpball.y_;
    DPoint tmp;
    if(tmpball.x_>-700*LENGTH_RATIO)                         //非角球区域发球
    {
        //球在我方半场中等距离时，在球到球门之间堵上一个
        tmp=tmpball.pointofline(field_.ourGoal_[GOAL_MIDDLE],TAC_DIST_DROPBALL);
        targetInit_[1]=tmp;
        if(tmpball.x_<-250*LENGTH_RATIO)
            targetInit_[2]=DPoint(100,0);      //等待接球
        else
        {
            tmp=tmpball.pointofline(field_.ourGoal_[GOAL_MIDDLE],TAC_DIST_DROPBALL+130);
            tmp=tmp.rotate(tmpball,Angle(50*Ang2Rad));
            targetInit_[2]=tmp;
        }

        tmp=tmpball.pointofline(field_.ourGoal_[GOAL_MIDDLE],TAC_DIST_DROPBALL+130);
        tmp=tmp.rotate(tmpball,Angle(-50*Ang2Rad));
        while(tmp.y_>550*WIDTH_RATIO)
            tmp=tmp.rotate(tmpball,Angle(5*Ang2Rad));
        targetInit_[4]=tmp;

        if(targetInit_[1].x_<-700*LENGTH_RATIO)
        {
            //堵门队员退到禁区里，应该是球门任意球，位置前移
            targetInit_[1].x_=-700*LENGTH_RATIO;
            targetInit_[2]=targetInit_[2].rotate(tmpball,Angle(50*Ang2Rad));
            targetInit_[4]=targetInit_[4].rotate(tmpball,Angle(-20*Ang2Rad));
        }
        //根据点的静态站位，避免冲突
        targetInit_[3]=backFieldPoint_;              //target[3]定位点要避免冲突
        targetInit_[3]=collisionAvoid_(targetInit_[3]);
    }
    else
    {
        tmp=tmpball.pointofline(field_.ourGoal_[GOAL_MIDDLE],TAC_DIST_DROPBALL);
        tmp=tmp.rotate(tmpball,Angle(15*Ang2Rad));
        targetInit_[1]=tmp;
        //在第一个防守队员上旋转一个角度
        tmp=tmpball.pointofline(field_.ourGoal_[GOAL_MIDDLE],TAC_DIST_DROPBALL+130);         //在原始程序中是AROUNDBALL，就需要通过tmp来计算，然后赋值
        tmp=tmp.rotate(tmpball,Angle(60*Ang2Rad));
        targetInit_[2]=tmp;

        targetInit_[3]=DPoint(backFieldPoint_.x_,300.0);
        targetInit_[3]=collisionAvoid_(targetInit_[3]);

        targetInit_[4]=backFieldPoint_;
        targetInit_[4]=collisionAvoid_(targetInit_[4]);
    }
    if(flipField)               //镜像变化
    {
        targetInit_[1].y_=-targetInit_[1].y_;
        targetInit_[2].y_=-targetInit_[2].y_;
        targetInit_[3].y_=-targetInit_[3].y_;
        targetInit_[4].y_=-targetInit_[4].y_;
    }
    targetAllocation();
}

void StaticPass::targetAllocation()               //基于当前位置和机器人编号的分配，目前场上编号最大的机器人分配为准
{
    double dis_thres = 70;      // 新旧站位点的距离阈值
    int num=-1;                                    //机器人编号
    if(world_model_->AgentID_==1)                  //我是守门员
    {
        target_=targetInit_[0];
        world_model_->RobotInfo_[world_model_->AgentID_-1].setTargetNum(0,0);
    }
    else
    {
        if(islargestNum_())                            //编号最大的占据主动
        {
            for(int i=1;i<world_model_->CurActiveNotGoalieNums_+1;i++)                //从targetInit[1]开始分配目标点
            {
                num=getNearestRobot_(targetInit_[i]);   // TODO: this allocation find out the neareast robot to the target points;
                                                        // the shortcoming of this is that robots may always switching position;
                                                        // consider another way: find out the nearest target point to robots
                                                        // MAYBE NOT!
                if(world_model_->CoachInfo_.MatchMode ==OUR_KICKOFF  || world_model_->CoachInfo_.MatchMode ==OUR_THROWIN    ||
                   world_model_->CoachInfo_.MatchMode ==OUR_FREEKICK || world_model_->CoachInfo_.MatchMode ==OUR_GOALKICK   ||
                   world_model_->CoachInfo_.MatchMode ==OUR_CORNERKICK)//我方发球时，才会赋值
                {
                    if(i==1)
                        m_nPassNumber_=num+1;
                    else if(i==2)
                        m_nCatchNumber_=num+1;
                    world_model_->RobotInfo_[world_model_->AgentID_-1].setcatchNum(m_nCatchNumber_);
                    world_model_->RobotInfo_[world_model_->AgentID_-1].setpassNum(m_nPassNumber_);
                }

                if(num==world_model_->AgentID_-1)      //如果分配到本台机器人
                {
                    target_ = stablePosition(targetInit_[i], preTarget_, dis_thres);
                    targetInit_[i] = target_;
                }

                world_model_->RobotInfo_[world_model_->AgentID_-1].setTargetNum(num,i);
            }
        }
        else
        {
            int index = world_model_->RobotInfo_[largestNum_].getTargetNum(world_model_->AgentID_-1);
            target_ = stablePosition(targetInit_[index], preTarget_, dis_thres);
            targetInit_[index] = target_;

            if(world_model_->CoachInfo_.MatchMode ==OUR_KICKOFF  || world_model_->CoachInfo_.MatchMode ==OUR_THROWIN    ||
               world_model_->CoachInfo_.MatchMode ==OUR_FREEKICK || world_model_->CoachInfo_.MatchMode ==OUR_GOALKICK   ||
               world_model_->CoachInfo_.MatchMode ==OUR_CORNERKICK)//我方发球时，才会赋值
            {
                m_nCatchNumber_=world_model_->RobotInfo_[largestNum_].getcatchNum();
                m_nPassNumber_=world_model_->RobotInfo_[largestNum_].getpassNum();
            }
        }
        //ROS_INFO("target:[%f %f], pre:[%f %f]", target_.x_, target_.y_, preTarget_.x_, preTarget_.y_);
        preTarget_ = target_;
    }
//    ROS_INFO("catch num: %d and pass_num:  %d  %d ",m_nCatchNumber_,m_nPassNumber_,world_model_->AgentID_);
    isPosition_=true;
}

void StaticPass::targetInitialize()
{
    // 给所有站位点赋值，防止后面也没有赋值导致出错
    targetInit_[0]=DPoint(-1050*LENGTH_RATIO,0.0);                  //静态站位时，守门员基本位置恒定
    targetInit_[1]=DPoint(-500*LENGTH_RATIO,200*WIDTH_RATIO);
    targetInit_[2]=DPoint(-500*LENGTH_RATIO,-200*WIDTH_RATIO);
    targetInit_[3]=DPoint(-200*LENGTH_RATIO,100*WIDTH_RATIO);
    targetInit_[4]=DPoint(-200*LENGTH_RATIO,-100*WIDTH_RATIO);
}

DPoint StaticPass::stablePosition(DPoint current_pos, DPoint pre_pos, double thres)
{
    current_pos.x_ = round(current_pos.x_/10.0)*10;     //对厘米级的误差忽略
    current_pos.y_ = round(current_pos.y_/10.0)*10;
    if(world_model_->CoachInfo_.MatchMode == OPP_FREEKICK || world_model_->CoachInfo_.MatchMode == OPP_THROWIN ||
       world_model_->CoachInfo_.MatchMode == OPP_GOALKICK || world_model_->CoachInfo_.MatchMode == OPP_CORNERKICK)
    {   // 对方发球时防止站位抖动里厉害
        double dis = current_pos.distance(pre_pos);
        if(dis < thres)
        {
            //ROS_INFO("less than dis_thres [%f %f]",pre_pos.x_, pre_pos.y_);
            current_pos = pre_pos;
        }
    }
    return current_pos;
}

void StaticPass::point4OurReady_(bool IsInOurArea)                   //我方发球时，站位点的计算
{
    vector<DPoint> candidate;         //候选点
    vector<double> energies;          //势能值

    if(IsInOurArea)                 //如果在我方场地发球
    {
        for(int i=400;i<800;i=i+50)         //候选点的横坐标，50为间隔
            for(int j=-540*WIDTH_RATIO;j<540*WIDTH_RATIO;j=j+60*WIDTH_RATIO)    //候选点的纵坐标，60|40为间隔
            {
                DPoint temp=DPoint(i,j);
                LineSegment B2candidate(ballPos_,temp);             //通向该候选点的传球路径
                LineSegment candidate2Goal(temp,field_.oppGoal_[GOAL_MIDDLE]);
                double line_length = B2candidate.distance();
                //倾向于长距离传球
                double energy = (line_length > 1000||line_length < 600) ? 0.1: (0.4-0.0015*fabs(line_length - 800)); //0.1~0.4
                //判断传球路径上有没有障碍物
                int near_obs = 0;
                for(int n=0;n<world_model_->Opponents_.size();n++)
                    if(candidate2Goal.distance(world_model_->Opponents_[n],true)<100||B2candidate.distance(world_model_->Opponents_[n],true)<150||temp.distance(world_model_->Opponents_[n])<100)
                        near_obs++;

                if(near_obs>3)  near_obs = 3;
                energy += 0.15*(3-near_obs)-0.05; //0.05~0.4
                //可射门角度
                double P2goal_left=field_.oppGoal_[GOAL_UPPER].angle(temp).radian_;
                double P2goal_right=field_.oppGoal_[GOAL_LOWER].angle(temp).radian_;
                double kick_ang = fabs(angularnorm(P2goal_left-P2goal_right));
                energy += 0.1*kick_ang/M_PI_2+0.1; //0.1~0.2

                candidate.push_back(temp);            //压入候选点
                energies.push_back(energy);           //压入势能值
            }

        DPoint catch_point=DPoint(0,0);
        double max_energy=0.0;
        //选择势能值最大的点作为接球机器人的点
        for(int i=0;i<candidate.size();i++)
        {
            if(energies[i]>max_energy)
            {
                max_energy=energies[i];
                catch_point=candidate[i];
            }
        }
        targetInit_[2]=catch_point;          //接球点

        DPoint next_point=DPoint(0,0);
        double next_energy=0.0;
        //得到势能值最大的点后在距离它比较选距离处再选次优点作为另一个机器人的静态站位点
        for(int i=0;i<candidate.size();i++)
        {
            if(energies[i]>next_energy && energies[i]!=max_energy && candidate[i].distance(catch_point)>500)
            {
                next_energy=energies[i];
                next_point=candidate[i];
            }
        }

        if(next_point==DPoint(0,0))  //没有找到第二个点
            next_point=DPoint(next_point.x_,-next_point.y_);   //取上一个点的对称点

        targetInit_[4]=next_point;           //干扰接球点

        //计算传球机器人站位点, 距离球1m，朝向对方球门
        targetInit_[1]=ballPos_.pointofline(field_.oppGoal_[GOAL_MIDDLE],-TAC_DIST_PASS);

        //计算第三台机器人的站位点
        targetInit_[3]=ballPos_.pointofline(targetInit_[2],TAC_DIST_CATCH);
        if(ballPos_.y_>0)
            targetInit_[3]=targetInit_[3].rotate(ballPos_,Angle(-30*Ang2Rad));           //绕球顺时针旋转30度，避免挡住传球路径
        else
            targetInit_[3]=targetInit_[3].rotate(ballPos_,Angle(30*Ang2Rad));            //绕球逆时针旋转30度，避免挡住传球路径
    }
    else
    {
        for(int i=300;i<700;i=i+50)         //候选点的横坐标，50为间隔
            for(int j=-540*WIDTH_RATIO;j<540*WIDTH_RATIO;j=j+60*WIDTH_RATIO)    //候选点的纵坐标，60|40为间隔
            {
                DPoint temp=DPoint(i,j);
                LineSegment B2candidate(ballPos_,temp);             //通向该候选点的传球路径
                LineSegment candidate2Goal(temp,field_.oppGoal_[GOAL_MIDDLE]);
                double line_length = B2candidate.distance();
                //倾向于长距离传球
                double energy = (line_length > 300||line_length < 200) ? 0.1: (0.4-0.006*fabs(line_length - 250)); //0.1~0.4
                //判断传球路径上有没有障碍物
                int near_obs = 0;
                for(int n=0;n<world_model_->Opponents_.size();n++)
                    if(candidate2Goal.distance(world_model_->Opponents_[n],true)<100||B2candidate.distance(world_model_->Opponents_[n],true)<150||temp.distance(world_model_->Opponents_[n])<100)
                        near_obs++;

                if(near_obs>3)  near_obs = 3;
                energy += 0.15*(3-near_obs)-0.05; //0.05~0.4
                //可射门角度
                double P2goal_left=field_.oppGoal_[GOAL_UPPER].angle(temp).radian_;
                double P2goal_right=field_.oppGoal_[GOAL_LOWER].angle(temp).radian_;
                double kick_ang = fabs(angularnorm(P2goal_left-P2goal_right));
                energy += 0.1*kick_ang/M_PI_2+0.1; //0.1~0.2

                candidate.push_back(temp);            //压入候选点
                energies.push_back(energy);           //压入势能值
            }

        DPoint catch_point=DPoint(0,0);
        double max_energy=0.0;
        //选择势能值最大的点作为接球机器人的点 （并且该点不能距离球太近）
        for(int i=0;i<candidate.size();i++)
        {
            if(energies[i]>max_energy && candidate[i].distance(ballPos_)>TAC_DIST_CATCH)
            {
                max_energy=energies[i];
                catch_point=candidate[i];
            }
        }
        targetInit_[2]=catch_point;          //接球点

        DPoint next_point=DPoint(0,0);
        double next_energy=0.0;
        //得到势能值最大的点后在距离它比较选距离处再选次优点作为另一个机器人的静态站位点
        for(int i=0;i<candidate.size();i++)
        {
            if(energies[i]>next_energy && energies[i]!=max_energy && candidate[i].distance(catch_point)>400 && candidate[i].distance(ballPos_)>TAC_DIST_CATCH)
            {
                next_energy=energies[i];
                next_point=candidate[i];
            }
        }

        if(next_point==DPoint(0,0))  //没有找到第二个点
            next_point=DPoint(next_point.x_,-next_point.y_);   //取上一个点的对称点

        targetInit_[4]=next_point;           //干扰接球点

        //由于在对方场地，保留一台机器在己方场地
        targetInit_[3]=backFieldPoint_;

        //计算传球机器人站位点
        targetInit_[1]=ballPos_.pointofline(DPoint((catch_point.x_+next_point.x_)/2,(catch_point.y_+next_point.y_)/2),-TAC_DIST_PASS);
    }
}
