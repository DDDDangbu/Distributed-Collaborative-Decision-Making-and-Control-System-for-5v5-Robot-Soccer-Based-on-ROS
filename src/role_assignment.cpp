#include "nubot_control/role_assignment.h"

using namespace nubot;

RoleAssignment::RoleAssignment()
{
    currentRoleTime_    = lastRoleChangeTime_ = ros::Time::now();
    last_activeID=0;
    last_activeUtility=0;
}

RoleAssignment::RoleAssignment(World_Model_Info & _world_model)
{
    world_model_ = & _world_model;
    currentRoleTime_    = lastRoleChangeTime_ = ros::Time::now();
    last_activeID=0;
    last_activeUtility=0;
}

RoleAssignment::~RoleAssignment(){}

int RoleAssignment::process()
{
    /** 一号机器人直接赋予其守门员角色,如果场地上只有一个机器人（并不是因为通信中断导致）赋予守门员角色*/
    if(world_model_->AgentID_==1)
        return GOALIE;
    char last_role = world_model_->RobotInfo_[world_model_->AgentID_-1].getCurrentRole();
    /** 根据比赛状态，给机器人设置角色*/
    adjustRole();
    if(last_activeID>=2)
        last_activeUtility=RoleUtilityMatrix_[last_activeID-1][0];
    calculateRoleUtility();
    selectRole();
    last_activeID=m_plan_->getWhichRoleID(ACTIVE);
    /** 这儿是真正的角色决定的时间，重新更新角色维持的时间*/
    currentRoleTime_ = ros::Time::now();
    /** 重新更新了角色，因此需要计算新的角色维持时间,初始值位0*/
    if(world_model_->RobotInfo_[world_model_->AgentID_-1].getCurrentRole()!=last_role)
    {
        lastRoleChangeTime_=ros::Time::now();
        world_model_->RobotInfo_[world_model_->AgentID_-1].setRolePreserveTime(0);
    }
    else
    {
        ros::Duration duration = currentRoleTime_-lastRoleChangeTime_;
        world_model_->RobotInfo_[world_model_->AgentID_-1].setRolePreserveTime(duration.toSec());
    }
    return world_model_->RobotInfo_[world_model_->AgentID_-1].getCurrentRole();
}

/**
    * 主要是计算角色的效能值，特别是主攻与防守两个角色的效能置（势能函数）
    */
void
RoleAssignment::calculateRoleUtility()
{
    double bonus(0);
    /** 主攻与防守的基点，效能值计算都是围绕着这两点展开*/
    PPoint activepos,defendpose,assistpose, midfieldpose;
    /** 主攻与防守最终效能置*/
    double activeutility(1),defendutiity(1),assistutiity(1), midfieldutility(1);
    /** 场地上没有足球信息，则所有角色都保持，无须分配任何角色*/
    if(world_model_->BallInfoState_==NOTSEEBALL)
        return ;

    for(int i = 1 ; i < OUR_TEAM ;i++)
    {
        /** 从二号机器人开始计算，一号是守门员*/
        RoleUtilityMatrix_[i-1][0]=1; //主攻
        RoleUtilityMatrix_[i-1][1]=1; //防守
        RoleUtilityMatrix_[i-1][2]=1; //助攻
        RoleUtilityMatrix_[i-1][3]=1; //中场
        /** 机器人无效则直接进入下一个机器人计算*/
        if(!world_model_->RobotInfo_[i].isValid())
            continue;

        PPoint ball_real_pos;
        double length = 0;

        DPoint ball_vec_coorect(0,0);
        if(world_model_->fuseBallInfo_.isVelocityKnown())
            ball_vec_coorect = world_model_->fuseBallInfo_.getVelocity();
        DPoint pt_ball = ball_vec_coorect*ACTIVE_BALL_VEL+world_model_->fuseBallInfo_.getGlobalLocation()
                -world_model_->RobotInfo_[i].getLocation();
        PPoint pts_ball(pt_ball);
        ball_real_pos = PPoint(pts_ball.angle_-world_model_->RobotInfo_[i].getHead(),pts_ball.radius_);

        /** 写入角色保持的时间，信息要传到其他机器人，在角色分配之后需要重新计算，再次赋值*/
        ros::Duration duration = currentRoleTime_-lastRoleChangeTime_;
        world_model_->RobotInfo_[world_model_->AgentID_-1].setRolePreserveTime(duration.toSec());

        /** PART I: 计算主攻的效能值， */
        currentRoleTime_ = ros::Time::now();
        if(world_model_->RobotInfo_[i].getDribbleState())     /** 如果带上球*/
            activeutility=ACTIVE_MAX;
        else
        {
            activepos=ball_real_pos;
            length = activepos.radius_;
            /** 与足球的远近来计算效能置*/
            if(isoppBlockWay(world_model_->RobotInfo_[i].getLocation(),world_model_->fuseBallInfo_.getGlobalLocation()))
                length = length + (2*M_PI - 4)*block_dist;

            activeutility = std::max(1.0, ACTIVE_K*length +ACTIVE_B);
            activeutility = std::min(200.0, activeutility);
            /** 足球位置于机器人之间的夹角，计算效能值*/
            activeutility = activeutility+ACTIVE_THETA_K*fabs(activepos.angle_.radian_)+ACTIVE_THETA_B;
        }
        /** 如果上一帧是主攻的角色，因为一定时间内角色保持会增加其效能置,多机器人系统该值怎么确认*/
        if(world_model_->RobotInfo_[i].getCurrentRole() == ACTIVE)
        {
            /** 这地方以秒计算是否有问题，值得商榷*/
            bonus=10+world_model_->RobotInfo_[i].getRolePreserveTime();
            bonus=std::max(10.0,bonus);
            bonus=std::min(100.0, bonus);
            activeutility = activeutility+bonus;
        }
        if(isoppBlockWay(world_model_->RobotInfo_[i].getLocation(),world_model_->fuseBallInfo_.getGlobalLocation()))
            activeutility = activeutility - bonus - 10;
        //        ROS_INFO("ID%d active:%f",i+1, activeutility);
        /** 主攻的角色效能置*/
        RoleUtilityMatrix_[i-1][0] = activeutility; //

        /** PART II: 计算防守的效能值， */
        DPoint defend_pose_global = passive_pt_;
        /** 全局坐标转换为局部坐标*/
        DPoint pt=defend_pose_global-world_model_->RobotInfo_[i].getLocation();
        PPoint pts(pt);
        defendpose = PPoint(pts.angle_-world_model_->RobotInfo_[i].getHead(),pts.radius_);

        /** 与防守点的远近来计算效能置*/
        defendutiity = std::max(1.0, PASSIVE_K*defendpose.radius_+PASSIVE_B);
        defendutiity = std::min(200.0, defendutiity);
        DPoint pt2 = DPoint(world_model_->fuseBallInfo_.getRealLocation()) - DPoint(defendpose); /** 在机器人体坐标系下向量*/
        Angle angrb2defpos= PPoint(pt2).angle_ - world_model_->RobotInfo_[i].getHead();
        defendutiity = defendutiity+PASSIVE_THETA_K*fabs(angrb2defpos.radian_)+PASSIVE_THETA_B;
        if(world_model_->RobotInfo_[i].getCurrentRole() == PASSIVE)
        {
            bonus=10+world_model_->RobotInfo_[i].getRolePreserveTime();
            bonus=std::max(10.0,bonus);
            bonus=std::min(20.0, bonus);
            defendutiity = defendutiity+bonus;
        }
        RoleUtilityMatrix_[i-1][1] = defendutiity;


        /** PART III: 计算助攻的效能值， */
        DPoint assist_pose_global = assist_pt_;
        /** 全局坐标转换为局部坐标*/
        DPoint pt_assit=assist_pose_global-world_model_->RobotInfo_[i].getLocation();
        PPoint pts_assist(pt_assit);
        assistpose = PPoint(pts_assist.angle_-world_model_->RobotInfo_[i].getHead(),pts_assist.radius_);
        /** 与助攻点的远近来计算效能置*/
        assistutiity = std::max(1.0, PASSIVE_K*assistpose.radius_+PASSIVE_B);
        assistutiity = std::min(200.0, assistutiity);
        DPoint pt3 = DPoint(world_model_->fuseBallInfo_.getRealLocation()) - DPoint(assistpose); /** 在机器人体坐标系下向两*/
        Angle angrb2assistpose= PPoint(pt3).angle_-world_model_->RobotInfo_[i].getHead();
        assistutiity = assistutiity+PASSIVE_THETA_K*fabs(angrb2assistpose.radian_)+PASSIVE_THETA_B;
        if(world_model_->RobotInfo_[i].getCurrentRole() == ASSISTANT)
        {
            bonus=10+world_model_->RobotInfo_[i].getRolePreserveTime();
            bonus=std::max(10.0,bonus);
            bonus=std::min(20.0, bonus);
            assistutiity = assistutiity+bonus;
        }
        RoleUtilityMatrix_[i-1][2] = assistutiity;

        /** PART IV: 计算中场的效能值， */
        DPoint mid_pose_global = assist_pt_;
        /** 全局坐标转换为局部坐标*/
        DPoint pt_mid=mid_pose_global-world_model_->RobotInfo_[i].getLocation();
        PPoint pts_mid(pt_mid);
        midfieldpose = PPoint(pts_mid.angle_-world_model_->RobotInfo_[i].getHead(),pts_mid.radius_);
        /** 与助攻点的远近来计算效能置*/
        midfieldutility = std::max(1.0, PASSIVE_K*midfieldpose.radius_+PASSIVE_B);
        midfieldutility = std::min(200.0, midfieldutility);
        DPoint pt4 = DPoint(world_model_->fuseBallInfo_.getRealLocation()) - DPoint(midfieldpose); /** 在机器人体坐标系下向两*/
        Angle angrb2midpose= PPoint(pt4).angle_-world_model_->RobotInfo_[i].getHead();
        midfieldutility = midfieldutility+PASSIVE_THETA_K*fabs(angrb2midpose.radian_)+PASSIVE_THETA_B;
        if(world_model_->RobotInfo_[i].getCurrentRole() == MIDFIELD)
        {
            bonus=10+world_model_->RobotInfo_[i].getRolePreserveTime();
            bonus=std::max(10.0,bonus);
            bonus=std::min(20.0, bonus);
            midfieldutility = midfieldutility+bonus;
        }
        RoleUtilityMatrix_[i-1][3] = midfieldutility;
    }
}
/**
    * 主要是判断是否存在角色冲突现象，如果存在冲突调整角色为NOROLE
    */
void RoleAssignment::adjustRole()
{
    /** 假设没有看到球，不需要调整角色，仍为上一帧的角色*/
    if(world_model_->BallInfoState_ == NOTSEEBALL){
        return;
    }
    /** 遍历所有机器人，判断是否存在角色冲突现象，如果存在则分配无角色*/
    bool isSameRole[OUR_TEAM];
    memset(isSameRole,false,OUR_TEAM*sizeof(bool));
    /** 1号机器人不予考虑，始终为守门员*/
    for(int j = 1 ; j < OUR_TEAM; j++)
    {
        for(int k = j+1; k< OUR_TEAM; k++)
        {
            /** ID为j的机器人无效，则不需要继续考虑，查看下一个机器人*/
            if(!world_model_->RobotInfo_[j].isValid())
                break;
            else if(world_model_->RobotInfo_[k].isValid())
            {
                /** 分配了相同的角色给两个机器人*/
                if( world_model_->RobotInfo_[j].getCurrentRole() ==
                        world_model_->RobotInfo_[k].getCurrentRole())
                    isSameRole[k] = isSameRole[j]=true;
            }
        }
    }
    for(int k = 1; k< OUR_TEAM; k++)
    {
        if(isSameRole[k])
            world_model_->RobotInfo_[k].setCurrentRole(NOROLE);
    }
}
/**
 * 没有球，且角色冲突时根据机器人编号，随便分配角色，无球具有专门的找球动作，与角色没有太大的关系
 * */
void
RoleAssignment::fixRole()
{
    if(world_model_->DribbleState_.is_dribble_)
        world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ACTIVE);
    else
    {
        switch(world_model_->AgentID_)
        {
        case 1:
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(GOALIE);
            break;
        case 2:
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ACTIVE);
            break;
        case 3:
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(PASSIVE);
            break;
        case 4:
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(MIDFIELD);
            break;
        case 5:
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ACIDPASSIVE);
            break;
        case 6:
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ASSISTANT);
            break;
        case 7:
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(BLOCK);
            break;
        case 8:
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(GAZER);
            break;
        default:
            break;
        }
    }
}

void RoleAssignment::selectRole()
{
    int   activemaxindex(0),defencemaxindex(0),assistmaxindex(0), midmaxindex(0);
    float Activemax=-100,Defencemax=-100,Assistmax(-100), MidFieldmax(-100);

    /// 没看到足球，且没有角色（与其他机器人冲突了）
    if(world_model_->BallInfoState_==NOTSEEBALL)
    {
        /// 按照编号分配的角色，如何消除角色冲突？
        if(world_model_->RobotInfo_[world_model_->AgentID_-1].getCurrentRole()==NOROLE)
            fixRole();
        return;
    }

    /// PART1：主攻的分配（每一个决策周期都会根据当前的势能值进行分配）
    /// 带上球过后分配为主攻
    else if(world_model_->DribbleState_.is_dribble_)
    {
        world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ACTIVE);
        return;
    }
    /// 足球已经踢出，然后强制给与起分配主攻接球机器人变成主攻,连续30帧给接球机器人分配主攻角色
    else if(world_model_->pass_state_.is_passing_)
    {
        if(world_model_->pass_state_.catch_id_ >0 && world_model_->pass_state_.catch_id_ <6)
        {
            if(world_model_->pass_state_.catch_id_ == world_model_->AgentID_)
            {
                world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ACTIVE);
                return;
            }
            /// 有接球机器人为主攻了，因此主攻角色不能再分配,给其效能值弄小值
            else
                RoleUtilityMatrix_[world_model_->AgentID_-2][0] = -100000;
        }
    }
    else if(world_model_->static_ball_dis_>100 && world_model_->static_pass_time_< WAIT_SECS  && !world_model_->is_static_pass_)
    {
        if(world_model_->catch_ID_ >0 && world_model_->catch_ID_ <6)
        {
            if(world_model_->catch_ID_ == world_model_->AgentID_)
            {
                world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ACTIVE);
                return;
            }
            /// 有接球机器人为主攻了，因此主攻角色不能再分配,给其效能值弄小值
            else
                RoleUtilityMatrix_[world_model_->AgentID_-2][0] = -100000;
        }
    }
    /// 刚进入start命令，直接赋值主攻角色，否则将其主攻势能值放小，保证自己不要分配主攻导致角色错误
    else if(world_model_->is_static_pass_)
    {
        if(world_model_->pass_ID_ == world_model_->AgentID_)
        {
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ACTIVE);
            return;
        }
        else
            RoleUtilityMatrix_[world_model_->AgentID_-2][0] = -100000;
    }
    /// 从二号机器人开始分配角色，先保证主攻
    int tmp_activeindex_1=0;
    int tmp_activeindex_2=0;
    for(int i=1; i < OUR_TEAM; i++)
        if(world_model_->RobotInfo_[i].isValid())
        {
            if(i==last_activeID-1&&RoleUtilityMatrix_[i-1][0]>last_activeUtility)
                tmp_activeindex_1=i+1;
            if(RoleUtilityMatrix_[i-1][0]>Activemax)
            {
                Activemax=RoleUtilityMatrix_[i-1][0];
                tmp_activeindex_2=i+1;
            }
        }
    if(tmp_activeindex_1>=2)
    {
        if(tmp_activeindex_1==tmp_activeindex_2)
            activemaxindex=tmp_activeindex_1;
        else if(RoleUtilityMatrix_[tmp_activeindex_2-2][0]>2*RoleUtilityMatrix_[tmp_activeindex_1-2][0])
            activemaxindex=tmp_activeindex_2;
        else
            activemaxindex=tmp_activeindex_1;
    }
    else
        activemaxindex=tmp_activeindex_2;

    if(activemaxindex==world_model_->AgentID_)
    {
        world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ACTIVE);
        return;
    }
    /// 经历主攻分配后，若未return，则该机器人没有得到主攻角色，则比较当前主攻角色和上次主攻角色ID是否相同
    int activeID=m_plan_->getWhichRoleID(ACTIVE);
    /// 如果主攻未切换且本机器人有角色，则不分配新的角色
    if(activeID==last_activeID&&world_model_->RobotInfo_[world_model_->AgentID_-1].getCurrentRole()!=NOROLE)
        return;
    else if(activeID!=last_activeID)
        last_activeID=activeID;

    /// PART2：其他角色的分配（并不是每个周期都分配，如果主攻角色没有发生转移，不重新分配其他角色）
    if(!world_model_->IsOurDribble_)
    {

        /// 从二号机器人开始分配角色，然后保证助攻机器人
        for(int i=1; i<OUR_TEAM; i++)
        {
            if(world_model_->RobotInfo_[i].isValid())
            {
                /// 分了主攻或防守的机器人不在考虑
                if(RoleUtilityMatrix_[i-1][2]>Assistmax && (i+1 != activemaxindex) &&
                        (i+1 != defencemaxindex))
                {
                    Assistmax=RoleUtilityMatrix_[i-1][2];
                    assistmaxindex=i+1;
                }
            }
        }
        if(assistmaxindex==world_model_->AgentID_)
        {
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ASSISTANT);
            return;
        }

        /// 从二号机器人开始分配角色，然后保证防守机器人*/
        for(int i=1; i<OUR_TEAM; i++)
        {
            if(world_model_->RobotInfo_[i].isValid())
            {
                /// 分了主攻的机器人不在考虑防守
                if(RoleUtilityMatrix_[i-1][1]>Defencemax && (i+1 != activemaxindex))
                {
                    Defencemax=RoleUtilityMatrix_[i-1][1];
                    defencemaxindex=i+1;
                }
            }
        }
        if(defencemaxindex==world_model_->AgentID_)
        {
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(PASSIVE);
            return;
        }
        /// 从二号机器人开始分配角色，最后保证中场机器人
        for(int i=1; i<OUR_TEAM; i++)
        {
            if(world_model_->RobotInfo_[i].isValid())
            {
                /// 分了主攻或防守或助攻的机器人不在考虑
                if((i+1 != activemaxindex) && (i+1 != defencemaxindex) && (i+1 != assistmaxindex))
                    midmaxindex=i+1;
            }
        }
        if(midmaxindex==world_model_->AgentID_)
        {
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(MIDFIELD);
            return;
        }
    }
    else
    {
        /// 从二号机器人开始分配角色，然后保证防守机器人*/
        for(int i=1; i<OUR_TEAM; i++)
        {
            if(world_model_->RobotInfo_[i].isValid())
            {
                /// 分了主攻的机器人不在考虑防守
                if(RoleUtilityMatrix_[i-1][1]>Defencemax && (i+1 != activemaxindex)&&
                        (i+1 != assistmaxindex))
                {
                    Defencemax=RoleUtilityMatrix_[i-1][1];
                    defencemaxindex=i+1;
                }
            }
        }
        if(defencemaxindex==world_model_->AgentID_)
        {
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(PASSIVE);
            return;
        }
        /// 从二号机器人开始分配角色，然后保证助攻机器人
        for(int i=1; i<OUR_TEAM; i++)
        {
            if(world_model_->RobotInfo_[i].isValid())
            {
                /// 分了主攻或防守的机器人不在考虑
                if(RoleUtilityMatrix_[i-1][2]>Assistmax && (i+1 != activemaxindex))
                {
                    Assistmax=RoleUtilityMatrix_[i-1][2];
                    assistmaxindex=i+1;
                }
            }
        }
        if(assistmaxindex==world_model_->AgentID_)
        {
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(ASSISTANT);
            return;
        }
        /// 从二号机器人开始分配角色，最后保证中场机器人
        for(int i=1; i<OUR_TEAM; i++)
        {
            if(world_model_->RobotInfo_[i].isValid())
            {
                /// 分了主攻或防守或助攻的机器人不在考虑
                if((i+1 != activemaxindex) && (i+1 != defencemaxindex) && (i+1 != assistmaxindex))
                    midmaxindex=i+1;
            }
        }
        if(midmaxindex==world_model_->AgentID_)
        {
            world_model_->RobotInfo_[world_model_->AgentID_-1].setCurrentRole(MIDFIELD);
            return;
        }

    }

}

void RoleAssignment::setPts(DPoint &assist, DPoint &midfield, DPoint &passive)
{
    assist_pt_ = assist;
    midfield_pt_ = midfield;
    passive_pt_ = passive;
}

bool RoleAssignment::isoppBlockWay(DPoint robot_pos, DPoint ball_pos)
{
    Line_       robot_ball_line(robot_pos,ball_pos);
    DPoint      robot_ball = ball_pos - robot_pos;
    double      projection = 0;
    DPoint      mate_pos(0,0);

    for(int i =1 ; i < OUR_TEAM ;i++)
    {
        if(world_model_->RobotInfo_[i].isValid() && i!=world_model_->AgentID_-1)
            mate_pos = world_model_->RobotInfo_[i].getLocation();

        DPoint robot_mate = mate_pos - robot_pos;
        if(robot_ball.length() != 0)
        {
            projection = robot_mate.dot(robot_ball)/robot_ball.length();
            //ROS_INFO("mate_pos[%f %f] projection:%f, robot_ball.norm:%f",
            //         mate_pos.x_, mate_pos.y_ ,projection,robot_ball.norm());
        }
        else
            return 0;

        if(projection > 0 && projection < robot_ball.length())
        {
            if(robot_ball_line.distance(mate_pos) < block_dist)
            {
                //ROS_INFO("role_assignment: mate_pos[%f %f] block my way!", mate_pos.x_, mate_pos.y_);
                return 1;
            }
        }

    }

    for(DPoint opp : world_model_->Opponents_)
    {
        DPoint robot_opp = opp - robot_pos;
        if(robot_ball.length() != 0)
        {
            projection = robot_opp.dot(robot_ball)/robot_ball.length();
            //ROS_INFO("opp[%f %f] projection:%f, robot_ball.norm:%f",
            //         opp.x_, opp.y_ ,projection,robot_ball.norm());
        }
        else
            return 0;       // 表明我跟球很近

        if(projection > 0 && projection < robot_ball.length())        // 对手在我与球之间
        {
            if(robot_ball_line.distance(opp) < block_dist)
            {
                //ROS_INFO("role_assignment: opp[%f %f] block my way!", opp.x_, opp.y_);
                return 1;
            }
        }
    }
    return 0;
}
