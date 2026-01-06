#ifndef STATICPASS_H
#define STATICPASS_H

#include <nubot_control/world_model_info.h>
#include <nubot_control/nubotcontrolConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nubot_control/common.hpp>

#define PI 3.1415926
#define OUR_PENALTY_SIDE 0
#define OPP_PENALTY_SIDE 1
#define sgn(x) ( ((x)<0) ? (-1) : (1) )

const double TAC_DIST_ANTI=330;                      //对方发球时，我方机器人与球之间的距离
const double TAC_DIST_DEF=100;
const double TAC_DIST_DROPBALL=150;                  //Dropball时，我方机器人与球之间的距离
const double TAC_DIST_PASS=50;                       //我方开球时，传球机器人与球之间的距离
const double TAC_DIST_CATCH=250;                     //我方开球时，接球机器人与球之间的距离
const double ME_OPP_DIS = 55;                        //我方机器人和对方机器人站位距离
const double RADIUS = TAC_DIST_ANTI + ME_OPP_DIS;
const double OppDefaultDis = 350.0;                  //对方发球时我方机器人离球的距离，这里考虑了定位误差
const double Ang2Rad=PI/180.0;                       //角度转换为弧度

namespace nubot {

class StaticPass                                     //先考虑能看到球的情况
{
public:
    StaticPass();
    ~StaticPass();

    bool    fuseBall_();                             //融合球的位置
    bool    fuseOpp_();                              //融合对方机器人的位置
    bool    islargestNum_();                         //编号最大的机器人
    int     getNearestRobot_(DPoint Pt);             //找到距离该点最近的机器人
    //void    targetReorder(DPoint* pts);            //对站位点重新排序
    void    targetReorder(vector<DPoint> &pts);
    //void    targetReorder(DPoint* pts);
    void    targetAllocation();                      //合理的分配target
    void    targetInitialize();                      //目标点初始化
    DPoint  targetAdjust(DPoint tmp);            //调整出界的站位点
    DPoint  defendTargetAdjust(DPoint pt_in, DPoint ref_pt);            // 在对方机器人开球时，调整出界以及在大禁区内的站位点
    bool    OptimizedinOurPenalty(DPoint &tmp);               // 如果是对方发球而我在可以在大禁区内，则调整为在大禁区内
    DPoint  RotAroundBallOpt(DPoint curren_pos, double rad);            // 绕足球旋转一定弧度
    DPoint  collisionAvoid_(DPoint tmp);                       //避免机器人的碰撞
    void    point4OurReady_(bool IsInOurArea);                   //我方发球时，站位点的计算
    DPoint  calOptPos(DPoint ball, DPoint opp, DPoint direction, double k);
    int     numInPenaltyArea(int which_side);

    void    staticReady_();                         //准备站位
    void    OurDefaultReady_();
    void    OppDefaultReady_();

    void    OppCornerReady_();
    void    OppGoalReady_();

    void    OurPenaltyReady_();
    void    OppPenaltyReady_();

    void    OurkickoffReady_();
    void    OppkickoffReady_();

    void    DropBallReady_();
    DPoint  stablePosition(DPoint current_pos, DPoint pre_pos, double thres);          //通过判断上一次的上一次的站位点，减少站位点的抖动
    
public:

    World_Model_Info *  world_model_;
    FieldInformation    field_;

    bool    isPosition_;                             //是否分配完毕无冲突
    int     largestNum_;                             //在场编号最大的机器人
    bool    isAllocation_[OUR_TEAM];                 //该机器人是否分配位置
    int     m_nCanBeInPenaltyArea;                   //能在禁区中的机器人个数
    int     m_nPassNumber_;                          //传球机器人编号
    int     m_nCatchNumber_;                         //接球机器人编号
    int     ballNumber_;                             //最后采用球的编号
    int     obsNumber_;                              //最后采用障碍物的编号
    int     targetNumber_[OUR_TEAM];                 //机器人最后分配的目标点编号
    vector<DPoint>   fuseOpps_;                      //融合后的对方机器人位置vector
    vector<DPoint>   lscp_;                          //机器人与球的连线与我方大禁区的交点
    DPoint  targetInit_[OUR_TEAM];                   //未分配的初始目标点        0:守门员位置，1:发球机器人位置，2:接球机器人位置，3and4:其他位置
    DPoint  target_;                                 //该机器人的站位点
    DPoint  preTarget_;                              //上次的目标点
    DPoint  ballPos_;                                //混合后的球坐标
    DPoint  oppPos_;                                 //单个对方机器位置
    DPoint  backFieldPoint_;                         //后卫点
    bool    isOurRobot(DPoint tmp);
    int     index_;                                  //已经确定好的targetInit的下标

    /* 对方开球时的站位最优点的参数值。k_越大，角度的影响越大，否则机器人的距离影响越大 */
    double  k_;
};

}
#endif //STATICPASS_H
