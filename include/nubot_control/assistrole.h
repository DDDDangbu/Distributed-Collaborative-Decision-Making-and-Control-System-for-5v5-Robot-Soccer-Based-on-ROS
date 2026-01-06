#ifndef ASSISTROLE_H
#define ASSISTROLE_H

#include <nubot_control/plan.h>

namespace nubot {

struct LocationWeight
{
    Circle circle_;
    int weight_;
    double disweight_;
};

class AssistRole
{
public:
    AssistRole();
    ~AssistRole();

    void process();     //根据当前状态选择需要执行的动作,现在只考虑了站位
    void assistCalculate();
    bool findEmptyPlace();
    void move2AssitPt();
    /** 根据与_point_from距离_max_distance以内的障碍物信息，搜索一个视野最佳的方向，可用于传球配合等功能 */
    double searchMaxSight(double &dest_angle, const DPoint &_point_from, const double &_angle_min, const double &_angle_max, const double &_max_distance);
    bool isBestPass(DPoint world_pt);       //是否在最佳的接球区域
    bool isObsActiver(DPoint candidate_point);     //是否阻碍到主攻射门

public:
    World_Model_Info * world_model_;
    Plan *plan_;

    bool                            isInOurfeild_;                     //用于站位决策的两个因素
    bool                            IsOurDribble_;
    bool                            init_finished_;
    std::vector<LocationWeight>     samples_;
    DPoint                          assist_pt_;
    std::vector<double> angle_;
    std::vector<double> angle_dis_;
    double pass_direction_;
    bool   can_pass_ ;
    double pass_sight_;
};

}
#endif // ASSISTROLE_H
