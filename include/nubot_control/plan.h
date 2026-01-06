#ifndef PLAN_H
#define PLAN_H

#include <cmath>
#include "nubot_control/subtargets.h"
#include "nubot_control/behaviour.hpp"
#include "nubot_control/world_model_info.h"

using namespace std;
namespace nubot{
class Plan
{
public:
        Plan();

        void catchBall();
        void catchBallForCoop();
        void catchBallSlowly();
        void catchMovingBall();
        void catchMotionlessBall();

        void   move2Positionwithobs(DPoint target, float maxvel, bool avoid_ball=false,bool is_static_passing=false);
        void   move2Positionwithobs_withball(DPoint target, float maxvel);
        void   update();
        DPoint repulsive2obstacle(float threshold, DPoint obstacle_pos, DPoint robot_pos, Angle robot_ori, float lamda=10);
        DPoint getWhichRolePos(Roles role);
        Angle getWhichRoleOri(Roles role);
        int    getWhichRoleID(Roles role);
        DPoint getAvailablePosIn3m(DPoint dribble_pos);
        bool   isNullInTrap(const std::vector<DPoint> & obs_info, const DPoint & robot_pos, const Angle & direction,double back_width, double front_width, double back_len, double front_len);
        bool   rotateMode(const std::vector<DPoint> & obs_info, const DPoint & robot_pos, const Angle & robot_direction, const Angle & target_direction);
        bool   pnpoly(const std::vector<DPoint> & pts, const std::vector<DPoint> & obs_pts);
        int    rand_num(int l, int r);
public:
        World_Model_Info * world_model_;
        Behaviour  m_behaviour_;
        Subtargets m_subtargets_;

        float kp;
        float kalpha;
        float kbeta;

        DPoint robot_pos_;
        Angle  robot_ori_;
        DPoint robot_vec_;
        DPoint ball_pos_;
        DPoint ball_vec_;

        vector<DPoint> target_;
};
}
#endif //PLAN_H
