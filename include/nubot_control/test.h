#ifndef TEST_H
#define TEST_H

#include "nubot_control/activerole.h"

namespace nubot{
class NubotTest
{
public:
    NubotTest();
    NubotTest(World_Model_Info& _world_model,Plan& _plan,ActiveRole& _active_role);
    void TestStart2End(char id_A, DPoint& startPos, Angle& startOri, DPoint& endPos, Angle& endOri, char& Mode);//测试函数，点到点跑位
    bool TestShoot(char id_A, DPoint& kick_target, char& delta_force);
    void TestCircle(char id_A, double &vx, double &radius);
    void TestLocation(char id_A);
    void TestPass(char&id_A,char& id_B,DPoint& PointA,DPoint& PointB,bool& is_pass);
    bool TestCatchBall(char id_A);

public:
    World_Model_Info* t_world_model;
    Plan* t_plan;
    ActiveRole* t_active_role;
    bool trigger_shoot_;
};
}
#endif // TEST_H
