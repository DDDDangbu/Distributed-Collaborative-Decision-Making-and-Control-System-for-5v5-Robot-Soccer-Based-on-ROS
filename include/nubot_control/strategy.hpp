#ifndef STRATEGY_H
#define STRATEGY_H
#include <cmath>
#include "nubot_control/role_assignment.h"
#include "nubot_control/goaliestrategy.h"
#include "nubot_control/activerole.h"
#include "nubot_control/midfieldrole.h"
#include "nubot_control/passiverole.h"
#include "nubot_control/assistrole.h"
namespace nubot{

class Strategy
{

public:
    Strategy();
    Strategy(World_Model_Info & _world_model, Plan & _plan);
    ~Strategy();
   void selectRole();
   void selectAction();
   void process();
   void calPassPositions();
   bool passStrategy();

public:
    RoleAssignment    RoleAssignment_;
    World_Model_Info * world_model_;
    int selected_role_;
    int selected_action_;
    Plan * m_plan_;
    ActiveRole        ActiveRole_;
    AssistRole        AssistRole_;
    PassiveRole       PassiveRole_;
    MidfieldRole      MidfieldRole_;
    GoalieStrategy    goalie_strategy_;
    bool  auto_competition;

    DPoint  assist_pt_;
    DPoint  midfield_pt_;
    DPoint  passive_pt_;
};

}
#endif // STRATEGY_H
