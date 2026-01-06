#ifndef MIDFIELDROLE_H
#define MIDFIELDROLE_H

#include <nubot_control/plan.h>

namespace nubot{

class MidfieldRole           //中场的策略，若在己方半场，盯人防守，若在对方半场跟助攻一样，站位接球
{
public:
      MidfieldRole();
     ~MidfieldRole();

      void asAssist();
      void asPassiver();
      void process();

      void midCalculate();
      void move2MidPt();
      bool getNearestOpp(DPoint &loc, DPoint ref_pt, bool ourfield);
      DPoint getPointInline(DPoint start, DPoint end, double dis);
      bool isBestPass(DPoint world_pt);               //是否在最佳的站位区域
      bool isObsOpp(DPoint candidate_point);          //是否在对方的射门角度内
      bool isObsActiver(DPoint candidate_point);      //是否在我方的射门角度内

public:
      World_Model_Info * world_model_;
      Plan   *plan_;
      DPoint midfield_pt_;

private:
      DPoint    assist_pt_;              // 助攻的位置
      DPoint    active_pt_;              // 主攻的位置
};

}
#endif // MIDFIELDROLE_H
