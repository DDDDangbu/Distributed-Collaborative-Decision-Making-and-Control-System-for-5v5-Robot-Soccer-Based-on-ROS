#ifndef PASSIVEROLE_H
#define PASSIVEROLE_H

#include <nubot_control/plan.h>
#include <algorithm>

namespace nubot {

struct Compare
{
    Compare(DPoint ref_pt){ refPt_ = ref_pt; }
    bool operator() (DPoint i, DPoint j)
    {
        return (i.distance(refPt_) < j.distance(refPt_)) ;
    }

    DPoint refPt_;
};

class PassiveRole
{
public:
      PassiveRole();
     ~PassiveRole();

      void   process();
      void   move2PassivePt();
      void   passiveCalculate();                                //新的站位点计算
#if 0
      bool   getNearestOpp(DPoint &loc, DPoint ref_pt, bool ourfield);
#else
      DPoint getNearestOpp();
#endif
      DPoint getPointInline(DPoint start, DPoint end, double dis);
      int    numInOurPenalty(DPoint notConsiderPt);

      void sortIncreasing(std::vector<DPoint>& pos_vec, DPoint ref_pt);        // sort vector of DPoint by increasing order with reference to ref_pt

public:
      World_Model_Info * world_model_;
      Plan *plan_;

      bool isInOurfeild_;                     //用于站位决策的两个因素
      bool IsOurDribble_;
      DPoint passive_pt_;
};

}
#endif // PASSIVEROLE_H
