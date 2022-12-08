#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>
#include "sample1_4limb.h"
#include "sample2_4limb_unsolvable.h"

namespace prioritized_acc_inverse_kinematics_solver_sample{

  class sample1_4limbItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_4limbItem>("sample1_4limbItem"); }
  protected:
    virtual void main() override{ sample1_4limb(); return; }
  };
  typedef cnoid::ref_ptr<sample1_4limbItem> sample1_4limbItemPtr;

  class sample2_4limb_unsolvableItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2_4limb_unsolvableItem>("sample2_4limb_unsolvableItem"); }
  protected:
    virtual void main() override{ sample2_4limb_unsolvable(); return; }
  };
  typedef cnoid::ref_ptr<sample2_4limb_unsolvableItem> sample2_4limb_unsolvableItemPtr;


  class PrioritizedAccInverseKinematicsSolverSamplePlugin : public cnoid::Plugin
  {
  public:

    PrioritizedAccInverseKinematicsSolverSamplePlugin() : Plugin("PrioritizedAccInverseKinematicsSolverSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample1_4limbItem::initializeClass(this);
      sample2_4limb_unsolvableItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(prioritized_acc_inverse_kinematics_solver_sample::PrioritizedAccInverseKinematicsSolverSamplePlugin)
