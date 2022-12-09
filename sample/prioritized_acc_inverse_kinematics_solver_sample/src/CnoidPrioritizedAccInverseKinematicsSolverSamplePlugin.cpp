#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace prioritized_acc_inverse_kinematics_solver_sample{
  void sample1_4limb();
  class sample1_4limbItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_4limbItem>("sample1_4limbItem"); }
  protected:
    virtual void main() override{ sample1_4limb(); return; }
  };
  typedef cnoid::ref_ptr<sample1_4limbItem> sample1_4limbItemPtr;

  void sample2_4limb_unsolvable();
  class sample2_4limb_unsolvableItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2_4limb_unsolvableItem>("sample2_4limb_unsolvableItem"); }
  protected:
    virtual void main() override{ sample2_4limb_unsolvable(); return; }
  };
  typedef cnoid::ref_ptr<sample2_4limb_unsolvableItem> sample2_4limb_unsolvableItemPtr;

  void sample3_4limb_angle();
  class sample3_4limb_angleItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample3_4limb_angleItem>("sample3_4limb_angleItem"); }
  protected:
    virtual void main() override{ sample3_4limb_angle(); return; }
  };
  typedef cnoid::ref_ptr<sample3_4limb_angleItem> sample3_4limb_angleItemPtr;

  void sample4_4limb_move();
  class sample4_4limb_moveItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample4_4limb_moveItem>("sample4_4limb_moveItem"); }
  protected:
    virtual void main() override{ sample4_4limb_move(); return; }
  };
  typedef cnoid::ref_ptr<sample4_4limb_moveItem> sample4_4limb_moveItemPtr;

  void sample5_com();
  class sample5_comItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample5_comItem>("sample5_comItem"); }
  protected:
    virtual void main() override{ sample5_com(); return; }
  };
  typedef cnoid::ref_ptr<sample5_comItem> sample5_comItemPtr;


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
      sample3_4limb_angleItem::initializeClass(this);
      sample4_4limb_moveItem::initializeClass(this);
      sample5_comItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(prioritized_acc_inverse_kinematics_solver_sample::PrioritizedAccInverseKinematicsSolverSamplePlugin)
