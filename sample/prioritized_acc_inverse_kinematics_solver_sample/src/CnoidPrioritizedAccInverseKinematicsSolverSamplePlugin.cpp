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

  void sample6_wolimit();
  class sample6_wolimitItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample6_wolimitItem>("sample6_wolimitItem"); }
  protected:
    virtual void main() override{ sample6_wolimit(); return; }
  };
  typedef cnoid::ref_ptr<sample6_wolimitItem> sample6_wolimitItemPtr;

  void sample7_limit();
  class sample7_limitItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample7_limitItem>("sample7_limitItem"); }
  protected:
    virtual void main() override{ sample7_limit(); return; }
  };
  typedef cnoid::ref_ptr<sample7_limitItem> sample7_limitItemPtr;

  void sample8_collision();
  class sample8_collisionItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample8_collisionItem>("sample8_collisionItem"); }
  protected:
    virtual void main() override{ sample8_collision(); return; }
  };
  typedef cnoid::ref_ptr<sample8_collisionItem> sample8_collisionItemPtr;


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
      sample6_wolimitItem::initializeClass(this);
      sample7_limitItem::initializeClass(this);
      sample8_collisionItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(prioritized_acc_inverse_kinematics_solver_sample::PrioritizedAccInverseKinematicsSolverSamplePlugin)
