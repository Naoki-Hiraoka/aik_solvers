#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace acc_lp_solver_sample{
  void sample1();
  class sample1Item : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1Item>("sample1Item"); }
  protected:
    virtual void main() override{ sample1(); return; }
  };
  typedef cnoid::ref_ptr<sample1Item> sample1ItemPtr;

  class AccLpSolverSamplePlugin : public cnoid::Plugin
  {
  public:

    AccLpSolverSamplePlugin() : Plugin("AccLpSolverSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample1Item::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(acc_lp_solver_sample::AccLpSolverSamplePlugin)
