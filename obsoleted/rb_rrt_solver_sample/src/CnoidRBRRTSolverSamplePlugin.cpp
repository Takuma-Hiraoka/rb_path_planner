#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace rb_rrt_solver_sample{
  void sample1_4limb();
  class sample1_4limbItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_4limbItem>("sample1_4limbItem"); }
  protected:
    virtual void main() override{ sample1_4limb(); return; }
  };
  typedef cnoid::ref_ptr<sample1_4limbItem> sample1_4limbItemPtr;

  void sample2_root();
  class sample2_rootItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2_rootItem>("sample2_rootItem"); }
  protected:
    virtual void main() override{ sample2_root(); return; }
  };
  typedef cnoid::ref_ptr<sample2_rootItem> sample2_rootItemPtr;

  void sample3_path();
  class sample3_pathItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample3_pathItem>("sample3_pathItem"); }
  protected:
    virtual void main() override{ sample3_path(); return; }
  };
  typedef cnoid::ref_ptr<sample3_pathItem> sample3_pathItemPtr;

  class RBRRTSolverSamplePlugin : public cnoid::Plugin
  {
  public:

    RBRRTSolverSamplePlugin() : Plugin("RBRRTSolverSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample1_4limbItem::initializeClass(this);
      sample2_rootItem::initializeClass(this);
      sample3_pathItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(rb_rrt_solver_sample::RBRRTSolverSamplePlugin)
