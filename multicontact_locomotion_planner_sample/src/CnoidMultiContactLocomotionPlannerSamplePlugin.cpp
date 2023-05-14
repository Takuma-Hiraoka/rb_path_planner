#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace multicontact_locomotion_planner_sample{
  void sample1_walk();
  class sample1_walkItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_walkItem>("sample1_walkItem"); }
  protected:
    virtual void main() override{ sample1_walk(); return; }
  };
  typedef cnoid::ref_ptr<sample1_walkItem> sample1_walkItemPtr;

  void sample2_quad();
  class sample2_quadItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2_quadItem>("sample2_quadItem"); }
  protected:
    virtual void main() override{ sample2_quad(); return; }
  };
  typedef cnoid::ref_ptr<sample2_quadItem> sample2_quadItemPtr;

  class MultiContactLocomotionPlannerSamplePlugin : public cnoid::Plugin
  {
  public:

    MultiContactLocomotionPlannerSamplePlugin() : Plugin("MultiContactLocomotionPlannerSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample1_walkItem::initializeClass(this);
      sample2_quadItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(multicontact_locomotion_planner_sample::MultiContactLocomotionPlannerSamplePlugin)
