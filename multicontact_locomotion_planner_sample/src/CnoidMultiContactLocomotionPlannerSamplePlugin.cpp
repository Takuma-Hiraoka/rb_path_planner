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

  void sample3_desk();
  class sample3_deskItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample3_deskItem>("sample3_deskItem"); }
  protected:
    virtual void main() override{ sample3_desk(); return; }
  };
  typedef cnoid::ref_ptr<sample3_deskItem> sample3_deskItemPtr;

  void sample4_ladder();
  class sample4_ladderItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample4_ladderItem>("sample4_ladderItem"); }
  protected:
    virtual void main() override{ sample4_ladder(); return; }
  };
  typedef cnoid::ref_ptr<sample4_ladderItem> sample4_ladderItemPtr;

  void sample0_display();
  class sample0_displayItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample0_displayItem>("sample0_displayItem"); }
  protected:
    virtual void main() override{ sample0_display(); return; }
  };
  typedef cnoid::ref_ptr<sample0_displayItem> sample0_displayItemPtr;

  class MultiContactLocomotionPlannerSamplePlugin : public cnoid::Plugin
  {
  public:

    MultiContactLocomotionPlannerSamplePlugin() : Plugin("MultiContactLocomotionPlannerSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample0_displayItem::initializeClass(this);
      sample1_walkItem::initializeClass(this);
      sample2_quadItem::initializeClass(this);
      sample3_deskItem::initializeClass(this);
      sample4_ladderItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(multicontact_locomotion_planner_sample::MultiContactLocomotionPlannerSamplePlugin)
