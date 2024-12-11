#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <choreonoid_viewer/choreonoid_viewer.h>

namespace rbrrt_sample{
  void view();
  class viewItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<viewItem>("viewItem"); }
  protected:
    virtual void main() override{ view(); return;}
  };
  typedef cnoid::ref_ptr<viewItem> viewItemPtr;

  class RBRRTSamplePlugin : public cnoid::Plugin
  {
  public:
    RBRRTSamplePlugin() : Plugin("RBRRTSamplePlugin")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      viewItem::initializeClass(this);
      return true;
    }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(rbrrt_sample::RBRRTSamplePlugin)
