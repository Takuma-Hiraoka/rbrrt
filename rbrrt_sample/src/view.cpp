#include "jaxon_common.h"

namespace rbrrt_sample {
  void view() {
    std::shared_ptr<rbrrt::RBRRTParam> param = std::make_shared<rbrrt::RBRRTParam>();
    generateJAXON(param);
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(param->robot);
    viewer->objects(param->abstractRobot);
    viewer->drawObjects();
  }
}
