//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANNER_MODULES_PLANEXP_PLANEXP_PACKAGE_H
#define PLANNER_MODULES_PLANEXP_PLANEXP_PACKAGE_H

namespace active_3d_planning {
  namespace initialize {
// force the linker to include this lib
// TODO(schmluk): there's probably a better way to do this...
    void planexp_package();

  }  // namespace initialize
}  // namespace active_3d_planning

#endif //PLANNER_MODULES_PLANEXP_PLANEXP_PACKAGE_H
