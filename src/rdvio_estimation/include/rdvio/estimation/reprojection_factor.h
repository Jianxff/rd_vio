#pragma once

#include <rdvio/types.h>

namespace rdvio {

class ReprojectionErrorFactor {
  public:
    virtual ~ReprojectionErrorFactor() = default;

  protected:
    ReprojectionErrorFactor() = default;
};

class ReprojectionPriorFactor {
  public:
    virtual ~ReprojectionPriorFactor() = default;

  protected:
    ReprojectionPriorFactor() = default;
};

} // namespace rdvio

