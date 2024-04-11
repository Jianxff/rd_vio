#pragma once

#include <rdvio/types.h>

namespace rdvio {

class RotationPriorFactor {
  public:
    virtual ~RotationPriorFactor() = default;

  protected:
    RotationPriorFactor() = default;
};

} // namespace rdvio

