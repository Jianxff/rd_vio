#pragma once

#include <rdvio/types.h>

namespace rdvio {

class PreIntegrationErrorFactor {
  public:
    virtual ~PreIntegrationErrorFactor() = default;

  protected:
    PreIntegrationErrorFactor() = default;
};

class PreIntegrationPriorFactor {
  public:
    virtual ~PreIntegrationPriorFactor() = default;

  protected:
    PreIntegrationPriorFactor() = default;
};

} // namespace rdvio

