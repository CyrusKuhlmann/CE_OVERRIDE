#include "localization/odomPod.h"

#include <cmath>
#include <cstdint>

OdomPod::OdomPod(double wheelDiameterIn)
    : inchesPerCentidegree_(M_PI * wheelDiameterIn / 36000.0) {}

double OdomPod::update(int32_t centidegrees) {
    if (!seeded_) {
        prevCentideg_ = centidegrees;
        seeded_ = true;
        return 0.0;
    }
    const int64_t delta = static_cast<int64_t>(centidegrees) - static_cast<int64_t>(prevCentideg_);
    prevCentideg_ = centidegrees;
    const double deltaIn = static_cast<double>(delta) * inchesPerCentidegree_;
    totalInches_ += deltaIn;
    return deltaIn;
}

void OdomPod::seed(int32_t centidegrees) {
    prevCentideg_ = centidegrees;
    seeded_ = true;
}

double OdomPod::totalInches() const { return totalInches_; }
