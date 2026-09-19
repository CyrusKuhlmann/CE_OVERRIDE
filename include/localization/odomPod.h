#pragma once

#include <cstdint>

class OdomPod {
public:
    explicit OdomPod(double wheelDiameterIn);  // in

    double update(int32_t centidegrees);  // returns delta in
    void seed(int32_t centidegrees);
    double totalInches() const;  // in

private:
    double inchesPerCentidegree_;
    int32_t prevCentideg_ = 0;
    double totalInches_ = 0.0;
    bool seeded_ = false;
};
