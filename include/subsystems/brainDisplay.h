#pragma once

#include "command/subsystem.h"

// Owns the brain LCD. periodic() runs every scheduler tick, including while disabled.
class BrainDisplay : public Subsystem {
public:
    void periodic() override;
};
