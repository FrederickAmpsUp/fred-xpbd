#pragma once

#include <cmath>

namespace fxpbd::w2d {

/**
 * @brief Base class for representing any PBD constraint
 */
class Constraint {
public:
    /**
     * @brief Solve the constraint, usually modifying references/pointers to particles in a system
     * @param dt The timestep size, in seconds
     */
    virtual void solve(float dt) = 0;
private:
};
}