#pragma once

#include <glm/glm.hpp>
#include <any>

namespace fxpbd::w2d {

/**
 * @brief Data Structure representing a single particle
 */
struct Particle {
    /**
     * @brief The position of the particle, in spatial units (s)
     * This field should be used and modified by constraints
     */
    glm::vec2 position;

    /**
     * @brief The previous position of the particle, in spatial units (s)
     * This field should NOT be modified by constraints
     */
    glm::vec2 lastPos;

    /**
     * @brief The velocity of the particle, in s/sec
     */
    glm::vec2 velocity;

    /**
     * @brief The mass of the particle, in mass units (m)
     */
    float mass;

    /**
     * @brief The radius of the particle, in spatial units (s)
     */
    float radius;

    /**
     * @brief Arbitrary data the user wishes to store with the particle
     */
    std::any data;
};

}