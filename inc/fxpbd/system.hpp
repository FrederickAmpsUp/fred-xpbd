#pragma once

#include <fxpbd/particle.hpp>
#include <fxpbd/constraint.hpp>
#include <vector>
#include <memory>

namespace fxpbd::w2d {

/**
 * @brief Structure representing a system of particles and constraints
 */
struct System {
    /**
     * @brief List of particles to simulate
     * This should generally not be directly modified, but reading is fine
     */
    std::vector<std::shared_ptr<Particle>> particles;

    /**
     * @brief List of constraints to simulate
     * This should generally not be directly modified, but reading is fine
     */
    std::vector<std::shared_ptr<Constraint>> constraints;

    /**
     * @brief Gravitational force applied to all particles in the system
     */
    glm::vec2 gravity;

    /**
     * @brief Run a single step of the system
     * @param dt Timestep size, in seconds
     */
    void update(float dt) {
        for (auto& particle : this->particles) {
            particle->velocity += dt * gravity;
            particle->lastPos = particle->position;
            particle->position += dt * particle->velocity;
        }

        for (auto& constraint : this->constraints) {
            constraint->solve(dt);
        }

        for (auto& particle : this->particles) {
            particle->velocity = (particle->position - particle->lastPos) / dt;
        }
    }

    /**
     * @brief Add a particle to the system
     * @param particle The particle to add
     */
    void add_particle(const std::shared_ptr<Particle>& particle) {
        this->particles.push_back(particle);
    }

    /**
     * @brief Add a constraint to the system
     * @param constraint The constraint to add
     */
    void add_constraint(const std::shared_ptr<Constraint>& constraint) {
        this->constraints.push_back(constraint);
    }
};
}