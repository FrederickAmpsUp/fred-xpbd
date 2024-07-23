#pragma once

#include <fxpbd/system.hpp>
#include <fxpbd/constraint.hpp>
#include <glm/glm.hpp>

namespace fxpbd::w2d::constraints {

/**
 * @brief Implement collisions between all particles in the system
 */
class ParticleCollision : public Constraint {
public:
    /**
     * @brief Constructor
     */
    ParticleCollision(fxpbd::w2d::System& system) : system(system) {}

    /**
     * @brief Solve collisions between all particles in the system, modifying particle position to avoid intersections.
     * @param dt The timestep size, in seconds
     */
    virtual void solve(float dt) override {
            // TODO: spatial partitioning optimization
        for (int i = 0; i < system.particles.size(); i++) {
            auto& particle = system.particles[i];
            for (int j = i + 1; j < system.particles.size(); j++) {
                auto& other = system.particles[j];

                float minDistSq = (particle->radius + other->radius) * (particle->radius + other->radius);

                float distSq = glm::dot(particle->position - other->position, particle->position - other->position);

                if (distSq < minDistSq) {
                    float totalInverseMass = (1.0f / particle->mass) + (1.0f / other->mass);
                    float dist = sqrt(distSq);
                    float travelDist = sqrt(minDistSq) - dist;

                    glm::vec2 dir = (particle->position - other->position) / dist;

                    particle->position += ((1.0f / particle->mass) / totalInverseMass) * dir * travelDist;
                    other->position    -= ((1.0f /    other->mass) / totalInverseMass) * dir * travelDist;
                }
            }
        }
    }
private:
    fxpbd::w2d::System& system;
};

/**
 * @brief Keep any 2 particles a specified distance apart
 */
class DistanceConservation : public Constraint {
public:
    /**
     * @brief Constructor
     */
    DistanceConservation(std::shared_ptr<fxpbd::w2d::Particle>& a, std::shared_ptr<fxpbd::w2d::Particle>& b, float length, float compliance = 0.0f) : a(a), b(b), restingLength(length), compliance(compliance) {}

    /**
     * @brief Solve the distance constraints
     * @param dt The timestep size, in seconds
     */
    virtual void solve(float dt) override {
        float l = glm::distance(a->position, b->position);

        float stiffnessCoeff = this->compliance / (dt * dt);

        float C = this->restingLength - l;
        glm::vec2 gradC = (b->position - a->position);

        float lambda = -C / ((1.0 / a->mass) + (1.0 / b->mass) + stiffnessCoeff);

        a->position += (1.0f / a->mass) * lambda * gradC;
        b->position -= (1.0f / b->mass) * lambda * gradC;
    }

private:
    std::shared_ptr<fxpbd::w2d::Particle> a, b;
    float restingLength, compliance;
};

/**
 * @brief Lock any particle in place
 */
class LockPosition : public Constraint {
public:
    /**
     * @brief Constructor
     */
    LockPosition(std::shared_ptr<fxpbd::w2d::Particle>& particle) : particle(particle) {
        this->position = particle->position;
    }

    /**
     * @brief Solve the constraint
     * @param dt The timestep size, in seconds
     */
    virtual void solve(float dt) override {
        particle->position = position;
    }
private:
    std::shared_ptr<fxpbd::w2d::Particle> particle;
    glm::vec2 position;
};
}