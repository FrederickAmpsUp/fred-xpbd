#include <fxpbd/builtin_constraints.hpp>
#include <fxpbd/system.hpp>
#include <fxpbd/constraint.hpp>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <iostream>

static const int NUM_SUBSTEPS = 8;
static const float TIMESCALE = 1.0f;

static const int WINDOW_SIZE_X = 1024, WINDOW_SIZE_Y = 1024;

class ConstrainToCircle : public fxpbd::w2d::Constraint {
public:
    ConstrainToCircle(fxpbd::w2d::System& system, float radius) : system(system), radius(radius) {
    }

    virtual void solve(float dt) override {
        for (auto& particle : system.particles) {
            
            float distToCenter = glm::length(particle->position) + particle->radius;

            if (distToCenter > this->radius) {
                glm::vec2 direction = -glm::normalize(particle->position);
                particle->position += direction * (distToCenter - this->radius);
            }
        }
    }
private:
    fxpbd::w2d::System& system;
    float radius;
};

class TriangleVolumeConservation : public fxpbd::w2d::Constraint {
public:
    TriangleVolumeConservation(std::shared_ptr<fxpbd::w2d::Particle> a, std::shared_ptr<fxpbd::w2d::Particle> b, std::shared_ptr<fxpbd::w2d::Particle> c, float targetVolume, float compliance = 0.0f) : a(a), b(b), c(c), desired(targetVolume), compliance(compliance) {}

    virtual void solve(float dt) override {
            // Heron's formula for area
        float sdA = glm::distance(a->position, b->position);
        float sdB = glm::distance(b->position, c->position);
        float sdC = glm::distance(c->position, a->position);

        float s = (sdA + sdB + sdC) / 2.0f;
        float area = sqrtf(s * (s - sdA) * (s - sdB) * (s - sdC));

        float C = area - desired;

        glm::vec2 ab = b->position - a->position;
        glm::vec2 bc = c->position - b->position;
        glm::vec2 ca = a->position - c->position;

        glm::vec2 gradCa = glm::vec2(bc.y, -bc.x);
        gradCa *= glm::sign(glm::dot(gradCa, -ab));

        glm::vec2 gradCb = glm::vec2(ca.y, -ca.x);
        gradCb *= glm::sign(glm::dot(gradCb, -bc));
        
        glm::vec2 gradCc = glm::vec2(ab.y, -ab.x);
        gradCc *= glm::sign(glm::dot(gradCc, -ca));

        float stiffnessCoeff = this->compliance / (dt * dt);

        float lambda = -C / ((1.0f / a->mass) * glm::dot(gradCa, gradCa) + (1.0f / b->mass) * glm::dot(gradCb, gradCb) + (1.0f / c->mass) * glm::dot(gradCc, gradCc) + stiffnessCoeff);

        a->position += (1.0f / a->mass) * gradCa * lambda;
        b->position += (1.0f / b->mass) * gradCb * lambda;
        c->position += (1.0f / c->mass) * gradCc * lambda;
    }
private:
    std::shared_ptr<fxpbd::w2d::Particle> a, b, c;
    float desired, compliance;
};

struct ParticleData {
    sf::Color color;
};

std::tuple<float, float, float> hueToRGB(float hue) {
    // Ensure hue is within the range [0.0, 1.0]
    hue = fmod(hue, 1.0f);
    if (hue < 0.0f) hue += 1.0f;

    float r, g, b;
    if (hue < 1.0f / 6.0f) {
        r = 1.0f;
        g = 6.0f * hue;
        b = 0.0f;
    } else if (hue < 2.0f / 6.0f) {
        r = 1.0f - 6.0f * (hue - 1.0f / 6.0f);
        g = 1.0f;
        b = 0.0f;
    } else if (hue < 3.0f / 6.0f) {
        r = 0.0f;
        g = 1.0f;
        b = 6.0f * (hue - 2.0f / 6.0f);
    } else if (hue < 4.0f / 6.0f) {
        r = 0.0f;
        g = 1.0f - 6.0f * (hue - 3.0f / 6.0f);
        b = 1.0f;
    } else if (hue < 5.0f / 6.0f) {
        r = 6.0f * (hue - 4.0f / 6.0f);
        g = 0.0f;
        b = 1.0f;
    } else {
        r = 1.0f;
        g = 0.0f;
        b = 1.0f - 6.0f * (hue - 5.0f / 6.0f);
    }

    return std::make_tuple(r, g, b);
}

void createCircle(fxpbd::w2d::System& system, float radius, int numVertices, float particleRadius, float compliance) {
    // Create the center particle
    auto center = std::make_shared<fxpbd::w2d::Particle>();
    center->position = glm::vec2(0.0f, 0.0f);
    center->lastPos = center->position;
    center->mass = M_PI * particleRadius * particleRadius;
    center->radius = particleRadius;

    system.add_particle(center);

    std::vector<std::shared_ptr<fxpbd::w2d::Particle>> vertices;

    // Angle between adjacent vertices in radians
    float angleIncrement = 2.0f * M_PI / numVertices;

    // Create the particles around the circle
    for (int i = 0; i < numVertices; ++i) {
        float angle = i * angleIncrement;
        glm::vec2 position = glm::vec2(radius * cos(angle), radius * sin(angle));

        auto vertex = std::make_shared<fxpbd::w2d::Particle>();
        vertex->position = position;
        vertex->lastPos = vertex->position;
        vertex->mass = M_PI * particleRadius * particleRadius;
        vertex->radius = particleRadius;
        
        system.add_particle(vertex);
        vertices.push_back(vertex);

        // Add distance constraint between the center and this vertex
        system.add_constraint(
            std::make_shared<fxpbd::w2d::constraints::DistanceConservation>(
                center, vertex, radius, compliance
            )
        );

        // Add distance constraint between this vertex and the previous vertex (to form the circular structure)
        if (i > 0) {
            system.add_constraint(
                std::make_shared<fxpbd::w2d::constraints::DistanceConservation>(
                    vertices[i - 1], vertex, glm::distance(vertices[i - 1]->position, vertex->position), compliance
                )
            );

            float a = glm::distance(vertices[i - 1]->position, vertex->position);
            float b = glm::distance(vertex->position, center->position);
            float c = glm::distance(vertices[i - 1]->position, center->position);

            float s = (a + b + c) / 2.0f;
            float area = sqrtf(s * (s - a) * (s - b) * (s - c));

            system.add_constraint(
                std::make_shared<TriangleVolumeConservation>(
                   vertices[i - 1], vertex, center, area, 0.0f
                )
            );
        }
    }

    // Add distance constraint between the last vertex and the first vertex to complete the circle
    system.add_constraint(
        std::make_shared<fxpbd::w2d::constraints::DistanceConservation>(
            vertices.front(), vertices.front(), glm::distance(vertices.back()->position, vertices.front()->position), compliance
        )
    );

    float a = glm::distance(vertices.back()->position, vertices.front()->position);
    float b = glm::distance(vertices.front()->position, center->position);
    float c = glm::distance(vertices.back()->position, center->position);

    float s = (a + b + c) / 2.0f;
        float area = sqrtf(s * (s - a) * (s - b) * (s - c));
    system.add_constraint(
        std::make_shared<TriangleVolumeConservation>(
            vertices.front(), vertices.back(), center, area, 0.0f
        )
    );
}

int main(int argc, char **argv) {
    fxpbd::w2d::System system;
    system.gravity = glm::vec2(0.0f, -1.0f);

    std::shared_ptr<ConstrainToCircle> bound = std::make_shared<ConstrainToCircle>(system, 0.9f);
    system.add_constraint(bound);

    std::shared_ptr<fxpbd::w2d::constraints::ParticleCollision> collision = std::make_shared<fxpbd::w2d::constraints::ParticleCollision>(system);
    system.add_constraint(collision);

    sf::RenderWindow win(sf::VideoMode(WINDOW_SIZE_X, WINDOW_SIZE_Y), "FXPBD Test Window");
    win.setFramerateLimit(60);

    // test a cirlce body made of triangle volume conservations
    createCircle(system, 0.2f, 30, 0.015f, 0.1f);

    unsigned long long frameCount = 0; 

    while (win.isOpen()) {
        sf::Event event;
        while (win.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                win.close();
            }
        }

        if (frameCount % 6ULL == 0) {
            float time = (float)frameCount / 60.0f;

            auto particle = std::make_shared<fxpbd::w2d::Particle>();

            const float density = 1.0f;

            // TODO: randomize this?
            particle->position = glm::vec2(0.0f, 0.5f);
            particle->lastPos  = particle->position;
            particle->radius = 0.04f + 0.02f * sinf(time * 10000.0f);
            particle->velocity = glm::vec2(cosf(time), sinf(time)) * 2.0f * (particle->radius / 0.02f) * 0.5f;
            particle->mass = M_PI * particle->radius * particle->radius * density; // mass proportional to radius
            
            auto rgb = hueToRGB(time / 6.28f);
            particle->data = ParticleData{sf::Color(std::get<0>(rgb) * 255, std::get<1>(rgb) * 255, std::get<2>(rgb) * 255)};

            system.add_particle(particle);

            std::cout << system.particles.size() << " particles" << std::endl;
        }

        float dt = 1.0f/60.0f * TIMESCALE;
        float ssdt = dt / (float)NUM_SUBSTEPS;

        for (int i = 0; i < NUM_SUBSTEPS; i++)
            system.update(ssdt);

        win.clear(sf::Color::Black);

        sf::CircleShape border(0.9f * WINDOW_SIZE_X / 2.0, 120);
        constexpr float winCenterY = WINDOW_SIZE_Y * (0.5f * ((float)WINDOW_SIZE_X / (float)WINDOW_SIZE_Y));
        border.setPosition(WINDOW_SIZE_X / 2.0 - WINDOW_SIZE_X * 0.9f / 2.0f, winCenterY - WINDOW_SIZE_Y * 0.9f / 2.0f);
        border.setFillColor(sf::Color(20, 20, 20));

        win.draw(border);

        for (auto& particle : system.particles) {
            sf::CircleShape circle(particle->radius * WINDOW_SIZE_X / 2.0f);

            glm::vec2 uv = 0.5f + 0.5f * particle->position;
            uv.y /= (float)WINDOW_SIZE_Y / (float)WINDOW_SIZE_X;

            circle.setPosition(WINDOW_SIZE_X*(uv.x - particle->radius/2.0f), WINDOW_SIZE_Y * (1.0 - uv.y - particle->radius/2.0f));

            try {
                circle.setFillColor(std::any_cast<ParticleData>(particle->data).color);
            } catch (std::bad_any_cast& e) {
                circle.setFillColor(sf::Color::White);
            }
            win.draw(circle);
        }

        win.display();

        ++frameCount;
    }
}