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

int main(int argc, char **argv) {
    fxpbd::w2d::System system;
    system.gravity = glm::vec2(0.0f, -1.0f);

    std::shared_ptr<ConstrainToCircle> bound = std::make_shared<ConstrainToCircle>(system, 0.9f);
    system.add_constraint(bound);

    std::shared_ptr<fxpbd::w2d::constraints::ParticleCollision> collision = std::make_shared<fxpbd::w2d::constraints::ParticleCollision>(system);
    system.add_constraint(collision);

    auto a = std::make_shared<fxpbd::w2d::Particle>();
    a->position = glm::vec2(-0.5, 0.0);
    system.add_constraint(std::make_shared<fxpbd::w2d::constraints::LockPosition>(a));

    std::shared_ptr<fxpbd::w2d::Particle> b = nullptr;

    const int links = 20;
    for (int i = 0; i < links; i++) {
        constexpr float dist = 1.0 / (float)links;

        b = std::make_shared<fxpbd::w2d::Particle>();
        b->position = a->position + glm::vec2(dist, 0.0);

        a->mass = 1.0;
        b->mass = 1.0;
        
        a->radius = dist / 2.1;
        b->radius = dist / 2.1;

        std::shared_ptr<fxpbd::w2d::constraints::DistanceConservation> link = std::make_shared<fxpbd::w2d::constraints::DistanceConservation>(a, b, dist);
        system.add_constraint(link);

        system.add_particle(a);

        a->data = ParticleData{sf::Color::White};

        a = b;
    }

    b->data = ParticleData{sf::Color::White};

    system.add_constraint(std::make_shared<fxpbd::w2d::constraints::LockPosition>(b));
    system.add_particle(b);

    sf::RenderWindow win(sf::VideoMode(WINDOW_SIZE_X, WINDOW_SIZE_Y), "FXPBD Test Window");
    win.setFramerateLimit(60);

    unsigned long long frameCount = 0; 

    while (win.isOpen()) {
        sf::Event event;
        while (win.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                win.close();
            }
        }

        if (frameCount % 3ULL == 0) {
            float time = (float)frameCount / 60.0f;

            auto particle = std::make_shared<fxpbd::w2d::Particle>();

            // TODO: randomize this?
            particle->position = glm::vec2(0.0f, 0.5f);
            particle->lastPos  = particle->position;
            particle->velocity = glm::vec2(cosf(time), sinf(time)) * 2.0f;
            particle->mass = 1.0f;
            particle->radius = 0.02f;
            
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