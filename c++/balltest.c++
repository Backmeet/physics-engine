#include "libs/engine.hpp"
#include "E:\vs code\files\winhelp\src\winhelp.hpp"
#include <iostream>

void render(winhelp::display& display) {
    for (const engine::circle& c : engine::circles) {
        winhelp::draw::circle(display.surface, c.position, c.radius, engine::config.circleColour, 1.0f);
    }

    for (const engine::line& l : engine::lines) {
        winhelp::draw::line(display.surface, l.start, l.end, engine::config.lineColour, (float)engine::config.lineThickness);
    }
}

int main() {
    engine::lines = {
        engine::line({0, 0}, {0, engine::config.H}),
        engine::line({engine::config.W-5,                0}, {engine::config.W-5, engine::config.H     }),
        engine::line({0            , engine::config.H - 50}, {engine::config.W  , engine::config.H - 50})
    };

    winhelp::display display({engine::config.W, engine::config.H}, "Ball test");
    float dt;
    engine::config.gravity = 3000;
    engine::config.friction = 0.3;
    constexpr int substeps = 5;

    
    while (1) {
        display.surface.fill({20, 20, 30});
        for (const winhelp::events::event& e : winhelp::events::get()) {
            if (e.type == winhelp::events::eventTypes::quit) {
                display.close();
                return 0;
            }
            if (e.type == winhelp::events::eventTypes::key_up) {
                if (e.keys.size() > 0 && e.keys[0] == winhelp::events::key::Space) {
                    engine::circles.push_back(engine::circle(engine::config.center, 15, {-500, 0}));
                }
            }
        }

        dt = 1.0f / winhelp::fps;
        float stepDt = dt / substeps;

        for (int i = 0; i < substeps; ++i) {
            engine::simulate(stepDt);
        }
        render(display);

        display.flip();
        winhelp::tick(60);
    }

}