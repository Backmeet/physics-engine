#include "../libs/engine.hpp"
#include "../libs/winhelp.hpp"
#include <iostream>

static bool lineBegin = false;
static winhelp::vec2 lineStart;
static winhelp::vec2 lineEnd;

void render(winhelp::display& display) {
    for (const engine::circle& c : engine::circles) {
        winhelp::draw::circle(display.surface, c.position, c.radius, engine::config.circleColour, 1.0f);
    }

    for (const engine::line& l : engine::lines) {
        winhelp::draw::line(display.surface, l.start, l.end, engine::config.lineColour, (float)engine::config.lineThickness);
    }

    if (lineBegin) {
        winhelp::draw::line(display.surface, lineStart, lineEnd, engine::config.lineColour, (float)engine::config.lineThickness);
    }
}

static float dt;
static constexpr int substeps = 5;

void simulate() {
    dt = 1.0f / winhelp::fps;
    float stepDt = dt / substeps;
    
    for (int i = 0; i < substeps; ++i) {
        engine::simulate(stepDt);
    }
}

int main() {
    engine::lines = {};
    engine::circles = {};

    winhelp::display display({engine::config.W, engine::config.H}, "Ball test");
    engine::config.gravity = 3000;
    engine::config.friction = 0.3;

    enum part {LINE=1, CIRCLE, SPRING};
    part selectedType = LINE;
    int circleSize = 10;

    while (1) {
        display.surface.fill(winhelp::vec3(20, 20, 30));
        for (const winhelp::events::event& e : winhelp::events::get()) {
            if (e.type == winhelp::events::eventTypes::quit) {
                display.close();
                return 0;
            }
            if (e.type == winhelp::events::eventTypes::key_up) {
                if (e.keys.size() > 0) {
                    for (winhelp::events::key k : e.keys) {
                        if (k == winhelp::events::key::Num1) {
                            selectedType = LINE;
                        } else if (k == winhelp::events::key::Num2) {
                            selectedType = CIRCLE;
                        } else if (k == winhelp::events::key::Num3) {
                            selectedType = SPRING;
                        }
                    }
                }
            } else if (e.type == winhelp::events::eventTypes::mouse_down) {
                if (e.click == winhelp::events::mouse::left) {
                    if (lineBegin) {
                        lineBegin = false;
                        engine::lines.push_back(engine::line(lineStart, lineEnd));
                    } else {
                        if (selectedType == CIRCLE) {
                            engine::circles.push_back(engine::circle(e.hit, circleSize));
                        } else if (selectedType == LINE) {
                            lineBegin = true;
                            lineStart = e.hit;
                        }
                    }
                } else if (e.click == winhelp::events::mouse::right) {
                    part closestType = CIRCLE;
                    int closestIndex = -1;
                    float closestDist = -1;
                    winhelp::vec2 mouse = e.hit;

                    for (size_t i = 0; i < engine::circles.size(); i++) {
                        float d = engine::distance(engine::circles[i].position, mouse);
                        if (closestDist != -1 && d < closestDist)  {
                            closestDist = d;
                            closestIndex = (int)i;
                        } else {
                            closestDist = d;
                        }
                    }

                    for (size_t i = 0; i < engine::lines.size(); i++) {
                        engine::line& l = engine::lines[i];
                        float d = engine::distance({((l.start.x + l.end.x) / 2), ((l.start.y + l.end.y) / 2)}, mouse);
                        if (d < closestDist) {
                            closestDist = d;
                            closestType = LINE;
                            closestIndex = (int)i;
                        }
                    }

                    if (closestIndex != -1) {
                        if (closestType == CIRCLE) {
                            engine::circles.erase(engine::circles.begin() + closestIndex);
                        } else if (closestType == LINE) {
                            engine::lines.erase(engine::lines.begin() + closestIndex);
                        }
                    }
                }
            } else if (e.type == winhelp::events::eventTypes::scroll_wheel_up and selectedType == CIRCLE) {
                circleSize = std::min((int)circleSize + (int)e.hit.y, 100);
            } else if (e.type == winhelp::events::eventTypes::scroll_wheel_down and selectedType == CIRCLE) {
                circleSize = std::max((int)circleSize - (int)e.hit.y, 1);
            }
        }
        if (lineBegin) {
            lineEnd = winhelp::mouse();
        }

        simulate();
        render(display);

        display.flip();
        winhelp::tick(60);
    }

}