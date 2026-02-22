#pragma once
#include "E:\vs code\files\winhelp\src\winhelp.hpp"
#include <cmath>
#include <vector>
using point = winhelp::vec2;
using colour = winhelp::vec3;

namespace engine {
    struct Config {
        int W;
        int H;
        float friction;
        float gravity;
        float restitution;
        point center;
        colour circleColour;
        colour lineColour;
        int lineThickness;

        Config(int width, int height, float fric, float grav, float rest, colour circleCol, colour lineCol, int lineThickness)
            : W(width), H(height), friction(fric), gravity(grav), restitution(rest), circleColour(circleCol), lineColour(lineCol), lineThickness(lineThickness) {
                center = point{ W / 2.0f, H / 2.0f };
            }
        
    };

    struct line {
        point start;
        point end;

        line(point s, point e) : start(s), end(e) {}
    };

    struct circle;

    Config config(800, 600, 0.98f, 9.81f, 0.8f, {255, 255, 255}, {255, 255, 255}, 2);

    double toRad(double degrees) {
        return degrees * M_PI / 180.0;
    }

    point rotatePoint(const point& p, float angle, point pivot=config.center) {
        float s = std::sin(toRad(angle));
        float c = std::cos(toRad(angle));
        return point{
            (p.x - pivot.x) * c - (p.y - pivot.y) * s + pivot.x,
            (p.x - pivot.x) * s + (p.y - pivot.y) * c + pivot.y
        };
    }

    std::vector<line> lines;
    std::vector<circle> circles;


    double dot(const point& a, const point& b) {
        return a.x * b.x + a.y * b.y;
    }

    double distance(const point& a, const point& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    point collisionNormalFromLine(const line& lineSeg, point closest, point ciclePos) {
        double dx = lineSeg.end.x - lineSeg.start.x;
        double dy = lineSeg.end.y - lineSeg.start.y;

        double nx = -dy, ny = dx;
        double length = std::pow((nx*nx + ny*ny), 0.5);
        if (length == 0) {
            return point{0, 0};
        }

        nx /= length;
        ny /= length;

        // Force normal to point the line -> circle
        double to_circle_x = ciclePos.x - closest.x;
        double to_circle_y = ciclePos.y - closest.y;

        if (nx * to_circle_x + ny * to_circle_y < 0) {
            nx = -nx;
            ny = -ny;
        }

        return point{nx, ny};
    }

    struct circle {
        point position;
        int radius;
        point velocity;

        circle(point pos={0, 0}, int r=15, point vel={0, 0}) {
            this->position = pos;
            this->radius = r;
            this->velocity = vel;
        }

        void collideWithLine(const line& l) {
            double dx = l.end.x - l.start.x;
            double dy = l.end.y - l.start.y;
            double seg_len_sq = dx * dx + dy * dy;
            if (seg_len_sq == 0) {
                return;
            }

            double t = ((position.x - l.start.x)*dx + (position.y - l.start.y)*dy)/seg_len_sq;
            t = std::fmax(0, std::fmin(1, t));

            point closest = point(
                l.start.x + dx * t,
                l.start.y + dy * t
            );

            double dist = distance(position, closest);
            if (dist >= radius) {
                return;
            }

            point normal = collisionNormalFromLine(
                l,
                closest,
                position
            );

            double overlap = radius - dist;
            position.x += normal.x * overlap;
            position.y += normal.y * overlap;

            double vel = dot(velocity, normal);
            if (vel >= 0) {
                return;
            } 

            point tangent = point{-normal.y, normal.x};
            double vel_tangent = dot(velocity, tangent);

            velocity.x = (-vel * config.restitution) * normal.x + vel_tangent * (1 - config.friction) * tangent.x; 
            velocity.y = (-vel * config.restitution) * normal.y + vel_tangent * (1 - config.friction) * tangent.y; 

        }

        void collideWithCircle(circle& other) {
            double dx = position.x - other.position.x;
            double dy = position.y - other.position.y;
            double distSq = dx * dx + dy * dy;
            double min_dist = radius + other.radius;
            if (distSq == 0 or distSq >= min_dist * min_dist)
                return;

            double dist = std::sqrt(distSq);
            double nx = dx / dist, ny = dy / dist;

            double overlap = min_dist - dist;
            double correction = overlap * 0.5;

            position.x += nx * correction;
            position.y += ny * correction;

            other.position.x -= nx * correction;
            other.position.y -= ny * correction;

            double rvx = velocity.x - other.velocity.x;
            double rvy = velocity.y - other.velocity.y;
            double velAlongNormal = rvx * nx + rvy * ny;

            if (velAlongNormal > 0) {
                return;
            }

            double j = -(1 + config.restitution) * velAlongNormal * 0.5;

            winhelp::vec2 impulse = winhelp::vec2(j * nx, j * ny);

            velocity += impulse;
            other.velocity -= impulse;
        }

        void simulate(float dt) {

            velocity.y += config.gravity * dt;
            position += velocity * dt;

            for (const line& l : lines) {
                collideWithLine(l);
            }

            for (circle& c : circles) {
                if (&c != this) {
                    collideWithCircle(c);
                }
            }
        }
    };

    struct Spring {

    };

    void simulate(float dt) {
        for (circle& c : circles) {
            c.simulate(dt);
        }
    }
}
