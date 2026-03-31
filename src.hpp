// Implementation of the controller for ACMOJ 2284 (Problem 047)
#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP

#include "math.h"
#include "monitor.h"
#include <algorithm>
#include <cmath>

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
        angle_bias = 0.0;
        cooldown = 0;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    // Simple state to break deadlocks deterministically
    double angle_bias; // radians, small rotation added to desired direction
    int cooldown;      // steps to keep bias

    // Predict if choosing velocity v_next would cause any collision in next TIME_INTERVAL
    bool safe_with_others(const Vec &v_next) const {
        int n = monitor ? monitor->get_robot_number() : 0;
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec pj = monitor->get_pos_cur(j);
            Vec vj = monitor->get_v_cur(j);
            double rj = monitor->get_r(j);

            Vec delta_pos = pos_cur - pj;
            Vec delta_v = v_next - vj;
            double delta_r = r + rj;

            double dvn = delta_v.norm();
            if (dvn <= 1e-12) {
                // Relative velocity ~ 0: keep current distance constant over the interval
                if (delta_pos.norm_sqr() <= delta_r * delta_r - EPSILON) {
                    return false;
                }
                continue;
            }

            double project = delta_pos.dot(delta_v);
            if (project >= 0) {
                // Moving away; min distance at t=0
                if (delta_pos.norm_sqr() <= delta_r * delta_r - EPSILON) {
                    return false;
                }
                continue;
            }
            // Distance decreases first; compute min distance within [0, TIME_INTERVAL]
            double along = (-project) / dvn; // projection length along -delta_v direction
            double min_dis_sqr;
            if (along < dvn * TIME_INTERVAL) {
                min_dis_sqr = delta_pos.norm_sqr() - along * along;
            } else {
                Vec end_delta = delta_pos + delta_v * TIME_INTERVAL;
                min_dis_sqr = end_delta.norm_sqr();
            }
            if (min_dis_sqr <= delta_r * delta_r - EPSILON) {
                return false;
            }
        }
        return true;
    }

    // Build a direction with the current angle bias applied
    Vec biased_dir(const Vec &dir) const {
        // Orthonormal basis from dir
        Vec u = dir.normalize();
        Vec perp(-u.y, u.x);
        double c = std::cos(angle_bias);
        double s = std::sin(angle_bias);
        Vec w = u * c + perp * s;
        return w.normalize();
    }

public:

    Vec get_v_next() {
        // Adjust bias if last round had a collision involving this robot
        if (monitor) {
            auto collided = monitor->get_collision(id);
            if (!collided.empty()) {
                // Flip small bias to try a different path
                angle_bias = (angle_bias >= 0.0 ? -0.3 : 0.3);
                cooldown = 5;
            } else if (cooldown > 0) {
                --cooldown;
                if (cooldown == 0) angle_bias = 0.0;
            }
        }

        // If already very close to target, stop to avoid unnecessary motion
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= EPSILON) {
            return Vec();
        }

        // Desired direction with a small bias
        Vec dir = biased_dir(to_tar);

        // Upper bound speed: reach target in one interval but no more than v_max
        double hi = std::min(v_max, dist / TIME_INTERVAL);
        double lo = 0.0;

        // If even full stop is unsafe (shouldn't happen), return zero
        if (!safe_with_others(Vec())) {
            return Vec();
        }

        // Binary search maximum safe speed along dir
        Vec best = Vec();
        for (int it = 0; it < 30; ++it) {
            double mid = 0.5 * (lo + hi);
            Vec v_try = dir * mid;
            if (safe_with_others(v_try)) {
                lo = mid;
                best = v_try;
            } else {
                hi = mid;
            }
        }

        return best;
    }
};


#endif //PPCA_SRC_HPP
