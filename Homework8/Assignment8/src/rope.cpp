#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes) {
        // Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for (int i = 0; i < num_nodes; i++) {
            Vector2D currentPos = start + (end - start) * static_cast<double>(i) / (num_nodes - 1.0);
            Mass *newMass = new Mass(currentPos, node_mass, false);
            masses.push_back(newMass);
        }

        for (int i = 0; i < num_nodes - 1; i++) {
            Spring *newSpring = new Spring(masses[i], masses[i + 1], k);
            springs.push_back(newSpring);
        }

        for (auto &i: pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity) {
        // Calculate forces due to springs using Hooke's Law
        for (auto &s: springs) {
            Vector2D displacement = s->m2->position - s->m1->position;
            float stretchAmount = displacement.norm() - s->rest_length;
            Vector2D springForce = s->k * displacement.unit() * stretchAmount;

            s->m1->forces += springForce;
            s->m2->forces -= springForce;
        }

        // Damping constant for global damping
        const double dampingConstant = 0.01;

        // Update forces, velocities, and positions for each mass
        for (auto &m: masses) {
            if (!m->pinned) {
                // Add gravitational force
                m->forces += gravity * m->mass;

                // Add damping force
                Vector2D dampingForce = -dampingConstant * m->velocity;
                m->forces += dampingForce;

                // Compute acceleration
                Vector2D acceleration = m->forces / m->mass;

                // // Update velocity and position using implicit Euler
                // m->velocity += acceleration * delta_t;
                // m->position += m->velocity * delta_t;

                 //explicit  Euler
                 m->position += m->velocity * delta_t;
                 m->velocity += acceleration * delta_t;

            }
            // Reset forces for next iteration
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity) {
        for (auto &s: springs) {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D ab = s->m2->position - s->m1->position;
            float length = ab.norm();
            Vector2D forceDirection = ab.unit();
            Vector2D force = s->k * (length - s->rest_length) * forceDirection;

            s->m1->forces += force;
            s->m2->forces -= force;
        }
        for (auto &m: masses) {
            Vector2D accelerations = (gravity + (m->forces / m->mass));
            if (!m->pinned) {
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D temp_position = m->position;
                // TODO (Part 4): Add global Verlet damping
                double damping_factor = 0.000001;// 阻尼
                m->position += (1 - damping_factor) * (m->position - m->last_position);
                m->position += accelerations * delta_t * delta_t;
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
