#pragma once

#include "rigidbody.h"
#include <vector>
#include <cmath>

// Minimal physics world: gravity, AABB collision detection, impulse response.
class PhysicsWorld {
 public:
  Vec3 gravity{0, -9.81f, 0};

  void addBody(RigidBody* body) { _bodies.push_back(body); }

  void step(float dt) {
    // Apply gravity
    for (auto* b : _bodies)
      if (!b->isStatic())
        b->applyForce(gravity * b->mass);

    // Integrate
    for (auto* b : _bodies)
      b->integrate(dt);

    // Detect and resolve collisions
    for (size_t i = 0; i < _bodies.size(); ++i) {
      for (size_t j = i + 1; j < _bodies.size(); ++j) {
        resolveAABB(*_bodies[i], *_bodies[j]);
      }
    }
  }

 private:
  std::vector<RigidBody*> _bodies;

  static void resolveAABB(RigidBody& a, RigidBody& b) {
    if (a.isStatic() && b.isStatic()) return;

    AABB ba = a.aabb(), bb = b.aabb();
    if (!ba.overlaps(bb)) return;

    // Find minimum penetration axis
    float overlapX = std::min(ba.max.x - bb.min.x, bb.max.x - ba.min.x);
    float overlapY = std::min(ba.max.y - bb.min.y, bb.max.y - ba.min.y);
    float overlapZ = std::min(ba.max.z - bb.min.z, bb.max.z - ba.min.z);

    Vec3 normal;
    float pen;
    if (overlapX <= overlapY && overlapX <= overlapZ) {
      pen = overlapX;
      normal = {(a.position.x < b.position.x) ? -1.0f : 1.0f, 0, 0};
    } else if (overlapY <= overlapZ) {
      pen = overlapY;
      normal = {0, (a.position.y < b.position.y) ? -1.0f : 1.0f, 0};
    } else {
      pen = overlapZ;
      normal = {0, 0, (a.position.z < b.position.z) ? -1.0f : 1.0f};
    }

    // Positional correction (push apart)
    float totalInv = a.invMass + b.invMass;
    if (totalInv > 0) {
      Vec3 correction = normal * (pen / totalInv);
      a.position += correction * a.invMass;
      b.position -= correction * b.invMass;
    }

    // Impulse-based velocity response
    float relVel = (a.velocity - b.velocity).dot(normal);
    if (relVel > 0) return;  // separating

    float e = std::min(a.restitution, b.restitution);
    float j = -(1.0f + e) * relVel / totalInv;

    Vec3 impulse = normal * j;
    a.velocity += impulse * a.invMass;
    b.velocity -= impulse * b.invMass;
  }
};
