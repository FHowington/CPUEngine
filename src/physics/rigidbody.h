#pragma once

#include "vec3.h"

struct AABB {
  Vec3 min, max;

  bool overlaps(const AABB& o) const {
    return min.x <= o.max.x && max.x >= o.min.x &&
           min.y <= o.max.y && max.y >= o.min.y &&
           min.z <= o.max.z && max.z >= o.min.z;
  }
};

class RigidBody {
 public:
  Vec3 position;
  Vec3 velocity;
  Vec3 force;       // accumulated force, cleared each step
  float mass = 1.0f;
  float invMass = 1.0f;  // 0 = static/infinite mass
  float restitution = 0.5f;
  Vec3 halfExtents;  // AABB half-size

  RigidBody() = default;
  RigidBody(Vec3 pos, float mass, Vec3 halfExt, float restitution = 0.5f)
      : position(pos), mass(mass),
        invMass(mass > 0 ? 1.0f / mass : 0.0f),
        restitution(restitution), halfExtents(halfExt) {}

  bool isStatic() const { return invMass == 0.0f; }

  AABB aabb() const {
    return {position - halfExtents, position + halfExtents};
  }

  void applyForce(const Vec3& f) { force += f; }

  // Semi-implicit Euler integration
  void integrate(float dt) {
    if (isStatic()) return;
    Vec3 accel = force * invMass;
    velocity += accel * dt;
    position += velocity * dt;
    force = Vec3();
  }
};
