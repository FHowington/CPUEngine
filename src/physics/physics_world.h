#pragma once

#include "rigidbody.h"
#include <algorithm>
#include <cmath>
#include <vector>

// Physics world with SIMD integration and sort-and-sweep broad phase.
class PhysicsWorld {
 public:
  Vec3 gravity{0, -9.81f, 0};

  void addBody(RigidBody* body) { _bodies.push_back(body); }

  void step(float dt) {
    const unsigned n = (unsigned)_bodies.size();
    if (n == 0) return;

    // ── Apply gravity + integrate ────────────────────────────────────────
#if defined(__AVX2__) && defined(__FMA__)
    syncToSoA(n);
    applyGravityAVX(_fy.data(), _mass.data(), n, gravity.y);
    integrateAVX(_px.data(), _py.data(), _pz.data(),
                 _vx.data(), _vy.data(), _vz.data(),
                 _fx.data(), _fy.data(), _fz.data(),
                 _invMass.data(), n, dt);
    syncFromSoA(n);
#else
    for (auto* b : _bodies)
      if (!b->isStatic())
        b->applyForce(gravity * b->mass);
    for (auto* b : _bodies)
      b->integrate(dt);
#endif

    // ── Broad phase: sort-and-sweep on X axis ────────────────────────────
    buildSortedList(n);
    _pairs.clear();
    sweepAndPrune();

    // ── Narrow phase + resolve ───────────────────────────────────────────
    for (auto& [i, j] : _pairs)
      resolveAABB(*_bodies[i], *_bodies[j]);
  }

 private:
  std::vector<RigidBody*> _bodies;

  // SoA arrays for SIMD integration
  std::vector<float> _px, _py, _pz;
  std::vector<float> _vx, _vy, _vz;
  std::vector<float> _fx, _fy, _fz;
  std::vector<float> _mass, _invMass;

  // Sort-and-sweep
  struct SAPEntry {
    float minX, maxX;
    unsigned idx;
  };
  std::vector<SAPEntry> _sorted;
  std::vector<std::pair<unsigned, unsigned>> _pairs;

  void syncToSoA(unsigned n) {
    auto resize = [&](std::vector<float>& v) { v.resize(n + 8, 0); };
    resize(_px); resize(_py); resize(_pz);
    resize(_vx); resize(_vy); resize(_vz);
    resize(_fx); resize(_fy); resize(_fz);
    resize(_mass); resize(_invMass);
    for (unsigned i = 0; i < n; ++i) {
      auto* b = _bodies[i];
      _px[i] = b->position.x; _py[i] = b->position.y; _pz[i] = b->position.z;
      _vx[i] = b->velocity.x; _vy[i] = b->velocity.y; _vz[i] = b->velocity.z;
      _fx[i] = b->force.x;    _fy[i] = b->force.y;    _fz[i] = b->force.z;
      _mass[i] = b->mass;      _invMass[i] = b->invMass;
    }
  }

  void syncFromSoA(unsigned n) {
    for (unsigned i = 0; i < n; ++i) {
      auto* b = _bodies[i];
      b->position = {_px[i], _py[i], _pz[i]};
      b->velocity = {_vx[i], _vy[i], _vz[i]};
      b->force = Vec3();
    }
  }

  void buildSortedList(unsigned n) {
    _sorted.resize(n);
    for (unsigned i = 0; i < n; ++i) {
      AABB box = _bodies[i]->aabb();
      _sorted[i] = {box.min.x, box.max.x, i};
    }
    std::sort(_sorted.begin(), _sorted.end(),
              [](const SAPEntry& a, const SAPEntry& b) { return a.minX < b.minX; });
  }

  void sweepAndPrune() {
    const unsigned n = (unsigned)_sorted.size();
    for (unsigned i = 0; i < n; ++i) {
      float maxX_i = _sorted[i].maxX;
      for (unsigned j = i + 1; j < n; ++j) {
        if (_sorted[j].minX > maxX_i) break;  // no more overlaps on X
        // X overlaps — check full AABB
        unsigned ai = _sorted[i].idx, bi = _sorted[j].idx;
        if (_bodies[ai]->isStatic() && _bodies[bi]->isStatic()) continue;
        AABB a = _bodies[ai]->aabb(), b = _bodies[bi]->aabb();
        if (a.overlaps(b))
          _pairs.push_back({ai, bi});
      }
    }
  }

  static void resolveAABB(RigidBody& a, RigidBody& b) {
    AABB ba = a.aabb(), bb = b.aabb();

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

    float totalInv = a.invMass + b.invMass;
    if (totalInv <= 0) return;

    // Positional correction
    Vec3 correction = normal * (pen / totalInv);
    a.position += correction * a.invMass;
    b.position -= correction * b.invMass;

    // Impulse response
    float relVel = (a.velocity - b.velocity).dot(normal);
    if (relVel > 0) return;

    float e = std::min(a.restitution, b.restitution);
    float j = -(1.0f + e) * relVel / totalInv;

    Vec3 impulse = normal * j;
    a.velocity += impulse * a.invMass;
    b.velocity -= impulse * b.invMass;
  }
};
