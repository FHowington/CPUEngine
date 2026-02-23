#pragma once

// Entity — base class for anything that updates over time in the scene.
class Entity {
 public:
  virtual ~Entity() = default;
  virtual void update(float dt) = 0;
};
