#pragma once

#include "collision.h"
#include "geometry.h"
#include <SDL.h>

class Camera {
 public:
  void handleEvent(const SDL_Event& event);
  void update(float deltaTime);
  const matrix<4,4>& getTransform() const { return _transform; }

  float getX()     const { return _x; }
  float getY()     const { return _y; }
  float getZ()     const { return _z; }
  float getPitch() const { return _rotX; }
  float getYaw()   const { return _rotY; }

  void setMoveSpeed(float s) { _moveSpeed = s; }
  float getMoveSpeed() const { return _moveSpeed; }
  void setLookSpeed(float s) { _lookSpeed = s; }
  float getLookSpeed() const { return _lookSpeed; }

  void setFOV(float fov) { _fov = fov; }
  float getFOV() const { return _fov; }

  void setNearClip(float d) { _nearClip = d; }
  void setFarClip(float d) { _farClip = d; }
  float getNearClip() const { return _nearClip; }
  float getFarClip() const { return _farClip; }

  void setCollisionWorld(const CollisionWorld* cw) { _collision = cw; }

 private:
  float _rotX = 0;
  float _rotY = 0;
  float _x = 0;
  float _y = 0;
  float _z = 0;

  bool _lLeft = false;
  bool _lRight = false;
  bool _lUp = false;
  bool _lDown = false;

  bool _mForward = false;
  bool _mBackward = false;
  bool _mLeft = false;
  bool _mRight = false;

  float _moveSpeed = 1.0f;
  float _lookSpeed = 1.0f;
  float _fov = 41.0f;
  float _nearClip = 2.0f;
  float _farClip = 100.0f;
  matrix<4,4> _transform;
  const CollisionWorld* _collision = nullptr;
};
