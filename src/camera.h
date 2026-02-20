#pragma once

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

  void setSensitivity(float s) { _sensitivity = s; }
  float getSensitivity() const { return _sensitivity; }

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

  float _sensitivity = 1.0f;
  matrix<4,4> _transform;
};
