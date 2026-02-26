#include "camera.h"

void Camera::handleEvent(const SDL_Event& event) {
  if (event.type == SDL_KEYDOWN) {
    switch (event.key.keysym.sym) {
      case SDLK_DOWN:  _lDown = true;     break;
      case SDLK_UP:    _lUp = true;       break;
      case SDLK_LEFT:  _lLeft = true;     break;
      case SDLK_RIGHT: _lRight = true;    break;
      case SDLK_w:     _mForward = true;  break;
      case SDLK_s:     _mBackward = true; break;
      case SDLK_a:     _mLeft = true;     break;
      case SDLK_d:     _mRight = true;    break;
    }
  } else if (event.type == SDL_KEYUP) {
    switch (event.key.keysym.sym) {
      case SDLK_DOWN:  _lDown = false;     break;
      case SDLK_UP:    _lUp = false;       break;
      case SDLK_LEFT:  _lLeft = false;     break;
      case SDLK_RIGHT: _lRight = false;    break;
      case SDLK_w:     _mForward = false;  break;
      case SDLK_s:     _mBackward = false; break;
      case SDLK_a:     _mLeft = false;     break;
      case SDLK_d:     _mRight = false;    break;
    }
  }
}

void Camera::update(float deltaTime) {
  float lookScale = deltaTime * 3.0f * _lookSpeed;
  if (_lUp)    _rotX -= lookScale;
  if (_lDown)  _rotX += lookScale;
  if (_lRight) _rotY -= lookScale;
  if (_lLeft)  _rotY += lookScale;

  matrix<4,4> rotXM = matrix<4,4>::rotationX(_rotX);
  matrix<4,4> rotYM = matrix<4,4>::rotationY(_rotY);
  matrix<4,4> rot(rotXM * rotYM);

  // Cap moveScale so a single frame can never move more than half the player width
  float moveScale = deltaTime * 10.0f * _moveSpeed;
  constexpr float hx = 0.5f, hy = 1.5f, hz = 0.5f;
  constexpr float maxStep = hx;  // never move more than half-extent per frame
  if (moveScale > maxStep) moveScale = maxStep;
  float dx = 0, dy = 0, dz = 0;
  if (_mForward) {
    dx -= moveScale * rot._m[8];
    dy -= moveScale * rot._m[9];
    dz -= moveScale * rot._m[10];
  } else if (_mBackward) {
    dx += moveScale * rot._m[8];
    dy += moveScale * rot._m[9];
    dz += moveScale * rot._m[10];
  }

  if (_mLeft) {
    dx -= moveScale * rot._m[0];
    dy -= moveScale * rot._m[1];
    dz -= moveScale * rot._m[2];
  } else if (_mRight) {
    dx += moveScale * rot._m[0];
    dy += moveScale * rot._m[1];
    dz += moveScale * rot._m[2];
  }

  // Resolve collisions per-axis to prevent tunneling and enable wall sliding
  if (_collision) {
    _x += dx;
    _collision->resolveMove(_x, _y, _z, hx, hy, hz);
    _y += dy;
    _collision->resolveMove(_x, _y, _z, hx, hy, hz);
    _z += dz;
    _collision->resolveMove(_x, _y, _z, hx, hy, hz);
  } else {
    _x += dx; _y += dy; _z += dz;
  }

  rot.set(3, 0, _x);
  rot.set(3, 1, _y);
  rot.set(3, 2, _z);

  _transform = invert(rot);
}
