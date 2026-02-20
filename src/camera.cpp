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
  if (_lUp)    _rotX -= 0.03f * _sensitivity;
  if (_lDown)  _rotX += 0.03f * _sensitivity;
  if (_lRight) _rotY -= 0.03f * _sensitivity;
  if (_lLeft)  _rotY += 0.03f * _sensitivity;

  matrix<4,4> rotXM = matrix<4,4>::rotationX(_rotX);
  matrix<4,4> rotYM = matrix<4,4>::rotationY(_rotY);
  matrix<4,4> rot(rotXM * rotYM);

  float moveScale = deltaTime * 10.0f * _sensitivity;  // Base speed = 10.0
  if (_mForward) {
    _x -= moveScale * rot._m[8];
    _y -= moveScale * rot._m[9];
    _z -= moveScale * rot._m[10];
  } else if (_mBackward) {
    _x += moveScale * rot._m[8];
    _y += moveScale * rot._m[9];
    _z += moveScale * rot._m[10];
  }

  if (_mLeft) {
    _x -= moveScale * rot._m[0];
    _y -= moveScale * rot._m[1];
    _z -= moveScale * rot._m[2];
  } else if (_mRight) {
    _x += moveScale * rot._m[0];
    _y += moveScale * rot._m[1];
    _z += moveScale * rot._m[2];
  }

  rot.set(3, 0, _x);
  rot.set(3, 1, _y);
  rot.set(3, 2, _z);

  _transform = invert(rot);
}
