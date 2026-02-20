#include "demo_game.h"
#include "engine.h"

int main() {
  Engine engine;
  DemoGame game;
  engine.run(game);
  return 0;
}
