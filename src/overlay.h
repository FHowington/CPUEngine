#pragma once

#include "Window.h"
#include <array>
#include <string>

// Draws 2D UI elements directly into the pixel framebuffer after 3D rendering.
//
// Coordinate system: (0,0) is the SDL top-left corner. Row y is pixels[y*W + x].
// This is independent of the 3D renderer's vertical flip; the overlay always
// appears right-side up in the SDL window.
//
// Font: embedded 8x8 bitmap glyphs for ASCII 0x20-0x7E.
// Text is drawn pixel-perfect with no scaling; at the default W*4 SDL window
// size each glyph is 32x32 pixels on the display.

class Overlay {
 public:
  // Draw a semi-transparent filled rectangle.
  // alpha: 0 = fully transparent, 255 = fully opaque solid bgColor.
  static void fillRect(int x, int y, int w, int h, unsigned bgColor, unsigned char alpha);

  // Draw a 1-pixel border rectangle (outline only).
  static void drawRect(int x, int y, int w, int h, unsigned color);

  // Draw a string of ASCII text. Each glyph is 8x8. No newline handling.
  static void drawText(int x, int y, const std::string& text, unsigned color = 0xFFFFFF);

 private:
  static void drawGlyph(int px, int py, unsigned char c, unsigned color);
  static const unsigned char font[96][8];  // ASCII 0x20-0x7E
};
