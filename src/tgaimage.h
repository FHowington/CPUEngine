#pragma once
#include "loader.h"
#include <fstream>

#pragma pack(push,1)
struct TGA_Header {
  char idlength;
  char colormaptype;
  char datatypecode;
  short colormaporigin;
  short colormaplength;
  char colormapdepth;
  short x_origin;
  short y_origin;
  short width;
  short height;
  char  bitsperpixel;
  char  imagedescriptor;
};

#pragma pack(pop)

struct TGAColor {
  union {
    struct {
      unsigned char b, g, r, a;
    };
    char raw[4]; // NOLINT
    unsigned int val;
  };
  int bytespp{1};

 TGAColor() : val(0) {
 }

 TGAColor(unsigned char R, unsigned char G, unsigned char B, unsigned char A=255) : b(B), g(G), r(R), a(A), bytespp(4) {
 }

 TGAColor(int v, int bpp) : val(v), bytespp(bpp) {
 }

 TGAColor(const TGAColor &c) : val(c.val), bytespp(c.bytespp) {
 }

 TGAColor(const char *p, int bpp) : val(0), bytespp(bpp) {
   for (int i=0; i<bpp; i++) {
     raw[i] = (uint8_t)p[i];
   }
 }

  TGAColor(TGAColor&& rhs) = default;
  TGAColor& operator=(TGAColor&& c) = default;

  TGAColor& operator=(const TGAColor &c) = default;
  ~TGAColor() = default;
};


class TGAImage { // NOLINT
public:
  char* data;
  int width;
  int height;
  int bytespp;

  bool   load_rle_data(std::ifstream &in);

public:
  enum Format {
    GRAYSCALE=1, RGB=3, RGBA=4
  };

  TGAImage();
  ~TGAImage();
  TGAImage(int w, int h, int bpp);
  TGAImage(const TGAImage &img);
  bool flip_horizontally();
  bool flip_vertically();
  bool read_tga_file(const char *filename);
  [[nodiscard]] TGAColor get(int x, int y) const;
  [[nodiscard]] fcolor get_and_light(int x, int y, float light) const;
  [[nodiscard]] fcolor get_and_light(int index, float light) const;
  [[nodiscard]] fcolor get(int index) const;
  [[nodiscard]] unsigned fast_get(int x, int y) const;
  bool set(int x, int y, TGAColor c);
  [[nodiscard]] unsigned get_width() const { return width; }
  [[nodiscard]] unsigned get_height() const { return height; }
};
