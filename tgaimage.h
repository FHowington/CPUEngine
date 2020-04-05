#pragma once
#include <fstream>
#include "loader.h"

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
    unsigned char raw[4];
    unsigned int val;
  };
  int bytespp;

 TGAColor() : val(0), bytespp(1) {
 }

 TGAColor(unsigned char R, unsigned char G, unsigned char B, unsigned char A=255) : b(B), g(G), r(R), a(A), bytespp(4) {
 }

 TGAColor(int v, int bpp) : val(v), bytespp(bpp) {
 }

 TGAColor(const TGAColor &c) : val(c.val), bytespp(c.bytespp) {
 }

 TGAColor(const unsigned char *p, int bpp) : val(0), bytespp(bpp) {
   for (int i=0; i<bpp; i++) {
     raw[i] = p[i];
   }
 }

  TGAColor & operator =(const TGAColor &c) {
    if (this != &c) {
      bytespp = c.bytespp;
      val = c.val;
    }
    return *this;
  }
};


class TGAImage {
protected:
  unsigned char* data;
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
  const TGAColor get(int x, int y) const;
  const fcolor get_and_light(const int x, const int y, const float light) const;
  bool set(int x, int y, TGAColor c);
  const unsigned get_width() { return width; }
  const unsigned get_height() const { return height; }
};
