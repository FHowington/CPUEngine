#include <iostream>
#include <fstream>
#include <string.h>
#include <time.h>
#include <math.h>
#include "tgaimage.h"

TGAImage::TGAImage() : data(NULL), width(0), height(0), bytespp(0) {
}

TGAImage::TGAImage(int w, int h, int bpp) : data(NULL), width(w), height(h), bytespp(bpp) {
  unsigned long nbytes = width*height*bytespp;
  data = new unsigned char[nbytes];
  memset(data, 0, nbytes);
}

TGAImage::TGAImage(const TGAImage &img) {
  width = img.width;
  height = img.height;
  bytespp = img.bytespp;
  unsigned long nbytes = width*height*bytespp;
  data = new unsigned char[nbytes];
  memcpy(data, img.data, nbytes);
}

TGAImage::~TGAImage() {
  if (data) delete [] data;
}


bool TGAImage::read_tga_file(const char *filename) {
  if (data) delete [] data;
  data = NULL;
  std::ifstream in;
  in.open (filename, std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "can't open file " << filename << "\n";
    in.close();
    return false;
  }
  TGA_Header header;
  in.read((char *)&header, sizeof(header));
  if (!in.good()) {
    in.close();
    std::cerr << "an error occured while reading the header\n";
    return false;
  }
  width   = header.width;
  height  = header.height;
  bytespp = header.bitsperpixel>>3;
  if (width<=0 || height<=0 || (bytespp!=GRAYSCALE && bytespp!=RGB && bytespp!=RGBA)) {
    in.close();
    std::cerr << "bad bpp (or width/height) value\n";
    return false;
  }
  unsigned long nbytes = bytespp*width*height;
  data = new unsigned char[nbytes];
  if (3==header.datatypecode || 2==header.datatypecode) {
    in.read((char *)data, nbytes);
    if (!in.good()) {
      in.close();
      std::cerr << "an error occured while reading the data\n";
      return false;
    }
  } else if (10==header.datatypecode||11==header.datatypecode) {
    if (!load_rle_data(in)) {
      in.close();
      std::cerr << "an error occured while reading the data\n";
      return false;
    }
  } else {
    in.close();
    std::cerr << "unknown file format " << (int)header.datatypecode << "\n";
    return false;
  }
  if (!(header.imagedescriptor & 0x20)) {
    flip_vertically();
  }
  if (header.imagedescriptor & 0x10) {
    flip_horizontally();
  }
  std::cerr << width << "x" << height << "/" << bytespp*8 << "\n";
  in.close();
  return true;
}

bool TGAImage::load_rle_data(std::ifstream &in) {
  unsigned long pixelcount = width*height;
  unsigned long currentpixel = 0;
  unsigned long currentbyte  = 0;
  TGAColor colorbuffer;
  do {
    unsigned char chunkheader = 0;
    chunkheader = in.get();
    if (!in.good()) {
      std::cerr << "an error occured while reading the data\n";
      return false;
    }
    if (chunkheader<128) {
      chunkheader++;
      for (int i=0; i<chunkheader; i++) {
        in.read((char *)colorbuffer.raw, bytespp);
        if (!in.good()) {
          std::cerr << "an error occured while reading the header\n";
          return false;
        }
        for (int t=0; t<bytespp; t++)
          data[currentbyte++] = colorbuffer.raw[t];
        currentpixel++;
        if (currentpixel>pixelcount) {
          std::cerr << "Too many pixels read\n";
          return false;
        }
      }
    } else {
      chunkheader -= 127;
      in.read((char *)colorbuffer.raw, bytespp);
      if (!in.good()) {
        std::cerr << "an error occured while reading the header\n";
        return false;
      }
      for (int i=0; i<chunkheader; i++) {
        for (int t=0; t<bytespp; t++)
          data[currentbyte++] = colorbuffer.raw[t];
        currentpixel++;
        if (currentpixel>pixelcount) {
          std::cerr << "Too many pixels read\n";
          return false;
        }
      }
    }
  } while (currentpixel < pixelcount);
  return true;
}

const TGAColor TGAImage::get(int x, int y) const {
  if (!data || x<0 || y<0 || x>=width || y>=height) {
    return TGAColor();
  }
  return TGAColor(data+(x+y*width)*bytespp, bytespp);
}

const fcolor TGAImage::get_and_light(const int x, const int y, const float light) const {
  unsigned char* offset = data+(x+y*width)*bytespp;
  return fcolor(0, offset[2] * light, offset[1] * light, offset[0] * light);
}

bool TGAImage::set(int x, int y, TGAColor c) {
  if (!data || x<0 || y<0 || x>=width || y>=height) {
    return false;
  }
  memcpy(data+(x+y*width)*bytespp, c.raw, bytespp);
  return true;
}

bool TGAImage::flip_horizontally() {
  if (!data) return false;
  int half = width>>1;
  for (int i=0; i<half; i++) {
    for (int j=0; j<height; j++) {
      TGAColor c1 = get(i, j);
      TGAColor c2 = get(width-1-i, j);
      set(i, j, c2);
      set(width-1-i, j, c1);
    }
  }
  return true;
}

bool TGAImage::flip_vertically() {
  if (!data) return false;
  unsigned long bytes_per_line = width*bytespp;
  unsigned char *line = new unsigned char[bytes_per_line];
  int half = height>>1;
  for (int j=0; j<half; j++) {
    unsigned long l1 = j*bytes_per_line;
    unsigned long l2 = (height-1-j)*bytes_per_line;
    memmove((void *)line,      (void *)(data+l1), bytes_per_line);
    memmove((void *)(data+l1), (void *)(data+l2), bytes_per_line);
    memmove((void *)(data+l2), (void *)line,      bytes_per_line);
  }
  delete [] line;
  return true;
}
