#include <metal_stdlib>
using namespace metal;

// ============================================================
// Shared types (must match metal_backend.mm)
// ============================================================

struct DeviceFace {
  uint v0, v1, v2;
  int t0x, t0y, t1x, t1y, t2x, t2y;
};

struct ScreenTri {
  int sx0, sy0, sz0;
  int sx1, sy1, sz1;
  int sx2, sy2, sz2;
  float wx0, wy0, wz0;
  float wx1, wy1, wz1;
  float wx2, wy2, wz2;
  float t0x, t0y;
  float t1x, t1y;
  float t2x, t2y;
  int shaderType;
  float globalIllum;
  int textureIdx;
  int texW, texH;
  int doubleSided;
};

struct DeviceLight {
  int type;
  float strength;
  float r, g, b;
  float dx, dy, dz;
};

struct VertexUniforms {
  float model[16];
  float cam[16];
  float focalLength;
  float nearClip;
  float farClip;
  uint rW;
  uint rH;
  int shaderType;
  float globalIllum;
  int doubleSided;
  int textureIdx;
  int texW;
  int texH;
  int faceCount;
};

struct RasterUniforms {
  uint rW;
  uint rH;
  uint pitch;
  uint tilesX;
  uint tilesY;
  int numLights;
};

constant int TILE_SIZE = 32;
constant int MAX_TRIS_PER_TILE = 1024;

// ============================================================
// Helpers
// ============================================================

// Row-major mat4 * vec3 (w=1)
// result[i] = v.x*m[i] + v.y*m[4+i] + v.z*m[8+i] + m[12+i]
float3 mat4MulVec3(const constant float* m, float3 v) {
  return float3(
    v.x*m[0]  + v.y*m[4]  + v.z*m[8]  + m[12],
    v.x*m[1]  + v.y*m[5]  + v.z*m[9]  + m[13],
    v.x*m[2]  + v.y*m[6]  + v.z*m[10] + m[14]
  );
}

int3 projectVertex(float3 cam, float focalLen, uint rW, uint rH) {
  float scale = 1.0f / (-focalLen * cam.z);
  float px = cam.x * scale * float(rW) * 2.0f + float(rW) * 0.5f;
  float py = cam.y * scale * float(rH) * 2.0f + float(rH) * 0.5f;
  px = clamp(px, -16000.0f, 16000.0f);
  py = clamp(py, -16000.0f, 16000.0f);
  return int3(int(px), int(py), int(cam.z * 65535.0f));
}

float clipT(float inZ, float outZ, float nearClip) {
  return (-inZ - nearClip) / (outZ - inZ);
}

void emitTri(device ScreenTri* outTris, device atomic_int* triCount,
             int3 sa, int3 sb, int3 sc,
             float3 wa, float3 wb, float3 wc,
             int doubleSided, DeviceFace f,
             int shaderTypeVal, float globalIllum,
             int textureIdx, int texW, int texH) {
  // Backface cull
  int cross_z = (sb.x - sa.x) * (sc.y - sa.y) - (sb.y - sa.y) * (sc.x - sa.x);
  if (cross_z >= 0 && !doubleSided) return;
  if (cross_z >= 0) {
    int3 tmp = sb; sb = sc; sc = tmp;
    float3 wtmp = wb; wb = wc; wc = wtmp;
  }

  int slot = atomic_fetch_add_explicit(triCount, 1, memory_order_relaxed);
  ScreenTri tri;
  tri.sx0 = sa.x; tri.sy0 = sa.y; tri.sz0 = sa.z;
  tri.sx1 = sb.x; tri.sy1 = sb.y; tri.sz1 = sb.z;
  tri.sx2 = sc.x; tri.sy2 = sc.y; tri.sz2 = sc.z;
  tri.wx0 = wa.x; tri.wy0 = wa.y; tri.wz0 = wa.z;
  tri.wx1 = wb.x; tri.wy1 = wb.y; tri.wz1 = wb.z;
  tri.wx2 = wc.x; tri.wy2 = wc.y; tri.wz2 = wc.z;
  tri.t0x = float(f.t0x); tri.t0y = float(f.t0y);
  tri.t1x = float(f.t1x); tri.t1y = float(f.t1y);
  tri.t2x = float(f.t2x); tri.t2y = float(f.t2y);
  tri.shaderType = shaderTypeVal;
  tri.globalIllum = globalIllum;
  tri.textureIdx = textureIdx;
  tri.texW = texW; tri.texH = texH;
  tri.doubleSided = doubleSided;
  outTris[slot] = tri;
}

// ============================================================
// Kernel 1: Clear framebuffer
// ============================================================
kernel void clearBuffers(device uint* pixels [[buffer(0)]],
                         device int* zbuff [[buffer(1)]],
                         constant uint& rW [[buffer(2)]],
                         constant uint& rH [[buffer(3)]],
                         constant uint& pitch [[buffer(4)]],
                         uint2 gid [[thread_position_in_grid]]) {
  if (gid.x >= rW || gid.y >= rH) return;
  uint idx = gid.y * pitch + gid.x;
  pixels[idx] = 0;
  zbuff[idx] = INT_MIN;
}

// ============================================================
// Kernel 2: Vertex transform + clip + project
// ============================================================
kernel void vertexTransform(device const float* verts [[buffer(0)]],
                            device const DeviceFace* faces [[buffer(1)]],
                            constant VertexUniforms& u [[buffer(2)]],
                            device ScreenTri* outTris [[buffer(3)]],
                            device atomic_int* triCount [[buffer(4)]],
                            uint gid [[thread_position_in_grid]]) {
  if (int(gid) >= u.faceCount) return;

  DeviceFace f = faces[gid];
  float3 v0 = float3(verts[f.v0*4], verts[f.v0*4+1], verts[f.v0*4+2]);
  float3 v1 = float3(verts[f.v1*4], verts[f.v1*4+1], verts[f.v1*4+2]);
  float3 v2 = float3(verts[f.v2*4], verts[f.v2*4+1], verts[f.v2*4+2]);

  float3 w0 = mat4MulVec3(u.model, v0);
  float3 w1 = mat4MulVec3(u.model, v1);
  float3 w2 = mat4MulVec3(u.model, v2);

  float3 c0 = mat4MulVec3(u.cam, w0);
  float3 c1 = mat4MulVec3(u.cam, w1);
  float3 c2 = mat4MulVec3(u.cam, w2);

  int b0 = (c0.z > -u.nearClip) ? 1 : 0;
  int b1 = (c1.z > -u.nearClip) ? 1 : 0;
  int b2 = (c2.z > -u.nearClip) ? 1 : 0;
  int numBehind = b0 + b1 + b2;
  if (numBehind == 3) return;

  int f0 = (c0.z < -u.farClip) ? 1 : 0;
  int f1 = (c1.z < -u.farClip) ? 1 : 0;
  int f2 = (c2.z < -u.farClip) ? 1 : 0;
  if (f0 + f1 + f2 == 3) return;

  if (numBehind == 0) {
    int3 s0 = projectVertex(c0, u.focalLength, u.rW, u.rH);
    int3 s1 = projectVertex(c1, u.focalLength, u.rW, u.rH);
    int3 s2 = projectVertex(c2, u.focalLength, u.rW, u.rH);
    emitTri(outTris, triCount, s0, s1, s2, w0, w1, w2,
            u.doubleSided, f, u.shaderType, u.globalIllum,
            u.textureIdx, u.texW, u.texH);
  } else if (numBehind == 1) {
    float3 ca, cb, cc, wa, wb, wc;
    if (b0) { ca=c0; cb=c1; cc=c2; wa=w0; wb=w1; wc=w2; }
    else if (b1) { ca=c1; cb=c2; cc=c0; wa=w1; wb=w2; wc=w0; }
    else { ca=c2; cb=c0; cc=c1; wa=w2; wb=w0; wc=w1; }

    float t1 = clipT(cb.z, ca.z, u.nearClip);
    float t2 = clipT(cc.z, ca.z, u.nearClip);
    float3 na1 = mix(cb, ca, t1);
    float3 na2 = mix(cc, ca, t2);
    float3 wna1 = mix(wb, wa, t1);
    float3 wna2 = mix(wc, wa, t2);

    int3 sna1 = projectVertex(na1, u.focalLength, u.rW, u.rH);
    int3 sb = projectVertex(cb, u.focalLength, u.rW, u.rH);
    int3 sc = projectVertex(cc, u.focalLength, u.rW, u.rH);
    int3 sna2 = projectVertex(na2, u.focalLength, u.rW, u.rH);

    emitTri(outTris, triCount, sna1, sb, sc, wna1, wb, wc,
            u.doubleSided, f, u.shaderType, u.globalIllum,
            u.textureIdx, u.texW, u.texH);
    emitTri(outTris, triCount, sna1, sc, sna2, wna1, wc, wna2,
            u.doubleSided, f, u.shaderType, u.globalIllum,
            u.textureIdx, u.texW, u.texH);
  } else { // numBehind == 2
    float3 ca, cb, cc, wa, wb, wc;
    if (!b0) { ca=c0; cb=c1; cc=c2; wa=w0; wb=w1; wc=w2; }
    else if (!b1) { ca=c1; cb=c2; cc=c0; wa=w1; wb=w2; wc=w0; }
    else { ca=c2; cb=c0; cc=c1; wa=w2; wb=w0; wc=w1; }

    float t1 = clipT(ca.z, cb.z, u.nearClip);
    float t2 = clipT(ca.z, cc.z, u.nearClip);
    float3 nb = mix(ca, cb, t1);
    float3 nc = mix(ca, cc, t2);
    float3 wnb = mix(wa, wb, t1);
    float3 wnc = mix(wa, wc, t2);

    int3 sa = projectVertex(ca, u.focalLength, u.rW, u.rH);
    int3 snb = projectVertex(nb, u.focalLength, u.rW, u.rH);
    int3 snc = projectVertex(nc, u.focalLength, u.rW, u.rH);

    emitTri(outTris, triCount, sa, snb, snc, wa, wnb, wnc,
            u.doubleSided, f, u.shaderType, u.globalIllum,
            u.textureIdx, u.texW, u.texH);
  }
}

// ============================================================
// Kernel 3: Bin triangles into tiles
// ============================================================
kernel void binTriangles(device const ScreenTri* tris [[buffer(0)]],
                         device atomic_int* tileCounts [[buffer(1)]],
                         device int* tileTris [[buffer(2)]],
                         constant uint& tilesX [[buffer(3)]],
                         constant uint& tilesY [[buffer(4)]],
                         constant int& numTris [[buffer(5)]],
                         uint gid [[thread_position_in_grid]]) {
  if (int(gid) >= numTris) return;

  ScreenTri t = tris[gid];
  int minX = min(t.sx0, min(t.sx1, t.sx2));
  int maxX = max(t.sx0, max(t.sx1, t.sx2));
  int minY = min(t.sy0, min(t.sy1, t.sy2));
  int maxY = max(t.sy0, max(t.sy1, t.sy2));

  int tMinX = max(0, minX / TILE_SIZE);
  int tMaxX = min(int(tilesX) - 1, maxX / TILE_SIZE);
  int tMinY = max(0, minY / TILE_SIZE);
  int tMaxY = min(int(tilesY) - 1, maxY / TILE_SIZE);

  for (int ty = tMinY; ty <= tMaxY; ++ty) {
    for (int tx = tMinX; tx <= tMaxX; ++tx) {
      int tileIdx = ty * int(tilesX) + tx;
      int slot = atomic_fetch_add_explicit(&tileCounts[tileIdx], 1, memory_order_relaxed);
      if (slot < MAX_TRIS_PER_TILE) {
        tileTris[tileIdx * MAX_TRIS_PER_TILE + slot] = int(gid);
      }
    }
  }
}

// ============================================================
// Kernel 4: Rasterize tiles
// ============================================================

int orient2d(int2 a, int2 b, int2 p) {
  return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

bool isTopLeft(int2 a, int2 b) {
  return (a.y == b.y && b.x < a.x) || (b.y < a.y);
}

kernel void rasterizeTiles(device const ScreenTri* tris [[buffer(0)]],
                           device const int* tileCounts [[buffer(1)]],
                           device const int* tileTris [[buffer(2)]],
                           device uint* pixels [[buffer(3)]],
                           device int* zbuff [[buffer(4)]],
                           device const DeviceLight* lights [[buffer(5)]],
                           constant RasterUniforms& u [[buffer(6)]],
                           uint2 gid [[thread_position_in_grid]]) {
  int px = int(gid.x);
  int py = int(gid.y);
  if (px >= int(u.rW) || py >= int(u.rH)) return;

  int tileX = px / TILE_SIZE;
  int tileY = py / TILE_SIZE;
  int tileIdx = tileY * int(u.tilesX) + tileX;

  uint fbIdx = uint(py) * u.pitch + uint(px);
  uint bestColor = pixels[fbIdx];
  int bestZ = zbuff[fbIdx];

  int count = tileCounts[tileIdx];
  if (count > MAX_TRIS_PER_TILE) count = MAX_TRIS_PER_TILE;

  for (int i = 0; i < count; ++i) {
    int triIdx = tileTris[tileIdx * MAX_TRIS_PER_TILE + i];
    ScreenTri t = tris[triIdx];

    int2 sv0 = int2(t.sx0, t.sy0);
    int2 sv1 = int2(t.sx1, t.sy1);
    int2 sv2 = int2(t.sx2, t.sy2);
    int2 p = int2(px, py);

    int e0 = orient2d(sv1, sv2, p);
    int e1 = orient2d(sv2, sv0, p);
    int e2 = orient2d(sv0, sv1, p);

    int bias0 = isTopLeft(sv1, sv2) ? 0 : -1;
    int bias1 = isTopLeft(sv2, sv0) ? 0 : -1;
    int bias2 = isTopLeft(sv0, sv1) ? 0 : -1;

    if ((e0 + bias0) >= 0 && (e1 + bias1) >= 0 && (e2 + bias2) >= 0) {
      float wTotal = float(e0 + e1 + e2);
      if (wTotal == 0.0f) continue;
      float bary0 = float(e0) / wTotal;
      float bary1 = float(e1) / wTotal;
      float bary2 = float(e2) / wTotal;

      int z = int(bary0 * float(t.sz0) + bary1 * float(t.sz1) + bary2 * float(t.sz2));
      if (z > bestZ) {
        float3 wp = float3(bary0*t.wx0 + bary1*t.wx1 + bary2*t.wx2,
                           bary0*t.wy0 + bary1*t.wy1 + bary2*t.wy2,
                           bary0*t.wz0 + bary1*t.wz1 + bary2*t.wz2);

        float3 edge1 = float3(t.wx1-t.wx0, t.wy1-t.wy0, t.wz1-t.wz0);
        float3 edge2 = float3(t.wx2-t.wx0, t.wy2-t.wy0, t.wz2-t.wz0);
        float3 n = normalize(cross(edge1, edge2));

        float r = t.globalIllum, g = t.globalIllum, bl = t.globalIllum;
        for (int li = 0; li < u.numLights; ++li) {
          DeviceLight l = lights[li];
          float3 ldir;
          if (l.type == 0) {
            ldir = float3(l.dx, l.dy, l.dz);
          } else {
            ldir = float3(l.dx, l.dy, l.dz) - wp;
            float len = length(ldir);
            if (len < 0.001f) continue;
            ldir /= len;
          }
          float d = max(0.0f, dot(n, ldir));
          float intensity = (l.type == 0) ? d : d * l.strength;
          r += intensity * l.r;
          g += intensity * l.g;
          bl += intensity * l.b;
        }

        int ri = min(255, int(r * 255.0f));
        int gi = min(255, int(g * 255.0f));
        int bi = min(255, int(bl * 255.0f));
        bestColor = uint(ri << 16) | uint(gi << 8) | uint(bi);
        bestZ = z;
      }
    }
  }

  pixels[fbIdx] = bestColor;
  zbuff[fbIdx] = bestZ;
}
