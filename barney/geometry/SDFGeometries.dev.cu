// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "barney/geometry/SDFGeometries.h"
#include "rtcore/TraceInterface.h"

RTC_DECLARE_GLOBALS(BARNEY_NS::render::OptixGlobals);

namespace BARNEY_NS {
  using namespace BARNEY_NS::render;

#if RTC_DEVICE_CODE
  inline __rtc_device float sdf_sqrt(float f) { return sqrtf(f); }
  inline __rtc_device float sdf_sign(float f) { return (f > 0.f) - (f < 0.f); }
  inline __rtc_device float sdf_length(vec2f v) { return sqrtf(dot(v, v)); }
  inline __rtc_device float sdf_fract(float f) { return f - floorf(f); }
  inline __rtc_device vec3f sdf_fract(vec3f v)
  {
    return vec3f(sdf_fract(v.x), sdf_fract(v.y), sdf_fract(v.z));
  }
  inline __rtc_device vec3f sdf_floor(vec3f v)
  {
    return vec3f(floorf(v.x), floorf(v.y), floorf(v.z));
  }
#endif

  struct SDFGeometriesPrograms {
#if RTC_DEVICE_CODE
    static inline __rtc_device float sdfSphere(const vec3f &p,
                                             const SDFPrimitive &g)
    {
      return length(p - g.p0) - g.r0;
    }

    static inline __rtc_device float sdfPill(const vec3f &p,
                                           const SDFPrimitive &g)
    {
      const vec3f ba = g.p1 - g.p0;
      const float h =
        clamp(dot(p - g.p0, ba) / dot(ba, ba), 0.f, 1.f);
      return length(p - g.p0 - h * ba) - g.r0;
    }

    static inline __rtc_device float sdfConePill(const vec3f &p,
                                               const SDFPrimitive &g)
    {
      const vec3f ba = g.p1 - g.p0;
      const float l2 = dot(ba, ba);
      const float rr = g.r0 - g.r1;
      const float a2 = l2 - rr * rr;
      const float il2 = 1.f / l2;
      const vec3f pa = p - g.p0;
      const float y = dot(pa, ba);
      const float z = y - l2;
      const vec3f xv = pa * l2 - ba * y;
      const float x2 = dot(xv, xv);
      const float y2 = y * y * l2;
      const float z2 = z * z * l2;
      const float k = sdf_sign(rr) * rr * rr * x2;
      if (sdf_sign(z) * a2 * z2 > k)
        return sdf_sqrt(x2 + z2) * il2 - g.r1;
      if (sdf_sign(y) * a2 * y2 < k)
        return sdf_sqrt(x2 + y2) * il2 - g.r0;
      return (sdf_sqrt(x2 * a2 * il2) + y * rr) * il2 - g.r0;
    }

    static inline __rtc_device float sdfCappedCone(const vec3f &p,
                                                 const SDFPrimitive &g)
    {
      const vec3f bav = g.p1 - g.p0;
      const float rba = g.r1 - g.r0;
      const float baba = dot(bav, bav);
      const float papa = dot(p - g.p0, p - g.p0);
      const float paba = dot(p - g.p0, bav) / baba;
      const float x = sdf_sqrt(max(0.f, papa - paba * paba * baba));
      const float cax = max(0.f, x - ((paba < 0.5f) ? g.r0 : g.r1));
      const float cay = fabs(paba - 0.5f) - 0.5f;
      const float k = rba * rba + baba;
      const float f = clamp((rba * (x - g.r0) + paba * baba) / k, 0.f, 1.f);
      const float cbx = x - g.r0 - f * rba;
      const float cby = paba - f;
      const float s = (cbx < 0.f && cay < 0.f) ? -1.f : 1.f;
      return s * sdf_sqrt(min(cax * cax + cay * cay * baba,
                               cbx * cbx + cby * cby * baba));
    }

    static inline __rtc_device float sdfTorus(const vec3f &p,
                                            const SDFPrimitive &g)
    {
      const vec3f c = g.p0;
      const vec2f q =
        vec2f(sdf_length(vec2f(p.x - c.x, p.z - c.z)) - g.r0, p.y - c.y);
      return sdf_length(q) - g.r1;
    }

    static inline __rtc_device float sdfCutSphere(const vec3f &p,
                                                const SDFPrimitive &g)
    {
      const vec3f c = g.p0;
      const float h = g.r1;
      const float w = sdf_sqrt(max(0.f, g.r0 * g.r0 - h * h));
      const vec2f q =
        vec2f(sdf_length(vec2f(p.x - c.x, p.z - c.z)), p.y - c.y);
      const float s =
        max((h - g.r0) * q.x * q.x + w * w * (h + g.r0 - 2.f * q.y),
            h * q.x - w * q.y);
      return (s < 0.f) ? sdf_length(q) - g.r0
                       : (q.x < w) ? h - q.y : sdf_length(q - vec2f(w, h));
    }

    static inline __rtc_device float sdfVesica(const vec3f &p,
                                             const SDFPrimitive &g)
    {
      const vec3f mid = 0.5f * (g.p0 + g.p1);
      const float l = length(g.p1 - g.p0);
      const vec3f v = (g.p1 - g.p0) / l;
      const float y = dot(p - mid, v);
      const vec2f q = vec2f(length(p - mid - y * v), fabs(y));
      const float r = 0.5f * l;
      const float d = 0.5f * (r * r - g.r0 * g.r0) / g.r0;
      const vec3f h =
        (r * q.x < d * (q.y - r)) ? vec3f(0.f, r, 0.f)
                                 : vec3f(-d, 0.f, d + g.r0);
      return sdf_length(q - vec2f(h.x, h.y)) - h.z;
    }

    static inline __rtc_device float sdfEllipsoid(const vec3f &p,
                                                const SDFPrimitive &g)
    {
      const vec3f r = g.p1;
      const float k0 = length((p - g.p0) / r);
      const float k1 = length((p - g.p0) / (r * r));
      return k0 * (k0 - 1.f) / k1;
    }

    static inline __rtc_device float sdfPrimitiveDist(const vec3f &p,
                                                    const SDFPrimitive &g)
    {
      switch (static_cast<SDFType>(g.type)) {
      case SDFType::SPHERE:
        return sdfSphere(p, g);
      case SDFType::PILL:
        return sdfPill(p, g);
      case SDFType::CONE_PILL:
      case SDFType::CONE_PILL_SIGMOID:
        return sdfConePill(p, g);
      case SDFType::CONE:
        return sdfCappedCone(p, g);
      case SDFType::TORUS:
        return sdfTorus(p, g);
      case SDFType::CUT_SPHERE:
        return sdfCutSphere(p, g);
      case SDFType::VESICA:
        return sdfVesica(p, g);
      case SDFType::ELLIPSOID:
        return sdfEllipsoid(p, g);
      default:
        return length(p - g.p0) - g.r0;
      }
    }

    static inline __rtc_device float sdfDisplacement(const vec3f &p,
                                                   const vec3f &userParams)
    {
      const float A = userParams.x;
      const float f = userParams.y;
      return A * (0.7f * sin(f * p.x * 0.72f) * sin(f * p.y * 0.65f)
                         * sin(f * p.z * 0.81f)
                  + 0.3f * cos(p.x * 2.12f) * cos(p.y * 2.23f)
                         * cos(p.z * 2.41f));
    }

    static inline __rtc_device float sminPoly(float a, float b, float k)
    {
      const float h = clamp(0.5f + 0.5f * (b - a) / k, 0.f, 1.f);
      return mix(b, a, h) - k * h * (1.f - h);
    }

    static inline __rtc_device box3f computeSDFAABB(const SDFPrimitive &g,
                                                  float padding = 0.f)
    {
      const float disp = g.userParams.x + padding;
      const float r0 = max(0.f, g.r0);
      const float r1 = max(0.f, g.r1);
      switch (static_cast<SDFType>(g.type)) {
      case SDFType::SPHERE:
      case SDFType::CUT_SPHERE:
        return box3f(g.p0 - (r0 + disp), g.p0 + (r0 + disp));
      case SDFType::TORUS:
        return box3f(g.p0 - (r0 + r1 + disp), g.p0 + (r0 + r1 + disp));
      case SDFType::ELLIPSOID:
        return box3f(g.p0 - (g.p1 + disp), g.p0 + (g.p1 + disp));
      default: {
        const float maxR = max(r0, r1) + disp;
        return box3f(min(g.p0, g.p1) - maxR, max(g.p0, g.p1) + maxR);
      }
      }
    }

    static inline __rtc_device float _sdfHash(vec3f p)
    {
      p = sdf_fract(p * vec3f(0.1031f, 0.1030f, 0.0973f));
      p += dot(p, vec3f(p.y + 33.33f, p.z + 33.33f, p.x + 33.33f));
      return sdf_fract((p.x + p.y) * p.z) * 2.f - 1.f;
    }

    static inline __rtc_device float sdfNoise3D(const vec3f &p)
    {
      const vec3f i = sdf_floor(p);
      const vec3f f = sdf_fract(p);
      const vec3f u = f * f * (3.f - 2.f * f);
      return mix(
          mix(mix(_sdfHash(i + vec3f(0, 0, 0)), _sdfHash(i + vec3f(1, 0, 0)), u.x),
              mix(_sdfHash(i + vec3f(0, 1, 0)), _sdfHash(i + vec3f(1, 1, 0)), u.x),
              u.y),
          mix(mix(_sdfHash(i + vec3f(0, 0, 1)), _sdfHash(i + vec3f(1, 0, 1)), u.x),
              mix(_sdfHash(i + vec3f(0, 1, 1)), _sdfHash(i + vec3f(1, 1, 1)), u.x),
              u.y),
          u.z);
    }

    static inline __rtc_device float sdfONoise(const vec3f &p)
    {
      return 0.6f * sdfNoise3D(p)
           + 0.4f * sdfNoise3D(p * 2.3f + vec3f(1.7f, 9.2f, 3.5f));
    }

    static inline __rtc_device float sdfEval(const vec3f &p,
                                           uint32_t primIdx,
                                           const SDFGeometries::DD &data,
                                           float depth)
    {
      const SDFPrimitive &g = data.geometries[primIdx];
      float d = sdfPrimitiveDist(p, g);

      if (g.userParams.x > 0.f && depth < data.distanceFromCamera)
        d += sdfDisplacement(p, g.userParams);

      if (g.numNeighbours > 0 && data.neighbours != nullptr
          && depth < data.blendDistanceFromCamera) {
        const float rMin = min(g.r0, g.r1 >= 0.f ? g.r1 : g.r0);
        const float rMax = max(g.r0, g.r1 >= 0.f ? g.r1 : g.r0);
        const float k =
          mix(rMin, rMax, data.blendLerpFactor) * data.blendFactor;
        for (uint8_t i = 0; i < g.numNeighbours; i++) {
          const uint64_t ni = data.neighbours[g.neighboursIndex + i];
          if (ni >= data.numGeometries)
            continue;
          const float nd = sdfPrimitiveDist(p, data.geometries[ni]);
          d = sminPoly(nd, d, k);
        }
      }

      if (data.noiseFactor > 0.f) {
        const float r0 = max(g.r0, 1e-3f);
        const float amplitude = data.noiseFactor * r0 * 0.1f;
        const float frequency = 2.f / r0;
        d += amplitude * sdfONoise(p * frequency);
      }

      return d;
    }

    static inline __rtc_device
    void bounds(const rtc::TraceInterface &ti,
                const void *geomData,
                owl::common::box3f &bounds,
                const int32_t primID)
    {
      const auto &self = *(const SDFGeometries::DD *)geomData;
      if (!self.geometries || primID < 0
          || (uint32_t)primID >= self.numGeometries) {
        bounds = {};
        return;
      }
      const SDFPrimitive &g = self.geometries[primID];
      float blendK = 0.f;
      if (g.numNeighbours > 0) {
        const float r0 = max(0.f, g.r0);
        const float r1 = max(0.f, g.r1 >= 0.f ? g.r1 : g.r0);
        blendK = mix(min(r0, r1), max(r0, r1), self.blendLerpFactor)
               * self.blendFactor;
      }
      bounds = computeSDFAABB(g, blendK);
    }

    static inline __rtc_device void closestHit(rtc::TraceInterface &ti)
    {
      Ray &ray = *(Ray *)ti.getPRD();
      const auto &self = *(SDFGeometries::DD *)ti.getProgramData();
      const OptixGlobals &globals = OptixGlobals::get(ti);
      const World::DD &world = globals.world;
      const int primID = ti.getPrimitiveIndex();
      const int instID = ti.getInstanceID();
      const float t_hit = ti.getRayTmax();

      const vec3f objectP = ray.P;
      const vec3f objectN = ray.unpackNormal();
      const vec3f worldP = ti.transformPointFromObjectToWorldSpace(objectP);
      const vec3f worldN =
        normalize(ti.transformNormalFromObjectToWorldSpace(objectN));

      render::HitAttributes hitData;
      hitData.primID = primID;
      hitData.instID = instID;
      hitData.t = t_hit;
      hitData.objectPosition = objectP;
      hitData.objectNormal = make_vec4f(objectN);
      hitData.worldPosition = worldP;
      hitData.worldNormal = worldN;

      auto interpolator = [&](const GeometryAttribute::DD &attrib,
                              bool) -> vec4f {
        return attrib.fromArray.valueAt(primID);
      };

      self.setHitAttributes(hitData, interpolator, world, ray.dbg());

      const DeviceMaterial &material = world.materials[self.materialID];
      material.setHit(ray, hitData, world.samplers, ray.dbg());
    }

    static inline __rtc_device void anyHit(rtc::TraceInterface &ti) {}

    static inline __rtc_device void intersect(rtc::TraceInterface &ti)
    {
      Ray &ray = *(Ray *)ti.getPRD();
      const auto &self = *(SDFGeometries::DD *)ti.getProgramData();
      const int primID = ti.getPrimitiveIndex();
      const int instID = ti.getInstanceID();
      const OptixGlobals &globals = OptixGlobals::get(ti);
      const World::DD &world = globals.world;

      if (!self.geometries || primID < 0
          || (uint32_t)primID >= self.numGeometries)
        return;

      const vec3f ro = ti.getObjectRayOrigin();
      const vec3f rd = ti.getObjectRayDirection();

      const SDFPrimitive &prim0 = self.geometries[primID];
      float blendK = 0.f;
      if (prim0.numNeighbours > 0) {
        const float r0 = max(0.f, prim0.r0);
        const float r1 = max(0.f, prim0.r1 >= 0.f ? prim0.r1 : prim0.r0);
        blendK = mix(min(r0, r1), max(r0, r1), self.blendLerpFactor)
               * self.blendFactor;
      }
      const box3f bounds = computeSDFAABB(prim0, blendK);
      float t0 = ti.getRayTmin();
      float t1 = ti.getRayTmax();
      if (!boxTest(t0, t1, bounds, ro, rd))
        return;

      const vec3f primCentre =
        prim0.r1 >= 0.f ? 0.5f * (prim0.p0 + prim0.p1) : prim0.p0;
      const vec3f roWorld = ti.getWorldRayOrigin();
      const float camDist = length(roWorld - primCentre);
      const float lipschitzCorr = 1.f / (1.f + self.noiseFactor * 0.31f);

      const float sdfSign =
        sdf_sign(sdfEval(ro + t0 * rd, primID, self, camDist));

      float t = t0;
      float candidateT = t0;
      float bestError = 1e9f;

      for (uint32_t i = 0; i < self.nbMarchIterations; i++) {
        const vec3f p = ro + t * rd;
        const float radius =
          fabs(sdfSign * sdfEval(p, primID, self, camDist));

        const float stepLength =
          max(radius * self.omega * lipschitzCorr, self.epsilon);

        if (t > 0.f) {
          const float error = radius / t;
          if (error < bestError) {
            bestError = error;
            candidateT = t;
          }
          if (bestError < self.epsilon)
            break;
        }
        if (t > t1)
          break;

        t += stepLength;
        if (t < t0)
          t = t0;
      }

      if (candidateT > t1 || bestError > self.epsilon * 100.f)
        return;

      const vec3f hp = ro + candidateT * rd;
      const float primScale = max(self.geometries[primID].r0, 1e-3f);
      const float e = primScale * 0.01f;
      const vec3f k0(1.f, -1.f, -1.f);
      const vec3f k1(-1.f, -1.f, 1.f);
      const vec3f k2(-1.f, 1.f, -1.f);
      const vec3f k3(1.f, 1.f, 1.f);
      const vec3f N = normalize(
          k0 * sdfEval(hp + e * k0, primID, self, camDist)
        + k1 * sdfEval(hp + e * k1, primID, self, camDist)
        + k2 * sdfEval(hp + e * k2, primID, self, camDist)
        + k3 * sdfEval(hp + e * k3, primID, self, camDist));

      if (OptixGlobals::hitOnInvisibleSide(globals, candidateT, ti))
        return;

      ray.P = hp;
      ray.packNormal(N);

      if (globals.hitIDs) {
        const int rayID = ti.getLaunchIndex().x
                        + ti.getLaunchDims().x * ti.getLaunchIndex().y;
        globals.hitIDs[rayID].primID = primID;
        globals.hitIDs[rayID].instID =
          world.instIDToUserInstID ? world.instIDToUserInstID[instID] : instID;
        globals.hitIDs[rayID].objID = self.userID;
      }

      ti.reportIntersection(candidateT, 0);
    }
#endif
  };

  RTC_EXPORT_USER_GEOM(SDFGeometries, SDFGeometries::DD, SDFGeometriesPrograms,
                       false, true);

} // namespace BARNEY_NS
