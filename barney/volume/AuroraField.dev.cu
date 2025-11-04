// ======================================================================== //
// Copyright 2023-2025 Ingo Wald                                            //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "barney/volume/AuroraField.h"
#include "barney/volume/MCAccelerator.h"
#include "rtcore/TraceInterface.h"

RTC_DECLARE_GLOBALS(BARNEY_NS::render::OptixGlobals);

namespace BARNEY_NS {

  struct MCAccel_Aurora_Programs {
    static inline __rtc_device
    void bounds(const rtc::TraceInterface &ti,
                const void *geomData,
                owl::common::box3f &bounds,  
                const int32_t primID)
    {
#if RTC_DEVICE_CODE
      MCVolumeAccel<AuroraSampler>
        ::boundsProg(ti,geomData,bounds,primID);
#endif
    }
    
    static inline __rtc_device
    void intersect(rtc::TraceInterface &ti)
    {
#if RTC_DEVICE_CODE
      // Custom Aurora field intersection - trace within aurora shell
      const void *pd = ti.getProgramData();
      const MCVolumeAccel<AuroraSampler>::DD &self = 
        *(typename MCVolumeAccel<AuroraSampler>::DD*)pd;
      const render::World::DD &world = render::OptixGlobals::get(ti).world;
      
      // Get ray in object space
      Ray &ray = *(Ray*)ti.getPRD();
      vec3f obj_org = ti.getObjectRayOrigin();
      vec3f obj_dir = ti.getObjectRayDirection();
      
      // Extract Aurora parameters from sampler
      const AuroraSampler::DD &auroraSampler = self.volume.sfSampler;
      float altitudeMin = auroraSampler.altitudeMin;
      float altitudeMax = auroraSampler.altitudeMax;
      
      // Check ray-sphere intersection for aurora shell (between altitudeMin and altitudeMax)
      vec3f oc = obj_org;
      float a = dot(obj_dir, obj_dir);
      float b = 2.0f * dot(oc, obj_dir);
      
      // Outer sphere intersection
      float c_outer = dot(oc, oc) - altitudeMax * altitudeMax;
      float discriminant_outer = b * b - 4.0f * a * c_outer;
      if (discriminant_outer < 0.0f)
        return; // No intersection with outer aurora sphere
        
      float sqrt_disc_outer = sqrtf(discriminant_outer);
      float t_near_outer = (-b - sqrt_disc_outer) / (2.0f * a);
      float t_far_outer = (-b + sqrt_disc_outer) / (2.0f * a);
      
      // Inner sphere intersection (exclude interior)
      float c_inner = dot(oc, oc) - altitudeMin * altitudeMin;
      float discriminant_inner = b * b - 4.0f * a * c_inner;
      
      float t_near = t_near_outer;
      float t_far = t_far_outer;
      
      if (discriminant_inner >= 0.0f) {
        float sqrt_disc_inner = sqrtf(discriminant_inner);
        float t_near_inner = (-b - sqrt_disc_inner) / (2.0f * a);
        float t_far_inner = (-b + sqrt_disc_inner) / (2.0f * a);
        
        // If ray originates inside the shell, clip near to inner sphere exit
        if (t_near_inner > t_near && t_near_inner < t_far) {
          t_near = t_near_inner;
        }
        // If ray would pass through inner sphere, limit to shell only
        if (t_far_inner < t_far && t_far_inner > t_near) {
          t_far = t_far_inner;
        }
      }
      
      // Clamp to valid ray range
      range1f tRange = { max(ti.getRayTmin(), t_near), min(ti.getRayTmax(), t_far) };
      if (tRange.lower >= tRange.upper)
        return; // No valid intersection range
      
      // Convert to macro cell grid space for DDA traversal
      vec3f mcGridOrigin = self.mcGrid.gridOrigin;
      vec3f mcGridSpacing = self.mcGrid.gridSpacing;
      
      vec3f dda_org = (obj_org - mcGridOrigin) * rcp(mcGridSpacing);
      vec3f dda_dir = obj_dir * rcp(mcGridSpacing);
      
      Random rng(ray.rngSeed, hash(ti.getRTCInstanceIndex(),
                                    ti.getGeometryIndex(),
                                    ti.getPrimitiveIndex()));
      
      // DDA traversal through macro cells within aurora shell
      dda::dda3(dda_org, dda_dir, tRange.upper,
                vec3ui(self.mcGrid.dims),
                [&](const vec3i &cellIdx, float t0, float t1) -> bool
                {
                  const float majorant = self.mcGrid.majorant(cellIdx);
                  
                  if (majorant == 0.f) return true;
                  
                  vec4f sample = 0.f;
                  range1f cellTRange = {max(t0, tRange.lower), min(t1, tRange.upper)};
                  
                  // Ensure we're within the aurora shell for this cell
                  vec3f cellOrg = obj_org + cellTRange.lower * obj_dir;
                  vec3f cellEnd = obj_org + cellTRange.upper * obj_dir;
                  float distOrg = length(cellOrg);
                  float distEnd = length(cellEnd);
                  
                  // Skip if entire cell is outside aurora shell
                  if ((distOrg < altitudeMin && distEnd < altitudeMin) ||
                      (distOrg > altitudeMax && distEnd > altitudeMax))
                    return true;
                  
                  if (!Woodcock::sampleRange(sample,
                                           self.volume,
                                           obj_org,
                                           obj_dir,
                                           cellTRange,
                                           majorant,
                                           rng,
                                           ray.dbg())) 
                    return true;
                  
                  vec3f P_obj = obj_org + cellTRange.upper * obj_dir;
                  vec3f P = ti.transformPointFromObjectToWorldSpace(P_obj);
                  
                  // For aurora, normal points radially outward
                  vec3f auroraDirection = normalize(P_obj);
                  
                  ray.setVolumeHit(P, auroraDirection,
                                  cellTRange.upper,
                                  getPos(sample));

                  ti.reportIntersection(cellTRange.upper, 0);
                  return false;
                },
                /*NO debug:*/false
                );
#endif
    }
    
    static inline __rtc_device
    void closestHit(rtc::TraceInterface &ti)
    {
#if RTC_DEVICE_CODE
      // Volume hit already processed in intersect
#endif
    }
    
    static inline __rtc_device
    bool anyHit(rtc::TraceInterface &ti)
    {
#if RTC_DEVICE_CODE
      return true; // Accept all volume hits
#endif
      return true;
    }
  };
  
  RTC_EXPORT_USER_GEOM(AuroraMC,AuroraField::DD,MCAccel_Aurora_Programs,false,false);
} 

