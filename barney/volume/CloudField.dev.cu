// ======================================================================== //
// Copyright 2023-2024 Ingo Wald                                            //
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

#include "barney/volume/CloudField.h"
#include "barney/volume/MCAccelerator.h"
#include "rtcore/TraceInterface.h"

RTC_DECLARE_GLOBALS(BARNEY_NS::render::OptixGlobals);

namespace BARNEY_NS {

  struct MCAccel_Cloud_Programs {
    static inline __rtc_device
    void bounds(const rtc::TraceInterface &ti,
                const void *geomData,
                owl::common::box3f &bounds,  
                const int32_t primID)
    {
#if RTC_DEVICE_CODE
      MCVolumeAccel<CloudSampler>
        ::boundsProg(ti,geomData,bounds,primID);
#endif
    }
    
    static inline __rtc_device
    void intersect(rtc::TraceInterface &ti)
    {
#if RTC_DEVICE_CODE
      // Custom Cloud sphere intersection - only trace within sphere bounds
      const void *pd = ti.getProgramData();
      const MCVolumeAccel<CloudSampler>::DD &self = 
        *(typename MCVolumeAccel<CloudSampler>::DD*)pd;
      const render::World::DD &world = render::OptixGlobals::get(ti).world;
      
      // Get ray in object space
      Ray &ray = *(Ray*)ti.getPRD();
      vec3f obj_org = ti.getObjectRayOrigin();
      vec3f obj_dir = ti.getObjectRayDirection();
      
      // Extract Cloud parameters from sampler
      const CloudSampler::DD &cloudSampler = self.volume.sfSampler;
      float sphereRadius = cloudSampler.sphereRadius;
      float maxHeight = cloudSampler.maxHeight;
      vec3f sphereCenter = cloudSampler.sphereCenter;
      
      // Check ray-sphere intersection for outer cloud boundary
      vec3f oc = obj_org - sphereCenter;
      float totalRadius = sphereRadius + maxHeight;
      float a = dot(obj_dir, obj_dir);
      float b = 2.0f * dot(oc, obj_dir);
      float c = dot(oc, oc) - totalRadius * totalRadius;
      
      float discriminant = b * b - 4.0f * a * c;
      if (discriminant < 0.0f)
        return; // No intersection with cloud sphere
        
      float sqrt_disc = sqrtf(discriminant);
      float t_near = (-b - sqrt_disc) / (2.0f * a);
      float t_far = (-b + sqrt_disc) / (2.0f * a);
      
      // Clamp to valid ray range
      range1f tRange = { max(ti.getRayTmin(), t_near), min(ti.getRayTmax(), t_far) };
      if (tRange.lower >= tRange.upper)
        return; // No valid intersection range
      
      // Convert to macro cell grid space for DDA traversal
      vec3f mcGridOrigin = self.mcGrid.gridOrigin;
      vec3f mcGridSpacing = self.mcGrid.gridSpacing;
      
      vec3f dda_org = (obj_org - mcGridOrigin) * rcp(mcGridSpacing);
      vec3f dda_dir = obj_dir * rcp(mcGridSpacing);
      
      Random rng(ray.rngSeed.next(hash(ti.getRTCInstanceIndex(),
                                       ti.getGeometryIndex(),
                                       ti.getPrimitiveIndex())));
      
      // DDA traversal through macro cells within sphere bounds
      dda::dda3(dda_org, dda_dir, tRange.upper,
                vec3ui(self.mcGrid.dims),
                [&](const vec3i &cellIdx, float t0, float t1) -> bool
                {
                  const float majorant = self.mcGrid.majorant(cellIdx);
                  
                  if (majorant == 0.f) return true;
                  
                  vec4f sample = 0.f;
                  range1f cellTRange = {max(t0, tRange.lower), min(t1, tRange.upper)};
                  
                  // Ensure we're within the sphere for this cell
                  vec3f cellOrg = obj_org + cellTRange.lower * obj_dir;
                  vec3f cellEnd = obj_org + cellTRange.upper * obj_dir;
                  float distOrg = length(cellOrg - sphereCenter);
                  float distEnd = length(cellEnd - sphereCenter);
                  
                  if (distOrg > totalRadius && distEnd > totalRadius)
                    return true; // Entire cell is outside sphere
                  
                  if (!Woodcock::sampleRange(sample,
                                           self.volume,
                                           obj_org,
                                           obj_dir,
                                           cellTRange,
                                           majorant,
                                           rng,
                                           ray.dbg)) 
                    return true;
                  
                  vec3f P_obj = obj_org + cellTRange.upper * obj_dir;
                  vec3f P = ti.transformPointFromObjectToWorldSpace(P_obj);
                  
                  ray.setVolumeHit(P, obj_dir,
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
  
  RTC_EXPORT_USER_GEOM(CloudMC,CloudField::DD,MCAccel_Cloud_Programs,false,false);
} 