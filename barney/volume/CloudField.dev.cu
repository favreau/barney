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
      float planetRadius = cloudSampler.planetRadius;
      float atmosphereThickness = cloudSampler.atmosphereThickness;
      
      // Check ray-sphere intersection for atmospheric shell (between planetRadius and totalRadius)
      vec3f oc = obj_org;
      float totalRadius = planetRadius + atmosphereThickness;
      float a = dot(obj_dir, obj_dir);
      float b = 2.0f * dot(oc, obj_dir);
      
      // Outer sphere intersection
      float c_outer = dot(oc, oc) - totalRadius * totalRadius;
      float discriminant_outer = b * b - 4.0f * a * c_outer;
      if (discriminant_outer < 0.0f)
        return; // No intersection with outer atmosphere sphere
        
      float sqrt_disc_outer = sqrtf(discriminant_outer);
      float t_outer_near = (-b - sqrt_disc_outer) / (2.0f * a);
      float t_outer_far = (-b + sqrt_disc_outer) / (2.0f * a);
      
      // Inner sphere intersection (planet surface)
      float c_inner = dot(oc, oc) - planetRadius * planetRadius;
      float discriminant_inner = b * b - 4.0f * a * c_inner;
      
      range1f tRange = { ti.getRayTmin(), ti.getRayTmax() };
      
      if (discriminant_inner >= 0.0f) {
        // Ray intersects inner planet sphere - exclude interior
        float sqrt_disc_inner = sqrtf(discriminant_inner);
        float t_inner_near = (-b - sqrt_disc_inner) / (2.0f * a);
        float t_inner_far = (-b + sqrt_disc_inner) / (2.0f * a);
        
        // Ray-marching only in atmospheric shell - exclude planet interior
        if (t_inner_near > tRange.lower && t_inner_near < tRange.upper) {
          // Ray enters planet from outside - march until planet surface
          tRange.upper = min(tRange.upper, t_inner_near);
        }
        if (t_inner_far > tRange.lower && t_inner_far < tRange.upper) {
          // Ray exits planet - start marching from planet surface
          tRange.lower = max(tRange.lower, t_inner_far);
        }
        if (t_inner_near <= tRange.lower && t_inner_far >= tRange.upper) {
          // Ray entirely inside planet - no atmospheric ray-marching
          return;
        }
      }
      
      // Constrain to outer atmosphere boundary
      tRange.lower = max(tRange.lower, t_outer_near);
      tRange.upper = min(tRange.upper, t_outer_far);
      
      if (tRange.lower >= tRange.upper)
        return; // No valid intersection range in atmospheric shell
      
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
                  
                  // Ensure we're within the atmospheric shell for this cell
                  vec3f cellOrg = obj_org + cellTRange.lower * obj_dir;
                  vec3f cellEnd = obj_org + cellTRange.upper * obj_dir;
                  float distOrg = length(cellOrg);
                  float distEnd = length(cellEnd);
                  
                  // Skip cell if entirely outside atmosphere
                  if (distOrg > totalRadius && distEnd > totalRadius)
                    return true; // Entire cell is outside atmosphere
                  
                  // Skip cell if entirely inside planet
                  if (distOrg < planetRadius && distEnd < planetRadius)
                    return true; // Entire cell is inside planet
                  
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